#pragma once

#include "ur5_clutter_core.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <future>
#include <limits>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>
#include <tuple>
#include <unordered_map>

namespace {

constexpr int kMaxLoopPlanningAttempts = 5;
constexpr double kExperiencePlanningBudgetSeconds = 120.0;
constexpr double kExperienceExpansionBudgetSeconds = kExperiencePlanningBudgetSeconds;
constexpr unsigned int kExperienceConnectorCandidates = 96;
constexpr unsigned int kExperienceRoadmapConnectorCandidates = 16;
constexpr double kExperienceNodeMergeTolerance = 0.015;
constexpr double kExperienceOptimizerBudgetSeconds = 2.0;
constexpr double kExperienceOptimizerImprovementRatio = 0.98;
constexpr unsigned int kExperienceShortcutCandidatesPerPass = 64;
constexpr double kExecutionStartDriftLogTolerance = 0.03;
constexpr double kExecutionStartJumpLimit = 0.20;
constexpr double kExecutionStartVelocityLogTolerance = 0.05;
constexpr double kExecutionAdvanceTrackingErrorLimit = 0.09;
constexpr double kExecutionMaxSlowdownFactor = 8.0;
constexpr double kLoopExecutionFinalErrorLimit = 0.20;
constexpr double kExecutionPrefixGuardSeconds = 0.75;
constexpr double kExecutionCommandStepLimit =
    kNominalJointSpeedRadPerSecond * kControlPeriodSeconds * 1.5;
constexpr double kExecutionCommandGoalTolerance = 1.0e-6;

struct SharedPlanningStatus {
  std::mutex mutex;
  int attempt = 0;
  int max_attempts = kMaxLoopPlanningAttempts;
  double budget_seconds =
      static_cast<double>(kMaxLoopPlanningAttempts) * kPlanningTimeBudgetSeconds;
  std::string phase = "Queued";
  std::string last_error;
};

struct LoopPlanningResult {
  PlanResult plan;
  int accepted_attempt = 0;
  int max_attempts = kMaxLoopPlanningAttempts;
  double budget_seconds =
      static_cast<double>(kMaxLoopPlanningAttempts) * kPlanningTimeBudgetSeconds;
  unsigned int roadmap_states = 0;
  bool used_cache = false;
  std::string error;
};

struct LoopTransitionKey {
  std::size_t from_index = 0;
  std::size_t to_index = 0;

  bool operator==(const LoopTransitionKey& other) const {
    return from_index == other.from_index && to_index == other.to_index;
  }
};

struct LoopTransitionKeyHash {
  std::size_t operator()(const LoopTransitionKey& key) const {
    return key.from_index ^ (key.to_index + 0x9e3779b97f4a7c15ULL + (key.from_index << 6) +
                             (key.from_index >> 2));
  }
};

struct CachedLoopPath {
  std::vector<JointArray> reusable_path_knots;
  int reuse_count = 0;
};

using LoopPathCache =
    std::unordered_map<LoopTransitionKey, CachedLoopPath, LoopTransitionKeyHash>;

struct RoadmapOptimizerStats {
  std::uint64_t shortcut_edges_added = 0;
  std::uint64_t rrt_attempts = 0;
  std::uint64_t rrt_paths_added = 0;
};

struct LoopExecutionPath {
  std::vector<JointArray> path;
  double current_vs_expected_start = 0.0;
  double current_vs_plan_start = 0.0;
  double start_velocity = 0.0;
};

struct LoopExecutionStats {
  int steps = 0;
  int contact_steps = 0;
  int command_clamp_steps = 0;
  double final_error = 0.0;
  double max_tracking_error = 0.0;
  double max_command_step = 0.0;
  double current_vs_expected_start = 0.0;
  double current_vs_plan_start = 0.0;
  double start_velocity = 0.0;
};

struct LimitedCommandTarget {
  JointArray target{};
  double requested_delta = 0.0;
  bool clamped = false;
};

class RrtConnectExperienceRoadmapLoopPlanner {
 public:
  explicit RrtConnectExperienceRoadmapLoopPlanner(const std::filesystem::path& scene_path)
      : scene_path_(scene_path),
        query_scene_(scene_path),
        query_space_(MakePlanningStateSpace(query_scene_, JointArray{}, false)),
        query_setup_(query_space_) {
    query_setup_.setStateValidityChecker([this](const ob::State* state) {
      return query_scene_.IsStateValid(StateToJoints(state));
    });
    query_setup_.setPlanner(
        MakePlanner(query_setup_.getSpaceInformation(), PlannerKind::kRrtConnect));
    query_setup_.setup();
    optimizer_thread_ = std::thread(&RrtConnectExperienceRoadmapLoopPlanner::OptimizerMain, this);
  }

  ~RrtConnectExperienceRoadmapLoopPlanner() {
    StopOptimizer();
  }

  RrtConnectExperienceRoadmapLoopPlanner(const RrtConnectExperienceRoadmapLoopPlanner&) = delete;
  RrtConnectExperienceRoadmapLoopPlanner& operator=(
      const RrtConnectExperienceRoadmapLoopPlanner&) = delete;

  void BeginExecutionOptimization(const JointArray& start_q,
                                  const JointArray& goal_q,
                                  const std::vector<JointArray>& roadmap_path) {
    std::vector<JointArray> clean_path;
    try {
      RequireFiniteJointArray(start_q, "Optimizer request start");
      RequireFiniteJointArray(goal_q, "Optimizer request goal");
      clean_path = RemoveConsecutiveDuplicateKnots(roadmap_path);
    } catch (const std::exception&) {
      return;
    }
    if (clean_path.size() < 2) {
      return;
    }
    const double clean_path_cost = JointPathLength(clean_path);
    if (!std::isfinite(clean_path_cost) || clean_path_cost <= 0.0) {
      return;
    }
    {
      std::lock_guard lock(optimizer_mutex_);
      optimizer_request_.active = true;
      optimizer_request_.start_q = start_q;
      optimizer_request_.goal_q = goal_q;
      optimizer_request_.roadmap_path = std::move(clean_path);
      optimizer_request_.roadmap_cost = clean_path_cost;
      optimizer_request_.generation = ++optimizer_generation_;
    }
    optimizer_cv_.notify_one();
  }

  void EndExecutionOptimization() {
    {
      std::lock_guard lock(optimizer_mutex_);
      optimizer_request_.active = false;
      ++optimizer_generation_;
    }
    optimizer_cv_.notify_all();
  }

  PlanResult Plan(const JointArray& start_q, const JointArray& goal_q) {
    std::lock_guard query_context_lock(query_context_mutex_);
    using Clock = std::chrono::steady_clock;
    SetStartAndGoal(query_setup_, query_space_, start_q, goal_q);
    query_setup_.getProblemDefinition()->clearSolutionPaths();

    const auto start_time = Clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(kExperiencePlanningBudgetSeconds));

    std::optional<std::vector<JointArray>> graph_path;
    try {
      graph_path = QueryExperienceGraph(start_q, goal_q);
    } catch (const std::exception& e) {
      std::cout << "Experience graph query skipped: " << e.what() << '\n';
    }
    if (graph_path.has_value()) {
      try {
        return MakeFastExperiencePlan(*graph_path, start_q, goal_q, start_time, deadline);
      } catch (const std::exception& e) {
        std::cout << "Experience graph fast path rejected: " << e.what()
                  << "; expanding with RRTConnect\n";
      }
    }

    Ur5Scene expansion_scene(scene_path_);
    PlanResult expanded = PlanPath(expansion_scene,
                                   start_q,
                                   goal_q,
                                   PlannerKind::kRrtConnectExperienceRoadmap,
                                   "RRTConnect experience roadmap expansion",
                                   kExperienceExpansionBudgetSeconds,
                                   true);
    InsertExperiencePath(expanded.path,
                         [this](const JointArray& from, const JointArray& to) {
                           return MotionValid(from, to);
                         });
    return expanded;
  }

  unsigned int RoadmapStateCount() const {
    std::shared_lock lock(roadmap_mutex_);
    return static_cast<unsigned int>(nodes_.size());
  }

  RoadmapOptimizerStats OptimizerStats() const {
    return RoadmapOptimizerStats{
        optimizer_shortcut_edges_added_.load(),
        optimizer_rrt_attempts_.load(),
        optimizer_rrt_paths_added_.load(),
    };
  }

  bool WaitForOptimizerIdle(std::chrono::milliseconds timeout) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (optimizer_inflight_attempts_.load() == 0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return optimizer_inflight_attempts_.load() == 0;
  }

  double NextPlanningBudgetSeconds() const {
    return kExperiencePlanningBudgetSeconds;
  }

 private:
  struct RoadmapSnapshot {
    std::vector<JointArray> nodes;
    std::vector<std::vector<std::pair<std::size_t, double>>> adjacency;
    std::uint64_t version = 0;
  };

  struct OptimizerRequest {
    bool active = false;
    std::uint64_t generation = 0;
    JointArray start_q{};
    JointArray goal_q{};
    std::vector<JointArray> roadmap_path;
    double roadmap_cost = std::numeric_limits<double>::infinity();
  };

  PlanResult MakeFastExperiencePlan(
      const std::vector<JointArray>& graph_knots,
      const JointArray& start_q,
      const JointArray& goal_q,
      std::chrono::steady_clock::time_point start_time,
      std::chrono::steady_clock::time_point deadline) const {
    using Clock = std::chrono::steady_clock;
    std::vector<JointArray> executable_path = SampleLinearPath(graph_knots, kInterpolatedPathStates);
    std::vector<JointArray> reusable_path_knots = graph_knots;
    OrientPathToRequestedEndpoints(executable_path, reusable_path_knots, start_q, goal_q);

    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - start_time).count();
    const double budget_seconds = std::chrono::duration<double>(deadline - start_time).count();

    std::cout << "OMPL planner: " << PlannerName(PlannerKind::kRrtConnectExperienceRoadmap) << '\n';
    std::cout << "OMPL path source: RRTConnect experience roadmap fast path\n";
    std::cout << "OMPL solve attempts: 0\n";
    std::cout << "Experience graph knots: " << graph_knots.size() << '\n';
    std::cout << "Experience graph sampled states: " << executable_path.size() << '\n';
    std::cout << "Experience graph smoothing skipped: true\n";
    std::cout << "Experience graph final collision scan skipped: true\n";
    std::cout << "Planning time budget: " << budget_seconds << " s\n";
    std::cout << "Planning wall time: " << elapsed_ms << " ms\n";

    PlanResult result;
    result.path = std::move(executable_path);
    result.reusable_path_knots = std::move(reusable_path_knots);
    result.used_c2_spline = false;
    result.path_kind = "experience roadmap fast path";
    result.plan_source = "RRTConnect experience roadmap fast path";
    result.solve_attempts = 0;
    result.approximate_attempts_rejected = 0;
    result.spline_repair_iterations = 0;
    result.raw_state_count = graph_knots.size();
    result.simplified_state_count = graph_knots.size();
    result.planning_wall_ms = elapsed_ms;
    return result;
  }

  static double JointPathLength(const std::vector<JointArray>& path) {
    if (path.size() < 2) {
      return 0.0;
    }
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double segment_length = JointDistanceStatic(path[i - 1], path[i]);
      if (!std::isfinite(segment_length)) {
        return std::numeric_limits<double>::infinity();
      }
      length += segment_length;
    }
    return length;
  }

  double JointDistance(const JointArray& a, const JointArray& b) const {
    return JointDistanceStatic(a, b);
  }

  static double JointDistanceStatic(const JointArray& a, const JointArray& b) {
    if (!JointArrayFinite(a) || !JointArrayFinite(b)) {
      return std::numeric_limits<double>::infinity();
    }
    double sum = 0.0;
    for (int i = 0; i < kDof; ++i) {
      const double d = a[i] - b[i];
      sum += d * d;
    }
    return std::sqrt(sum);
  }

  static bool MotionValidWith(const ob::StateSpacePtr& space,
                              og::SimpleSetup& setup,
                              const JointArray& from,
                              const JointArray& to) {
    if (!JointArrayFinite(from) || !JointArrayFinite(to)) {
      return false;
    }
    ob::ScopedState<> from_state(space);
    ob::ScopedState<> to_state(space);
    FillState(from, from_state);
    FillState(to, to_state);
    return setup.getSpaceInformation()->checkMotion(from_state.get(), to_state.get());
  }

  bool MotionValid(const JointArray& from, const JointArray& to) {
    return MotionValidWith(query_space_, query_setup_, from, to);
  }

  RoadmapSnapshot SnapshotRoadmap() const {
    std::shared_lock lock(roadmap_mutex_);
    return RoadmapSnapshot{nodes_, adjacency_, roadmap_version_};
  }

  std::size_t FindOrAddNodeLocked(const JointArray& q) {
    RequireFiniteJointArray(q, "Experience roadmap node");
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      if (MaxJointDelta(nodes_[i], q) <= kExperienceNodeMergeTolerance) {
        return i;
      }
    }
    nodes_.push_back(q);
    adjacency_.emplace_back();
    return nodes_.size() - 1;
  }

  bool AddExperienceEdgeLocked(std::size_t a, std::size_t b) {
    if (a == b) {
      return false;
    }
    const double weight = JointDistance(nodes_[a], nodes_[b]);
    if (!std::isfinite(weight)) {
      return false;
    }
    auto add_one_way = [&](std::size_t from, std::size_t to) {
      for (auto& [existing, existing_weight] : adjacency_[from]) {
        if (existing == to) {
          if (weight + 1e-9 < existing_weight) {
            existing_weight = weight;
            return true;
          }
          return false;
        }
      }
      adjacency_[from].push_back({to, weight});
      return true;
    };
    const bool forward_changed = add_one_way(a, b);
    const bool backward_changed = add_one_way(b, a);
    roadmap_version_ += (forward_changed || backward_changed) ? 1 : 0;
    return forward_changed || backward_changed;
  }

  void AddRoadmapConnectors(
      std::vector<std::size_t> node_indices,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) {
    if (node_indices.empty()) {
      return;
    }
    std::sort(node_indices.begin(), node_indices.end());
    node_indices.erase(std::unique(node_indices.begin(), node_indices.end()), node_indices.end());

    const RoadmapSnapshot snapshot = SnapshotRoadmap();
    std::vector<std::pair<std::size_t, std::size_t>> valid_edges;
    for (const std::size_t node_index : node_indices) {
      if (node_index >= snapshot.nodes.size()) {
        continue;
      }
      std::vector<std::pair<double, std::size_t>> candidates;
      candidates.reserve(snapshot.nodes.size());
      for (std::size_t i = 0; i < snapshot.nodes.size(); ++i) {
        if (i == node_index) {
          continue;
        }
        const double distance = JointDistance(snapshot.nodes[node_index], snapshot.nodes[i]);
        if (std::isfinite(distance)) {
          candidates.push_back({distance, i});
        }
      }
      std::sort(candidates.begin(), candidates.end());

      unsigned int connected = 0;
      for (const auto& [_, target] : candidates) {
        if (connected >= kExperienceRoadmapConnectorCandidates) {
          break;
        }
        if (!motion_valid(snapshot.nodes[node_index], snapshot.nodes[target])) {
          continue;
        }
        valid_edges.push_back({node_index, target});
        ++connected;
      }
    }

    std::unique_lock lock(roadmap_mutex_);
    for (const auto& [from, to] : valid_edges) {
      if (from < nodes_.size() && to < nodes_.size()) {
        AddExperienceEdgeLocked(from, to);
      }
    }
  }

  void InsertExperiencePath(
      const std::vector<JointArray>& path,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) {
    if (path.empty()) {
      return;
    }
    std::vector<JointArray> distinct_path;
    try {
      distinct_path = RemoveConsecutiveDuplicateKnots(path);
    } catch (const std::exception&) {
      return;
    }
    if (distinct_path.size() < 2) {
      return;
    }
    std::vector<std::size_t> path_indices;
    path_indices.reserve(distinct_path.size());
    std::optional<std::size_t> previous;
    {
      std::unique_lock lock(roadmap_mutex_);
      for (const JointArray& q : distinct_path) {
        const std::size_t current = FindOrAddNodeLocked(q);
        path_indices.push_back(current);
        if (previous.has_value()) {
          AddExperienceEdgeLocked(*previous, current);
        }
        previous = current;
      }
    }
    AddRoadmapConnectors(std::move(path_indices), motion_valid);
  }

  bool CommitShortcutEdge(const JointArray& from, const JointArray& to) {
    RequireFiniteJointArray(from, "Experience shortcut start");
    RequireFiniteJointArray(to, "Experience shortcut goal");
    std::unique_lock lock(roadmap_mutex_);
    const std::size_t from_index = FindOrAddNodeLocked(from);
    const std::size_t to_index = FindOrAddNodeLocked(to);
    return AddExperienceEdgeLocked(from_index, to_index);
  }

  void AddTemporaryConnectors(
      const std::vector<JointArray>& nodes,
      const JointArray& query,
      std::size_t query_index,
      std::vector<std::vector<std::pair<std::size_t, double>>>& adjacency,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) const {
    std::vector<std::pair<double, std::size_t>> candidates;
    candidates.reserve(nodes.size());
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const double distance = JointDistance(query, nodes[i]);
      if (std::isfinite(distance)) {
        candidates.push_back({distance, i});
      }
    }
    std::sort(candidates.begin(), candidates.end());

    unsigned int connected = 0;
    for (const auto& [distance, target] : candidates) {
      if (connected >= kExperienceConnectorCandidates) {
        break;
      }
      if (!motion_valid(query, nodes[target])) {
        continue;
      }
      adjacency[query_index].push_back({target, distance});
      adjacency[target].push_back({query_index, distance});
      ++connected;
    }
  }

  std::optional<std::vector<JointArray>> QueryExperienceGraphInSnapshot(
      const RoadmapSnapshot& snapshot,
      const JointArray& start_q,
      const JointArray& goal_q,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) const {
    if (snapshot.nodes.empty()) {
      return std::nullopt;
    }

    const std::size_t start_index = snapshot.nodes.size();
    const std::size_t goal_index = snapshot.nodes.size() + 1;
    std::vector<std::vector<std::pair<std::size_t, double>>> adjacency = snapshot.adjacency;
    adjacency.resize(snapshot.nodes.size() + 2);
    AddTemporaryConnectors(snapshot.nodes, start_q, start_index, adjacency, motion_valid);
    AddTemporaryConnectors(snapshot.nodes, goal_q, goal_index, adjacency, motion_valid);
    if (motion_valid(start_q, goal_q)) {
      const double direct_weight = JointDistance(start_q, goal_q);
      if (std::isfinite(direct_weight)) {
        adjacency[start_index].push_back({goal_index, direct_weight});
        adjacency[goal_index].push_back({start_index, direct_weight});
      }
    }

    const std::size_t vertex_count = adjacency.size();
    const std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
    std::vector<double> distance(vertex_count, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> previous(vertex_count, invalid_index);
    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<>> queue;
    distance[start_index] = 0.0;
    queue.push({0.0, start_index});

    while (!queue.empty()) {
      const auto [current_distance, current] = queue.top();
      queue.pop();
      if (current_distance > distance[current]) {
        continue;
      }
      if (current == goal_index) {
        break;
      }

      for (const auto& [target, edge_cost] : adjacency[current]) {
        if (!std::isfinite(edge_cost)) {
          continue;
        }
        const double candidate = distance[current] + edge_cost;
        if (!std::isfinite(candidate)) {
          continue;
        }
        if (candidate < distance[target]) {
          distance[target] = candidate;
          previous[target] = current;
          queue.push({candidate, target});
        }
      }
    }

    if (previous[goal_index] == invalid_index) {
      return std::nullopt;
    }

    std::vector<std::size_t> path_indices;
    for (std::size_t v = goal_index; v != invalid_index; v = previous[v]) {
      path_indices.push_back(v);
      if (v == start_index) {
        break;
      }
    }
    if (path_indices.empty() || path_indices.back() != start_index) {
      return std::nullopt;
    }
    std::reverse(path_indices.begin(), path_indices.end());

    std::vector<JointArray> joints;
    joints.reserve(path_indices.size());
    for (const std::size_t index : path_indices) {
      if (index == start_index) {
        joints.push_back(start_q);
      } else if (index == goal_index) {
        joints.push_back(goal_q);
      } else {
        joints.push_back(snapshot.nodes[index]);
      }
    }
    return RemoveConsecutiveDuplicateKnots(joints);
  }

  std::optional<std::vector<JointArray>> QueryExperienceGraph(const JointArray& start_q,
                                                              const JointArray& goal_q) {
    const RoadmapSnapshot snapshot = SnapshotRoadmap();
    return QueryExperienceGraphInSnapshot(
        snapshot, start_q, goal_q, [this](const JointArray& from, const JointArray& to) {
          return MotionValid(from, to);
        });
  }

  unsigned int TryShortcutRoadmapPath(
      const OptimizerRequest& request,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) {
    if (request.roadmap_path.size() < 3) {
      return 0;
    }

    unsigned int checked = 0;
    unsigned int added = 0;
    const std::size_t path_size = request.roadmap_path.size();
    for (std::size_t gap = path_size - 1; gap >= 2; --gap) {
      for (std::size_t start = 0; start + gap < path_size; ++start) {
        if (checked >= kExperienceShortcutCandidatesPerPass) {
          return added;
        }
        const std::size_t goal = start + gap;
        const std::vector<JointArray> subpath(request.roadmap_path.begin() + start,
                                              request.roadmap_path.begin() + goal + 1);
        const double existing_length = JointPathLength(subpath);
        const double direct_length =
            JointDistance(request.roadmap_path[start], request.roadmap_path[goal]);
        if (direct_length >= existing_length * kExperienceOptimizerImprovementRatio) {
          continue;
        }
        ++checked;
        if (!motion_valid(request.roadmap_path[start], request.roadmap_path[goal])) {
          continue;
        }
        added += CommitShortcutEdge(request.roadmap_path[start], request.roadmap_path[goal]) ? 1 : 0;
      }
      if (gap == 2) {
        break;
      }
    }
    if (added > 0) {
      optimizer_shortcut_edges_added_.fetch_add(added);
      std::cout << "Roadmap optimizer added shortcut edges: " << added << '\n';
    }
    return added;
  }

  bool TryAddRrtConnectImprovement(
      const OptimizerRequest& request,
      const std::function<bool(const JointArray&, const JointArray&)>& motion_valid) {
    if (stop_optimizer_.load()) {
      return false;
    }

    optimizer_rrt_attempts_.fetch_add(1);
    Ur5Scene optimizer_scene(scene_path_);
    PlanResult candidate = PlanPath(optimizer_scene,
                                    request.start_q,
                                    request.goal_q,
                                    PlannerKind::kRrtConnectExperienceRoadmap,
                                    "RRTConnect experience roadmap background optimizer",
                                    kExperienceOptimizerBudgetSeconds,
                                    true);
    const double candidate_cost = JointPathLength(candidate.path);
    if (candidate.path.size() < 2 || candidate_cost <= 0.0) {
      return false;
    }

    double baseline_cost = request.roadmap_cost;
    const RoadmapSnapshot snapshot = SnapshotRoadmap();
    if (std::optional<std::vector<JointArray>> current_path =
            QueryExperienceGraphInSnapshot(snapshot, request.start_q, request.goal_q, motion_valid)) {
      baseline_cost = std::min(baseline_cost, JointPathLength(*current_path));
    }

    if (candidate_cost >= baseline_cost * kExperienceOptimizerImprovementRatio) {
      return false;
    }

    InsertExperiencePath(candidate.path, motion_valid);
    optimizer_rrt_paths_added_.fetch_add(1);
    std::ostringstream out;
    out << "Roadmap optimizer added shorter RRTConnect path: old_cost=" << baseline_cost
        << " new_cost=" << candidate_cost << " states=" << candidate.path.size() << '\n';
    std::cout << out.str();
    return true;
  }

  void OptimizerMain() {
    try {
      Ur5Scene worker_scene(scene_path_);
      auto worker_space = MakePlanningStateSpace(worker_scene, JointArray{}, false);
      og::SimpleSetup worker_setup(worker_space);
      worker_setup.setStateValidityChecker([&worker_scene](const ob::State* state) {
        return worker_scene.IsStateValid(StateToJoints(state));
      });
      worker_setup.setPlanner(
          MakePlanner(worker_setup.getSpaceInformation(), PlannerKind::kRrtConnect));
      worker_setup.setup();

      auto motion_valid = [&](const JointArray& from, const JointArray& to) {
        return MotionValidWith(worker_space, worker_setup, from, to);
      };

      while (!stop_optimizer_.load()) {
        OptimizerRequest request;
        {
          std::unique_lock lock(optimizer_mutex_);
          optimizer_cv_.wait(lock, [&]() {
            return stop_optimizer_.load() || optimizer_request_.active;
          });
          if (stop_optimizer_.load()) {
            break;
          }
          request = optimizer_request_;
        }

        optimizer_inflight_attempts_.fetch_add(1);
        try {
          const unsigned int shortcut_edges = TryShortcutRoadmapPath(request, motion_valid);
          bool added_rrt_path = false;
          if (!stop_optimizer_.load()) {
            added_rrt_path = TryAddRrtConnectImprovement(request, motion_valid);
          }

          if (shortcut_edges == 0 && !added_rrt_path) {
            std::unique_lock lock(optimizer_mutex_);
            optimizer_cv_.wait_for(lock, std::chrono::milliseconds(150), [&]() {
              return stop_optimizer_.load() || !optimizer_request_.active ||
                     optimizer_request_.generation != request.generation;
            });
          }
        } catch (const std::exception& e) {
          std::cout << "Roadmap optimizer attempt skipped: " << e.what() << '\n';
        }
        optimizer_inflight_attempts_.fetch_sub(1);
      }
    } catch (const std::exception& e) {
      std::cout << "Roadmap optimizer stopped: " << e.what() << '\n';
    }
  }

  void StopOptimizer() {
    stop_optimizer_.store(true);
    optimizer_cv_.notify_all();
    if (optimizer_thread_.joinable()) {
      optimizer_thread_.join();
    }
  }

  std::filesystem::path scene_path_;
  std::mutex query_context_mutex_;
  Ur5Scene query_scene_;
  ob::StateSpacePtr query_space_;
  og::SimpleSetup query_setup_;
  mutable std::shared_mutex roadmap_mutex_;
  std::vector<JointArray> nodes_;
  std::vector<std::vector<std::pair<std::size_t, double>>> adjacency_;
  std::uint64_t roadmap_version_ = 0;
  std::atomic<bool> stop_optimizer_{false};
  std::thread optimizer_thread_;
  std::mutex optimizer_mutex_;
  std::condition_variable optimizer_cv_;
  OptimizerRequest optimizer_request_;
  std::uint64_t optimizer_generation_ = 0;
  std::atomic<std::uint64_t> optimizer_shortcut_edges_added_{0};
  std::atomic<std::uint64_t> optimizer_rrt_attempts_{0};
  std::atomic<std::uint64_t> optimizer_rrt_paths_added_{0};
  std::atomic<std::uint64_t> optimizer_inflight_attempts_{0};
};

std::string FormatJointArray(const JointArray& q) {
  std::ostringstream out;
  out << '[' << std::fixed << std::setprecision(4);
  for (int i = 0; i < kDof; ++i) {
    if (i > 0) {
      out << ", ";
    }
    out << q[i];
  }
  out << ']';
  return out.str();
}

double MaxAbsJointMagnitude(const JointArray& q) {
  if (!JointArrayFinite(q)) {
    return std::numeric_limits<double>::infinity();
  }
  double magnitude = 0.0;
  for (const double value : q) {
    magnitude = std::max(magnitude, std::abs(value));
  }
  return magnitude;
}

LimitedCommandTarget LimitCommandTargetStep(const JointArray& previous_target,
                                            const JointArray& desired_target) {
  RequireFiniteJointArray(previous_target, "Previous command target");
  RequireFiniteJointArray(desired_target, "Desired command target");
  const double requested_delta = MaxJointDelta(previous_target, desired_target);
  if (requested_delta <= kExecutionCommandStepLimit) {
    return LimitedCommandTarget{desired_target, requested_delta, false};
  }
  const double alpha = kExecutionCommandStepLimit / requested_delta;
  return LimitedCommandTarget{
      BlendJoints(previous_target, desired_target, alpha),
      requested_delta,
      true,
  };
}

void LogLoopExecutionContact(Ur5Scene& scene,
                             std::string_view phase,
                             std::size_t from_index,
                             std::size_t to_index,
                             int control_step,
                             double progress,
                             double tracking_error) {
  std::cout << "First loop execution contact: phase=" << phase
            << ", segment=" << from_index + 1 << " -> " << to_index + 1
            << ", step=" << control_step
            << ", progress=" << progress
            << ", tracking_error=" << tracking_error
            << ", min_clearance=" << scene.MinimumObstacleClearance() << '\n';
  const std::vector<std::string> pairs = scene.CurrentObstacleContactPairs();
  for (const std::string& pair : pairs) {
    std::cout << "  contact: " << pair << '\n';
  }
}

void LogLoopExecutionTimeout(Ur5Scene& scene,
                             std::size_t from_index,
                             std::size_t to_index,
                             int steps,
                             double path_progress,
                             const JointArray& last_commanded_target,
                             double last_tracking_error,
                             double max_tracking_error,
                             double max_command_step,
                             int command_clamp_steps) {
  std::cout << "Loop execution timeout: segment " << from_index + 1 << " -> "
            << to_index + 1
            << ", steps=" << steps
            << ", progress=" << path_progress
            << ", last_tracking_error=" << last_tracking_error
            << ", max_tracking_error=" << max_tracking_error
            << ", max_command_step=" << max_command_step
            << ", command_clamp_steps=" << command_clamp_steps
            << ", current_q=" << FormatJointArray(scene.CurrentConfiguration())
            << ", last_commanded_target=" << FormatJointArray(last_commanded_target)
            << '\n';
}

std::string FormatToolPosition(Ur5Scene& scene, const JointArray& q) {
  scene.SetConfiguration(q);
  const auto tool = scene.ToolPosition();
  std::ostringstream out;
  out << '[' << std::fixed << std::setprecision(3) << tool[0] << ", " << tool[1] << ", "
      << tool[2] << ']';
  return out.str();
}

std::string GoalSideLabel(const std::array<double, 3>& tool) {
  if (tool[0] > kRingPlaneX + 0.04) {
    return "needle-side";
  }
  if (tool[0] < kShelfGoalTargetX) {
    return "shelf-side";
  }
  return "ring-plane";
}

std::optional<std::size_t> ShelfWindowIndexForGoal(const std::array<double, 3>& tool) {
  std::optional<std::size_t> best_index;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
    if (std::abs(kWindowSpecs[i].plane_x - kShelfWindowPlaneX) > 1e-9) {
      continue;
    }
    if (!ToolAlignedWithWindow(tool, kWindowSpecs[i])) {
      continue;
    }
    const double distance = std::abs(tool[1] - kWindowSpecs[i].center_y) +
                            std::abs(tool[2] - kWindowSpecs[i].center_z);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

void UpdatePlanningStatus(SharedPlanningStatus& status,
                          int attempt,
                          std::string phase,
                          std::string last_error = {},
                          int max_attempts = kMaxLoopPlanningAttempts,
                          double budget_seconds =
                              static_cast<double>(kMaxLoopPlanningAttempts) *
                              kPlanningTimeBudgetSeconds) {
  std::lock_guard lock(status.mutex);
  status.attempt = attempt;
  status.max_attempts = max_attempts;
  status.budget_seconds = budget_seconds;
  status.phase = std::move(phase);
  status.last_error = std::move(last_error);
}

std::tuple<int, int, double, std::string, std::string> ReadPlanningStatus(
    SharedPlanningStatus& status) {
  std::lock_guard lock(status.mutex);
  return {status.attempt, status.max_attempts, status.budget_seconds, status.phase,
          status.last_error};
}

LoopPlanningResult PlanWithRetries(const std::filesystem::path& scene_path,
                                   const JointArray& start_q,
                                   const JointArray& goal_q,
                                   SharedPlanningStatus& status,
                                   PlannerKind planner_kind,
                                   LoopPathCache* path_cache = nullptr,
                                   std::optional<LoopTransitionKey> cache_key = std::nullopt,
                                   RrtConnectExperienceRoadmapLoopPlanner* experience_planner = nullptr) {
  LoopPlanningResult result;
  if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap && experience_planner != nullptr) {
    const double budget_seconds = experience_planner->NextPlanningBudgetSeconds();
    try {
      UpdatePlanningStatus(status,
                           1,
                           "RRTConnect experience roadmap " +
                               std::to_string(static_cast<int>(kExperienceExpansionBudgetSeconds)) +
                               " s expansion if graph misses",
                           {},
                           1,
                           budget_seconds);
      result.plan = experience_planner->Plan(start_q, goal_q);
      result.accepted_attempt = 1;
      result.max_attempts = 1;
      result.budget_seconds = budget_seconds;
      result.roadmap_states = experience_planner->RoadmapStateCount();
      result.used_cache = false;
      UpdatePlanningStatus(
          status, 1, "RRTConnect experience roadmap query succeeded", {}, 1, budget_seconds);
      return result;
    } catch (const std::exception& e) {
      result.max_attempts = 1;
      result.budget_seconds = budget_seconds;
      result.roadmap_states = experience_planner->RoadmapStateCount();
      result.error = "RRTConnect experience roadmap exhausted " +
                     std::to_string(static_cast<int>(budget_seconds)) +
                     " s without connecting this query; roadmap states=" +
                     std::to_string(result.roadmap_states) + "; " + e.what();
      UpdatePlanningStatus(
          status, 1, "RRTConnect experience roadmap query failed", result.error, 1, budget_seconds);
      return result;
    }
  }

  if (planner_kind == PlannerKind::kRrtConnect && path_cache != nullptr && cache_key.has_value()) {
    const auto cached = path_cache->find(*cache_key);
    if (cached != path_cache->end() && !cached->second.reusable_path_knots.empty()) {
      try {
        UpdatePlanningStatus(status,
                             0,
                             "Improving cached path, reuse " +
                                 std::to_string(cached->second.reuse_count + 1));
        Ur5Scene planning_scene(scene_path);
        result.plan = PlanFromReusablePath(
            planning_scene, start_q, goal_q, cached->second.reusable_path_knots, planner_kind);
        result.accepted_attempt = 1;
        result.used_cache = true;
        cached->second.reusable_path_knots = result.plan.reusable_path_knots;
        ++cached->second.reuse_count;
        UpdatePlanningStatus(status, 0, "Cached path improved");
        return result;
      } catch (const std::exception& e) {
        result.error = e.what();
        path_cache->erase(cached);
        UpdatePlanningStatus(status, 0, "Cached path rejected, replanning", result.error);
      }
    }
  }

  for (int attempt = 1; attempt <= kMaxLoopPlanningAttempts; ++attempt) {
    try {
      UpdatePlanningStatus(status,
                           attempt,
                           "Attempt " + std::to_string(attempt) + "/" +
                               std::to_string(kMaxLoopPlanningAttempts) + ": planning");
      Ur5Scene planning_scene(scene_path);
      result.plan = PlanPath(planning_scene, start_q, goal_q, planner_kind);
      result.accepted_attempt = attempt;
      result.used_cache = false;
      if (planner_kind == PlannerKind::kRrtConnect && path_cache != nullptr &&
          cache_key.has_value() && !result.plan.reusable_path_knots.empty()) {
        (*path_cache)[*cache_key] = CachedLoopPath{result.plan.reusable_path_knots, 0};
      }
      UpdatePlanningStatus(status, attempt, "Planning succeeded");
      return result;
    } catch (const std::exception& e) {
      result.error = e.what();
      UpdatePlanningStatus(status,
                           attempt,
                           "Planning attempt failed, retrying",
                           result.error);
    }
  }
  UpdatePlanningStatus(status, kMaxLoopPlanningAttempts, "Retry limit exhausted", result.error);
  return result;
}

std::vector<JointArray> BuildLoopGoals() {
  // Alternate through generated shelf-side finish-line poses, returning to one
  // robust primary-ring staging pose between shelf targets.
  return {
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {0.181099, -0.834922, 1.194771, -1.182946, 1.054233, 0.014927},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.272575, -0.767697, 1.033570, -1.239197, 0.364753, -0.525439},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.353023, -0.834006, 1.059281, -1.259558, -0.014792, -0.148578},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {0.182364, -0.833842, 1.252389, -1.429893, 1.210342, 0.526325},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.278323, -0.793324, 1.001705, -1.911068, 0.323109, 0.283218},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.346598, -0.881528, 1.170102, -1.202129, 0.318140, -0.647455},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {0.185796, -0.865629, 1.137453, -0.767655, 1.076431, -0.698241},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.081339, -1.030858, 1.346229, -0.776390, 1.276983, -0.607144},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.362126, -0.821684, 1.084173, -1.102080, 0.490087, 0.057136},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {0.148336, -0.892706, 1.085891, -0.505721, 1.140027, 0.082147},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.068444, -0.993667, 1.269892, -0.684621, 1.357602, 0.005784},
      {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
      {-0.360138, -0.899815, 1.264117, -1.706853, 0.388415, 0.139379},
  };
}

void RenderPlanningProgress(LivePidViewer& viewer,
                            SharedPlanningStatus& status,
                            std::string_view demo_title,
                            std::size_t from_index,
                            std::size_t to_index,
                            double elapsed_seconds) {
  const auto [attempt, max_attempts, budget_seconds, phase, last_error] =
      ReadPlanningStatus(status);
  const int percent = std::min(99, static_cast<int>(100.0 * elapsed_seconds / budget_seconds));

  std::ostringstream left;
  left << demo_title << "\n"
       << "Planning to next goal: " << percent << "%\n"
       << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << phase;
  if (attempt > 0) {
    left << "\nAttempt " << attempt << "/" << max_attempts;
  }
  if (!last_error.empty()) {
    left << "\nLast error: " << last_error;
  }

  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

void RenderExecutionStatus(LivePidViewer& viewer,
                           std::string_view demo_title,
                           std::size_t from_index,
                           std::size_t to_index,
                           const PlanResult& plan,
                           int accepted_attempt,
                           int max_attempts,
                           double command_time,
                           double trajectory_duration,
                           int step) {
  const double percent =
      trajectory_duration > 0.0 ? 100.0 * std::min(command_time / trajectory_duration, 1.0) : 100.0;
  std::ostringstream left;
  left << demo_title << "\n"
       << "Executing path (PID): " << std::fixed << std::setprecision(0) << percent << "%\n"
       << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << "Path: " << plan.path_kind << "\n"
       << "Spline fit: " << (plan.used_c2_spline ? "success" : "failed; using linear fallback")
       << "\n"
       << "Plan source: " << plan.plan_source << "\n"
       << "Accepted attempt: " << accepted_attempt << "/" << max_attempts << "\n"
       << "Planning time: " << std::setprecision(1) << plan.planning_wall_ms << " ms\n"
       << "OMPL solve attempts: " << plan.solve_attempts << "\n"
       << "Spline repair iterations: " << plan.spline_repair_iterations << "\n"
       << "Control step: " << step;
  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

void RenderPlanAcceptedStatus(LivePidViewer& viewer,
                              std::string_view demo_title,
                              std::size_t from_index,
                              std::size_t to_index,
                              const PlanResult& plan,
                              int accepted_attempt,
                              int max_attempts) {
  std::ostringstream left;
  left << demo_title << "\n"
       << "Successfully planned geometric path\n"
       << "Simplified geometric path\n";
  if (plan.used_c2_spline) {
    left << "Smoothed final path\n";
  } else {
    left << "Failed to smooth path\n"
         << "Using fallback: " << plan.path_kind << "\n";
  }
  left << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << "Plan source: " << plan.plan_source << "\n"
       << "Accepted attempt: " << accepted_attempt << "/" << max_attempts << "\n"
       << "Planning time: " << std::fixed << std::setprecision(1) << plan.planning_wall_ms << " ms";

  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

LoopExecutionPath PrepareLoopExecutionPath(Ur5Scene& scene,
                                           const JointArray& expected_start_q,
                                           const std::vector<JointArray>& planned_path,
                                           std::size_t from_index,
                                           std::size_t to_index) {
  if (planned_path.empty()) {
    throw std::runtime_error("Cannot execute an empty loop path");
  }

  LoopExecutionPath prepared;
  prepared.path = planned_path;
  for (const JointArray& q : prepared.path) {
    RequireFiniteJointArray(q, "Loop planned path state");
  }
  const JointArray execution_start = scene.CurrentConfiguration();
  RequireFiniteJointArray(execution_start, "Controller scene start state");
  prepared.start_velocity = MaxAbsJointMagnitude(scene.CurrentVelocity());
  prepared.current_vs_expected_start = MaxAbsJointError(execution_start, expected_start_q);
  prepared.current_vs_plan_start = MaxAbsJointError(execution_start, prepared.path.front());
  std::cout << "Loop execution start handoff: segment " << from_index + 1 << " -> "
            << to_index + 1 << ", current_vs_expected="
            << prepared.current_vs_expected_start << " rad, current_vs_plan_start="
            << prepared.current_vs_plan_start << " rad, start_velocity="
            << prepared.start_velocity << " rad/s\n";
  if (prepared.current_vs_expected_start > kExecutionStartDriftLogTolerance) {
    std::cout << "Loop execution start drift: continuing from live controller state without "
                 "jumping back to the nominal start pose\n";
  }
  if (prepared.start_velocity > kExecutionStartVelocityLogTolerance) {
    std::cout << "Loop execution start velocity reset: clearing stale MuJoCo qvel at the "
                 "current controller pose\n";
  }
  if (prepared.current_vs_plan_start > kExecutionStartJumpLimit) {
    throw std::runtime_error("Refusing loop execution because the first path target is too far "
                             "from the controller scene state");
  }
  scene.ResetTo(execution_start);
  prepared.path.front() = execution_start;
  return prepared;
}

void ValidateLoopExecutionPrefix(Ur5Scene& dynamic_guard_scene,
                                 Ur5Scene& command_guard_scene,
                                 const std::vector<JointArray>& executable_path,
                                 std::size_t from_index,
                                 std::size_t to_index) {
  if (executable_path.empty()) {
    throw std::runtime_error("Cannot guard an empty loop path");
  }

  dynamic_guard_scene.ResetTo(executable_path.front());
  dynamic_guard_scene.SetSimulationTimestep(kControlPeriodSeconds);

  JointArray integral{};
  JointArray last_commanded_target = executable_path.front();
  const double trajectory_duration = EstimateExecutionDuration(executable_path);
  const double timestep = kControlPeriodSeconds;
  const int guard_steps = static_cast<int>(
      std::ceil(std::min(kExecutionPrefixGuardSeconds,
                         trajectory_duration + kFinalHoldSeconds) /
                timestep));
  double path_progress = 0.0;

  for (int control_step = 0; control_step < guard_steps; ++control_step) {
    const bool path_complete = path_progress >= 1.0;
    const JointArray desired_target =
        path_complete ? executable_path.back()
                      : EvaluateSampledPath(executable_path, path_progress);
    const LimitedCommandTarget command =
        LimitCommandTargetStep(last_commanded_target, desired_target);

    const std::vector<std::string> command_contacts =
        command_guard_scene.ObstacleContactPairs(command.target);
    if (!command_contacts.empty()) {
      std::cout << "Loop execution prefix guard rejected command target: segment "
                << from_index + 1 << " -> " << to_index + 1
                << ", step=" << control_step
                << ", progress=" << path_progress
                << ", min_clearance=" << command_guard_scene.MinimumObstacleClearance()
                << '\n';
      for (const std::string& pair : command_contacts) {
        std::cout << "  target contact: " << pair << '\n';
      }
      throw std::runtime_error("Loop execution prefix command target is in obstacle contact");
    }

    dynamic_guard_scene.ApplyPidStep(command.target, integral);
    const double tracking_error =
        MaxAbsJointError(dynamic_guard_scene.CurrentConfiguration(), command.target);
    if (dynamic_guard_scene.ObstacleContactCount() > 0) {
      LogLoopExecutionContact(dynamic_guard_scene,
                              "prefix guard",
                              from_index,
                              to_index,
                              control_step,
                              path_progress,
                              tracking_error);
      throw std::runtime_error("Loop execution prefix guard made obstacle contact");
    }

    const bool command_reached_desired =
        !command.clamped &&
        MaxJointDelta(command.target, desired_target) <= kExecutionCommandGoalTolerance;
    last_commanded_target = command.target;

    if (!path_complete && command_reached_desired &&
        tracking_error <= kExecutionAdvanceTrackingErrorLimit) {
      path_progress = std::min(
          1.0,
          path_progress + (trajectory_duration > 0.0 ? timestep / trajectory_duration : 1.0));
    }
  }
}

LoopExecutionStats ExecuteLoopPathHeadless(Ur5Scene& scene,
                                           const JointArray& expected_start_q,
                                           const PlanResult& plan,
                                           std::size_t from_index,
                                           std::size_t to_index,
                                           Ur5Scene* prefix_dynamic_guard_scene = nullptr,
                                           Ur5Scene* prefix_command_guard_scene = nullptr,
                                           std::function<void()> start_background_optimization = {}) {
  const LoopExecutionPath prepared =
      PrepareLoopExecutionPath(scene, expected_start_q, plan.path, from_index, to_index);
  const std::vector<JointArray>& executable_path = prepared.path;
  if (prefix_dynamic_guard_scene != nullptr && prefix_command_guard_scene != nullptr) {
    ValidateLoopExecutionPrefix(*prefix_dynamic_guard_scene,
                                *prefix_command_guard_scene,
                                executable_path,
                                from_index,
                                to_index);
  }

  LoopExecutionStats stats;
  stats.current_vs_expected_start = prepared.current_vs_expected_start;
  stats.current_vs_plan_start = prepared.current_vs_plan_start;
  stats.start_velocity = prepared.start_velocity;
  JointArray integral{};
  const double trajectory_duration = EstimateExecutionDuration(executable_path);
  const double total_execution_duration = trajectory_duration + kFinalHoldSeconds;
  scene.SetSimulationTimestep(kControlPeriodSeconds);
  const double timestep = kControlPeriodSeconds;
  const int final_hold_steps_required =
      static_cast<int>(std::ceil(kFinalHoldSeconds / timestep));
  const int max_control_steps =
      static_cast<int>(std::ceil(total_execution_duration * kExecutionMaxSlowdownFactor / timestep));
  JointArray last_commanded_target = executable_path.front();
  double path_progress = 0.0;
  int final_hold_steps = 0;
  bool completed = false;
  bool background_optimization_started = !start_background_optimization;
  double last_tracking_error = 0.0;
  JointArray last_commanded_target_snapshot = last_commanded_target;
  auto record_contacts = [&](std::string_view phase,
                             int control_step,
                             double progress,
                             double tracking_error) {
    const int contacts = scene.ObstacleContactCount();
    if (contacts > 0) {
      LogLoopExecutionContact(
          scene, phase, from_index, to_index, control_step, progress, tracking_error);
      throw std::runtime_error("Loop execution made obstacle contact");
    }
    return contacts;
  };

  for (int control_step = 0; control_step <= max_control_steps && !completed; ++control_step) {
    const bool path_complete = path_progress >= 1.0;
    const JointArray desired_target =
        path_complete ? executable_path.back()
                      : EvaluateSampledPath(executable_path, path_progress);
    const LimitedCommandTarget command =
        LimitCommandTargetStep(last_commanded_target, desired_target);
    const double command_step = MaxJointDelta(last_commanded_target, command.target);
    stats.max_command_step = std::max(stats.max_command_step, command_step);
    stats.command_clamp_steps += command.clamped ? 1 : 0;

    scene.ApplyPidStep(command.target, integral);
    const double tracking_error = MaxAbsJointError(scene.CurrentConfiguration(), command.target);
    last_tracking_error = tracking_error;
    last_commanded_target_snapshot = command.target;
    stats.max_tracking_error = std::max(stats.max_tracking_error, tracking_error);
    stats.contact_steps +=
        record_contacts("motion", control_step, path_progress, tracking_error) > 0 ? 1 : 0;
    ++stats.steps;

    const bool command_reached_desired =
        !command.clamped &&
        MaxJointDelta(command.target, desired_target) <= kExecutionCommandGoalTolerance;
    last_commanded_target = command.target;

    if (path_complete) {
      if (command_reached_desired && tracking_error <= kExecutionAdvanceTrackingErrorLimit) {
        ++final_hold_steps;
      } else {
        final_hold_steps = 0;
      }
      completed = final_hold_steps >= final_hold_steps_required;
    } else if (command_reached_desired && tracking_error <= kExecutionAdvanceTrackingErrorLimit) {
      path_progress = std::min(
          1.0,
          path_progress + (trajectory_duration > 0.0 ? timestep / trajectory_duration : 1.0));
    }

    if (!background_optimization_started &&
        static_cast<double>(control_step + 1) * timestep >= kExecutionPrefixGuardSeconds) {
      start_background_optimization();
      background_optimization_started = true;
    }
  }

  if (!completed) {
    LogLoopExecutionTimeout(scene,
                            from_index,
                            to_index,
                            stats.steps,
                            path_progress,
                            last_commanded_target_snapshot,
                            last_tracking_error,
                            stats.max_tracking_error,
                            stats.max_command_step,
                            stats.command_clamp_steps);
    throw std::runtime_error("PID controller could not track the loop path within the slowdown limit");
  }

  stats.final_error = MaxAbsJointError(scene.CurrentConfiguration(), executable_path.back());
  std::cout << "Loop segment " << from_index + 1 << " -> " << to_index + 1
            << " headless execution path_kind=\"" << plan.path_kind
            << "\", steps=" << stats.steps
            << ", final_error=" << stats.final_error
            << ", max_tracking_error=" << stats.max_tracking_error
            << ", max_command_step=" << stats.max_command_step
            << ", command_clamp_steps=" << stats.command_clamp_steps
            << ", contact_steps=" << stats.contact_steps << '\n';

  if (stats.final_error > kLoopExecutionFinalErrorLimit) {
    throw std::runtime_error("PID controller did not reach the final loop target closely enough");
  }
  if (stats.contact_steps > 0) {
    throw std::runtime_error("Loop execution made obstacle contact");
  }
  return stats;
}

void ExecuteLoopPath(Ur5Scene& scene,
                     LivePidViewer& viewer,
                     const std::filesystem::path& scene_path,
                     std::string_view demo_title,
                     const JointArray& start_q,
                     const PlanResult& plan,
                     int accepted_attempt,
                     int max_attempts,
                     std::size_t from_index,
                     std::size_t to_index,
                     bool live_realtime,
                     std::function<void()> start_background_optimization = {}) {
  using Clock = std::chrono::steady_clock;

  const LoopExecutionPath prepared =
      PrepareLoopExecutionPath(scene, start_q, plan.path, from_index, to_index);
  const std::vector<JointArray>& executable_path = prepared.path;

  Ur5Scene tool_path_scene(scene_path);
  const std::vector<std::array<double, 3>> planned_tool_path =
      ComputeToolPath(tool_path_scene, executable_path);
  Ur5Scene prefix_dynamic_guard_scene(scene_path);
  Ur5Scene prefix_command_guard_scene(scene_path);
  ValidateLoopExecutionPrefix(prefix_dynamic_guard_scene,
                              prefix_command_guard_scene,
                              executable_path,
                              from_index,
                              to_index);
  viewer.SetPlannedToolPath(planned_tool_path);
  viewer.ResetClock();

  JointArray integral{};
  int step = 0;
  int contact_steps = 0;
  double max_tracking_error = 0.0;
  auto next_render_time = Clock::now();
  const double trajectory_duration = EstimateExecutionDuration(executable_path);
  const double total_execution_duration = trajectory_duration + kFinalHoldSeconds;
  scene.SetSimulationTimestep(kControlPeriodSeconds);
  const double timestep = kControlPeriodSeconds;
  const int final_hold_steps_required =
      static_cast<int>(std::ceil(kFinalHoldSeconds / timestep));
  const int max_control_steps =
      static_cast<int>(std::ceil(total_execution_duration * kExecutionMaxSlowdownFactor / timestep));
  JointArray last_commanded_target = executable_path.front();
  double path_progress = 0.0;
  int final_hold_steps = 0;
  int command_clamp_steps = 0;
  double max_command_step = 0.0;
  bool completed = false;
  bool background_optimization_started = !start_background_optimization;
  double last_tracking_error = 0.0;
  JointArray last_commanded_target_snapshot = last_commanded_target;

  const auto control_start_wall_time = Clock::now();
  for (int control_step = 0;
       control_step <= max_control_steps && viewer.active() && !completed;
       ++control_step) {
    const bool path_complete = path_progress >= 1.0;
    const double command_time =
        path_complete ? trajectory_duration : path_progress * trajectory_duration;
    const JointArray desired_target =
        path_complete ? executable_path.back()
                      : EvaluateSampledPath(executable_path, path_progress);
    const LimitedCommandTarget command =
        LimitCommandTargetStep(last_commanded_target, desired_target);
    const double command_step = MaxJointDelta(last_commanded_target, command.target);
    max_command_step = std::max(max_command_step, command_step);
    command_clamp_steps += command.clamped ? 1 : 0;

    scene.ApplyPidStep(command.target, integral);
    const double tracking_error = MaxAbsJointError(scene.CurrentConfiguration(), command.target);
    last_tracking_error = tracking_error;
    last_commanded_target_snapshot = command.target;
    max_tracking_error = std::max(max_tracking_error, tracking_error);
    const int obstacle_contacts = scene.ObstacleContactCount();
    if (obstacle_contacts > 0) {
      LogLoopExecutionContact(
          scene, "motion", from_index, to_index, control_step, path_progress, tracking_error);
      throw std::runtime_error("Loop execution made obstacle contact");
    }
    contact_steps += obstacle_contacts > 0 ? 1 : 0;
    ++step;

    const bool command_reached_desired =
        !command.clamped &&
        MaxJointDelta(command.target, desired_target) <= kExecutionCommandGoalTolerance;
    last_commanded_target = command.target;

    if (path_complete) {
      if (command_reached_desired && tracking_error <= kExecutionAdvanceTrackingErrorLimit) {
        ++final_hold_steps;
      } else {
        final_hold_steps = 0;
      }
      completed = final_hold_steps >= final_hold_steps_required;
    } else if (command_reached_desired && tracking_error <= kExecutionAdvanceTrackingErrorLimit) {
      path_progress =
          std::min(1.0, path_progress + (trajectory_duration > 0.0 ? timestep / trajectory_duration : 1.0));
    }

    if (!background_optimization_started &&
        static_cast<double>(control_step + 1) * timestep >= kExecutionPrefixGuardSeconds) {
      start_background_optimization();
      background_optimization_started = true;
    }

    const auto now = Clock::now();
    if (now >= next_render_time || completed) {
      RenderExecutionStatus(viewer,
                            demo_title,
                            from_index,
                            to_index,
                            plan,
                            accepted_attempt,
                            max_attempts,
                            command_time,
                            trajectory_duration,
                            step);
      next_render_time = now + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(1.0 / kLiveRenderHz));
    }

    if (live_realtime) {
      const auto target_time = control_start_wall_time +
                               std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(
                                       static_cast<double>(control_step + 1) * timestep));
      const auto before_sleep = Clock::now();
      if (target_time > before_sleep) {
        std::this_thread::sleep_until(target_time);
      }
    }
  }

  if (viewer.active() && !completed) {
    LogLoopExecutionTimeout(scene,
                            from_index,
                            to_index,
                            step,
                            path_progress,
                            last_commanded_target_snapshot,
                            last_tracking_error,
                            max_tracking_error,
                            max_command_step,
                            command_clamp_steps);
    throw std::runtime_error("PID controller could not track the loop path within the slowdown limit");
  }

  const double final_error = MaxAbsJointError(scene.CurrentConfiguration(), executable_path.back());
  std::cout << "Loop segment " << from_index + 1 << " -> " << to_index + 1
            << " executed path_kind=\"" << plan.path_kind << "\", accepted_attempt="
            << accepted_attempt << ", planning_wall_ms=" << plan.planning_wall_ms
            << ", steps=" << step
            << ", final_error=" << final_error << ", max_tracking_error=" << max_tracking_error
            << ", max_command_step=" << max_command_step
            << ", command_clamp_steps=" << command_clamp_steps
            << ", contact_steps=" << contact_steps << '\n';

  if (final_error > kLoopExecutionFinalErrorLimit) {
    throw std::runtime_error("PID controller did not reach the final loop target closely enough");
  }
  if (contact_steps > 0) {
    throw std::runtime_error("Loop execution made obstacle contact");
  }
}

}  // namespace

int RunGoalLoopDemo(int argc, char** argv, PlannerKind planner_kind, std::string_view demo_title) {
  bool live_realtime = true;
  bool check_goals = false;
  bool check_transitions = false;
  bool check_cache = false;
  bool check_optimizer = false;
  bool check_execution = false;
  bool check_execution_async_planning = false;
  std::size_t check_execution_cycles = 1;
  std::chrono::milliseconds check_execution_handoff_delay{20};
  std::optional<std::size_t> live_segment_limit;
  std::optional<std::filesystem::path> scene_arg;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--live-fast") {
      live_realtime = false;
    } else if (arg == "--live-realtime") {
      live_realtime = true;
    } else if (arg == "--check-goals") {
      check_goals = true;
    } else if (arg == "--check-transitions") {
      check_goals = true;
      check_transitions = true;
    } else if (arg == "--check-cache") {
      check_goals = true;
      check_cache = true;
    } else if (arg == "--check-optimizer") {
      check_goals = true;
      check_optimizer = true;
    } else if (arg == "--check-execution") {
      check_goals = true;
      check_execution = true;
    } else if (arg == "--check-execution-stress") {
      check_goals = true;
      check_execution = true;
      check_execution_cycles = 6;
    } else if (arg == "--check-execution-live-handoff") {
      check_goals = true;
      check_execution = true;
      check_execution_cycles = 6;
      check_execution_handoff_delay = std::chrono::milliseconds(300);
    } else if (arg == "--check-execution-live-loop") {
      check_goals = true;
      check_execution = true;
      check_execution_async_planning = true;
      check_execution_cycles = 6;
      check_execution_handoff_delay = std::chrono::milliseconds(300);
    } else if (arg == "--check-execution-cycles") {
      if (i + 1 >= argc) {
        std::cerr << "error: --check-execution-cycles requires a positive integer\n";
        return 1;
      }
      try {
        check_execution_cycles = std::stoul(argv[++i]);
      } catch (const std::exception&) {
        std::cerr << "error: --check-execution-cycles requires a positive integer\n";
        return 1;
      }
      if (check_execution_cycles == 0) {
        std::cerr << "error: --check-execution-cycles requires a positive integer\n";
        return 1;
      }
      check_goals = true;
      check_execution = true;
    } else if (arg == "--live-cycles") {
      if (i + 1 >= argc) {
        std::cerr << "error: --live-cycles requires a positive integer\n";
        return 1;
      }
      try {
        live_segment_limit = std::stoul(argv[++i]) * BuildLoopGoals().size();
      } catch (const std::exception&) {
        std::cerr << "error: --live-cycles requires a positive integer\n";
        return 1;
      }
      if (*live_segment_limit == 0) {
        std::cerr << "error: --live-cycles requires a positive integer\n";
        return 1;
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--live-fast|--live-realtime|--live-cycles N|--check-goals|--check-transitions|--check-cache|--check-optimizer|--check-execution|--check-execution-stress|--check-execution-live-handoff|--check-execution-live-loop|--check-execution-cycles N]"
                   " [scene.xml]\n";
      return 0;
    } else if (!scene_arg.has_value()) {
      scene_arg = std::filesystem::path(arg);
    } else {
      std::cerr << "error: unexpected argument: " << arg << '\n';
      return 1;
    }
  }

  const std::filesystem::path scene_path =
      scene_arg.value_or(std::filesystem::path(MOTION_PLANNING_DEFAULT_SCENE));

  try {
    Ur5Scene scene(scene_path);
    const std::vector<JointArray> goals = BuildLoopGoals();
    std::cout << "Loaded scene: " << scene_path << '\n';
    std::cout << "Robot model: " << scene.robot_label() << '\n';
    std::cout << "Control mode: " << scene.ControlModeLabel() << '\n';
    std::cout << "Loop goals: " << goals.size() << '\n';
    int needle_side_goals = 0;
    int shelf_side_goals = 0;
    std::array<int, kShelfWindowCount> shelf_window_goals{};
    bool all_goals_valid = true;
    bool alternating_sides = true;
    std::string previous_side;
    for (std::size_t i = 0; i < goals.size(); ++i) {
      scene.SetConfiguration(goals[i]);
      const auto tool = scene.ToolPosition();
      const std::string side = GoalSideLabel(tool);
      const bool valid = scene.IsStateValid(goals[i]);
      needle_side_goals += side == "needle-side" ? 1 : 0;
      shelf_side_goals += side == "shelf-side" ? 1 : 0;
      if (side == "shelf-side") {
        const std::optional<std::size_t> shelf_window = ShelfWindowIndexForGoal(tool);
        if (shelf_window.has_value() && *shelf_window >= kShelfWindowStartIndex) {
          ++shelf_window_goals[*shelf_window - kShelfWindowStartIndex];
        }
      }
      all_goals_valid = valid && all_goals_valid;
      alternating_sides = (previous_side.empty() || previous_side != side) && alternating_sides;
      previous_side = side;
      std::cout << "  goal " << i + 1 << " side=" << side << " valid=" << std::boolalpha
                << valid << " q=" << FormatJointArray(goals[i])
                << " tool=" << FormatToolPosition(scene, goals[i]) << '\n';
    }
    std::cout << "Needle-side loop goals: " << needle_side_goals << '\n';
    std::cout << "Shelf-side loop goals: " << shelf_side_goals << '\n';
    for (std::size_t window = 0; window < kShelfWindowCount; ++window) {
      std::cout << kWindowSpecs[kShelfWindowStartIndex + window].name
                << " goals: " << shelf_window_goals[window] << '\n';
    }
    std::cout << "Loop goals alternate sides: " << std::boolalpha << alternating_sides << '\n';
    std::cout << "Loop goals valid: " << std::boolalpha << all_goals_valid << '\n';

    if (check_goals) {
      bool shelf_windows_balanced = true;
      for (const int window_goal_count : shelf_window_goals) {
        shelf_windows_balanced = shelf_windows_balanced && window_goal_count == 4;
      }
      if (goals.size() != 24 || needle_side_goals != 12 || shelf_side_goals != 12 ||
          !shelf_windows_balanced || !alternating_sides || !all_goals_valid) {
        return 1;
      }
      if (check_cache) {
        LoopPathCache path_cache;
        std::optional<RrtConnectExperienceRoadmapLoopPlanner> experience_planner;
        if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
          experience_planner.emplace(scene_path);
        }
        SharedPlanningStatus first_status;
        SharedPlanningStatus second_status;
        const LoopTransitionKey cache_key{0, 1};
        const LoopPlanningResult first =
            PlanWithRetries(scene_path,
                            goals[0],
                            goals[1],
                            first_status,
                            planner_kind,
                            planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
                            cache_key,
                            experience_planner ? &*experience_planner : nullptr);
        LoopPlanningResult second;
        if (first.accepted_attempt != 0) {
          second = PlanWithRetries(scene_path,
                                   goals[0],
                                   goals[1],
                                   second_status,
                                   planner_kind,
                                   planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
                                   cache_key,
                                   experience_planner ? &*experience_planner : nullptr);
        }
        std::cout << "cache smoke first_used_cache=" << std::boolalpha << first.used_cache
                  << " second_used_cache=" << second.used_cache
                  << " cached_entries=" << path_cache.size()
                  << " first_source=\"" << first.plan.plan_source << "\""
                  << " second_source=\"" << second.plan.plan_source << "\"";
        if (experience_planner) {
          std::cout << " experience_roadmap_states=" << experience_planner->RoadmapStateCount();
        }
        std::cout << '\n';
        if (planner_kind == PlannerKind::kRrtConnect &&
            (first.accepted_attempt == 0 || second.accepted_attempt == 0 || first.used_cache ||
             !second.used_cache || path_cache.size() != 1)) {
          return 1;
        }
        if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap &&
            (first.accepted_attempt == 0 || second.accepted_attempt == 0 || first.used_cache ||
             second.used_cache || path_cache.size() != 0 ||
             first.plan.plan_source != "RRTConnect experience roadmap expansion" ||
             second.plan.plan_source != "RRTConnect experience roadmap fast path" ||
             experience_planner->RoadmapStateCount() == 0)) {
          return 1;
        }
        if (planner_kind != PlannerKind::kRrtConnect && planner_kind != PlannerKind::kRrtConnectExperienceRoadmap &&
            second.used_cache) {
          return 1;
        }
      }
      if (check_optimizer) {
        if (planner_kind != PlannerKind::kRrtConnectExperienceRoadmap) {
          std::cerr << "error: --check-optimizer only applies to the RRTConnect experience roadmap demo\n";
          return 1;
        }

        RrtConnectExperienceRoadmapLoopPlanner experience_planner(scene_path);
        SharedPlanningStatus first_status;
        SharedPlanningStatus second_status;
        const LoopTransitionKey cache_key{0, 1};
        const LoopPlanningResult first =
            PlanWithRetries(scene_path,
                            goals[0],
                            goals[1],
                            first_status,
                            planner_kind,
                            nullptr,
                            cache_key,
                            &experience_planner);
        if (first.accepted_attempt == 0 || first.plan.path.empty()) {
          std::cerr << "optimizer smoke initial plan failed: " << first.error << '\n';
          return 1;
        }
        const LoopPlanningResult second =
            PlanWithRetries(scene_path,
                            goals[0],
                            goals[1],
                            second_status,
                            planner_kind,
                            nullptr,
                            cache_key,
                            &experience_planner);
        if (second.accepted_attempt == 0 || second.plan.path.empty()) {
          std::cerr << "optimizer smoke roadmap query failed: " << second.error << '\n';
          return 1;
        }

        const std::vector<JointArray>& optimizer_path =
            second.plan.reusable_path_knots.empty() ? second.plan.path
                                                    : second.plan.reusable_path_knots;
        const RoadmapOptimizerStats before = experience_planner.OptimizerStats();
        experience_planner.BeginExecutionOptimization(goals[0], goals[1], optimizer_path);

        RoadmapOptimizerStats after = before;
        for (int poll = 0; poll < 50; ++poll) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          after = experience_planner.OptimizerStats();
          if (after.shortcut_edges_added > before.shortcut_edges_added ||
              after.rrt_attempts > before.rrt_attempts ||
              after.rrt_paths_added > before.rrt_paths_added) {
            break;
          }
        }
        experience_planner.EndExecutionOptimization();
        experience_planner.WaitForOptimizerIdle(
            std::chrono::milliseconds(static_cast<int>((kExperienceOptimizerBudgetSeconds + 1.0) *
                                                       1000.0)));
        after = experience_planner.OptimizerStats();

        std::cout << "optimizer smoke shortcut_edges_added="
                  << after.shortcut_edges_added - before.shortcut_edges_added
                  << " rrt_attempts=" << after.rrt_attempts - before.rrt_attempts
                  << " rrt_paths_added=" << after.rrt_paths_added - before.rrt_paths_added
                  << " roadmap_states=" << experience_planner.RoadmapStateCount() << '\n';
        if (after.shortcut_edges_added == before.shortcut_edges_added &&
            after.rrt_attempts == before.rrt_attempts &&
            after.rrt_paths_added == before.rrt_paths_added) {
          return 1;
        }
      }
      if (check_execution) {
        Ur5Scene execution_scene(scene_path);
        Ur5Scene prefix_dynamic_guard_scene(scene_path);
        Ur5Scene prefix_command_guard_scene(scene_path);
        execution_scene.ResetTo(goals.front());
        LoopPathCache path_cache;
        std::optional<RrtConnectExperienceRoadmapLoopPlanner> experience_planner;
        if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
          experience_planner.emplace(scene_path);
        }

        double max_handoff_error = 0.0;
        double max_first_target_jump = 0.0;
        double max_start_velocity = 0.0;
        double max_final_error = 0.0;
        double max_tracking_error = 0.0;
        int total_control_steps = 0;
        int max_control_steps = 0;
        int total_contact_steps = 0;
        const std::size_t execution_cycles = check_execution_cycles;
        const std::size_t execution_segments = goals.size() * execution_cycles;
        for (std::size_t segment = 0; segment < execution_segments; ++segment) {
          const std::size_t i = segment % goals.size();
          const std::size_t next = (i + 1) % goals.size();
          const JointArray segment_start_q = execution_scene.CurrentConfiguration();
          RequireFiniteJointArray(segment_start_q, "Headless execution planning start");
          SharedPlanningStatus status;
          const LoopTransitionKey cache_key{i, next};
          auto plan_segment = [&]() {
            return PlanWithRetries(scene_path,
                                   segment_start_q,
                                   goals[next],
                                   status,
                                   planner_kind,
                                   planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
                                   cache_key,
                                   experience_planner ? &*experience_planner : nullptr);
          };
          LoopPlanningResult result;
          if (check_execution_async_planning) {
            auto future = std::async(std::launch::async, plan_segment);
            while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
              static_cast<void>(ReadPlanningStatus(status));
              std::this_thread::sleep_for(std::chrono::milliseconds(16));
            }
            result = future.get();
          } else {
            result = plan_segment();
          }
          if (result.accepted_attempt == 0 || result.plan.path.empty()) {
            std::cerr << "execution transition " << i + 1 << " -> " << next + 1
                      << " failed to plan: " << result.error << '\n';
            return 1;
          }

          std::function<void()> start_background_optimization;
          if (experience_planner) {
            const std::vector<JointArray> optimizer_path =
                result.plan.reusable_path_knots.empty() ? result.plan.path
                                                        : result.plan.reusable_path_knots;
            RrtConnectExperienceRoadmapLoopPlanner* planner = &*experience_planner;
            const JointArray optimizer_start_q = segment_start_q;
            const JointArray optimizer_goal_q = goals[next];
            auto begin_background_optimization =
                [planner, optimizer_start_q, optimizer_goal_q, optimizer_path]() {
                  planner->BeginExecutionOptimization(
                      optimizer_start_q, optimizer_goal_q, optimizer_path);
                };
            if (check_execution_async_planning) {
              start_background_optimization = begin_background_optimization;
            } else {
              begin_background_optimization();
            }
            std::this_thread::sleep_for(check_execution_handoff_delay);
          }

          LoopExecutionStats stats;
          try {
            stats = ExecuteLoopPathHeadless(execution_scene,
                                            segment_start_q,
                                            result.plan,
                                            i,
                                            next,
                                            &prefix_dynamic_guard_scene,
                                            &prefix_command_guard_scene,
                                            std::move(start_background_optimization));
          } catch (...) {
            if (experience_planner) {
              experience_planner->EndExecutionOptimization();
            }
            throw;
          }
          if (experience_planner) {
            experience_planner->EndExecutionOptimization();
          }

          max_handoff_error = std::max(max_handoff_error, stats.current_vs_expected_start);
          max_first_target_jump = std::max(max_first_target_jump, stats.current_vs_plan_start);
          max_start_velocity = std::max(max_start_velocity, stats.start_velocity);
          max_final_error = std::max(max_final_error, stats.final_error);
          max_tracking_error = std::max(max_tracking_error, stats.max_tracking_error);
          total_control_steps += stats.steps;
          max_control_steps = std::max(max_control_steps, stats.steps);
          total_contact_steps += stats.contact_steps;
        }

        std::cout << "headless execution smoke segments=" << execution_segments
                  << " cycles=" << execution_cycles
                  << " handoff_delay_ms=" << check_execution_handoff_delay.count()
                  << " async_planning=" << std::boolalpha << check_execution_async_planning
                  << " max_handoff_error=" << max_handoff_error
                  << " max_first_target_jump=" << max_first_target_jump
                  << " max_start_velocity=" << max_start_velocity
                  << " max_final_error=" << max_final_error
                  << " max_tracking_error=" << max_tracking_error
                  << " total_control_steps=" << total_control_steps
                  << " max_control_steps=" << max_control_steps
                  << " total_contact_steps=" << total_contact_steps;
        if (experience_planner) {
          const RoadmapOptimizerStats stats = experience_planner->OptimizerStats();
          std::cout << " optimizer_rrt_attempts=" << stats.rrt_attempts
                    << " optimizer_shortcuts=" << stats.shortcut_edges_added
                    << " optimizer_paths=" << stats.rrt_paths_added;
        }
        std::cout << '\n';
      }
      if (check_transitions) {
        std::array<int, kWindowSpecs.size()> transition_window_hits{};
        LoopPathCache path_cache;
        std::optional<RrtConnectExperienceRoadmapLoopPlanner> experience_planner;
        if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
          experience_planner.emplace(scene_path);
        }
        for (std::size_t i = 0; i < goals.size(); ++i) {
          const std::size_t next = (i + 1) % goals.size();
          SharedPlanningStatus status;
          const LoopTransitionKey cache_key{i, next};
          const LoopPlanningResult result =
              PlanWithRetries(scene_path,
                              goals[i],
                              goals[next],
                              status,
                              planner_kind,
                              planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
                              cache_key,
                              experience_planner ? &*experience_planner : nullptr);
          if (result.accepted_attempt == 0 || result.plan.path.empty()) {
            std::cerr << "transition " << i + 1 << " -> " << next + 1
                      << " failed: " << result.error << '\n';
            return 1;
          }
          const std::array<int, kWindowSpecs.size()> window_hits =
              WindowHitCounts(scene, result.plan.path);
          for (std::size_t window = 0; window < kWindowSpecs.size(); ++window) {
            transition_window_hits[window] += window_hits[window] > 0 ? 1 : 0;
          }
          std::cout << "transition " << i + 1 << " -> " << next + 1
                    << " planned path=\"" << result.plan.path_kind
                    << "\" source=\"" << result.plan.plan_source
                    << "\" accepted_attempt=" << result.accepted_attempt
                    << " used_cache=" << std::boolalpha << result.used_cache
                    << " planning_wall_ms=" << result.plan.planning_wall_ms << '\n';
        }
        for (std::size_t window = 0; window < kWindowSpecs.size(); ++window) {
          std::cout << "transition coverage " << kWindowSpecs[window].name << ": "
                    << transition_window_hits[window] << '\n';
          if (transition_window_hits[window] == 0) {
            return 1;
          }
        }
      }
      return 0;
    }

    std::size_t current_index = 0;
    scene.ResetTo(goals[current_index]);
    LivePidViewer viewer(scene, {}, true);
    if (live_segment_limit.has_value() && !viewer.active()) {
      std::cerr << "error: live viewer is unavailable, cannot run bounded live cycle check\n";
      return 1;
    }
    viewer.RenderStatus(std::string(demo_title) + "\nStarting at goal 1", "Esc: stop");
    LoopPathCache path_cache;
    std::optional<RrtConnectExperienceRoadmapLoopPlanner> experience_planner;
    if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
      experience_planner.emplace(scene_path);
    }

    std::size_t executed_live_segments = 0;
    while (viewer.active() &&
           (!live_segment_limit.has_value() || executed_live_segments < *live_segment_limit)) {
      const std::size_t next_index = (current_index + 1) % goals.size();
      const JointArray planning_start_q = scene.CurrentConfiguration();
      RequireFiniteJointArray(planning_start_q, "Live planning start");
      viewer.SetGoalGhost(goals[next_index]);
      SharedPlanningStatus status;
      const auto planning_start = std::chrono::steady_clock::now();
      auto future = std::async(std::launch::async, [&]() {
        const LoopTransitionKey cache_key{current_index, next_index};
        return PlanWithRetries(
            scene_path,
            planning_start_q,
            goals[next_index],
            status,
            planner_kind,
            planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
            cache_key,
            experience_planner ? &*experience_planner : nullptr);
      });

      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
             viewer.active()) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - planning_start)
                .count();
        RenderPlanningProgress(viewer, status, demo_title, current_index, next_index, elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }
      if (!viewer.active()) {
        break;
      }

      LoopPlanningResult planning = future.get();
      if (planning.accepted_attempt == 0 || planning.plan.path.empty()) {
        std::cerr << "error: loop planning failed\n";
        std::cerr << "start_index=" << current_index + 1
                  << " start_q=" << FormatJointArray(planning_start_q) << '\n';
        std::cerr << "goal_index=" << next_index + 1
                  << " goal_q=" << FormatJointArray(goals[next_index]) << '\n';
        std::cerr << "last_error=" << planning.error << '\n';
        std::ostringstream left;
        left << demo_title << "\n";
        if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
          left << "RRTConnect experience roadmap budget exhausted\n"
               << "Segment " << current_index + 1 << " -> " << next_index + 1 << "\n"
               << "Budget: " << std::fixed << std::setprecision(0) << planning.budget_seconds
               << " s\n"
               << "Roadmap states: " << planning.roadmap_states << "\n";
        } else {
          left << "Planning failed\n"
               << "Segment " << current_index + 1 << " -> " << next_index + 1 << "\n";
        }
        left << "See terminal for start/goal q";
        viewer.RenderStatus(left.str(), "Esc: close");
        while (viewer.active()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          viewer.RenderStatus(left.str(), "Esc: close");
        }
        return 1;
      }

      RenderPlanAcceptedStatus(viewer,
                               demo_title,
                               current_index,
                               next_index,
                               planning.plan,
                               planning.accepted_attempt,
                               planning.max_attempts);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      std::function<void()> start_background_optimization;
      if (experience_planner) {
        const std::vector<JointArray> optimizer_path =
            planning.plan.reusable_path_knots.empty() ? planning.plan.path
                                                      : planning.plan.reusable_path_knots;
        RrtConnectExperienceRoadmapLoopPlanner* planner = &*experience_planner;
        const JointArray optimizer_start_q = planning_start_q;
        const JointArray optimizer_goal_q = goals[next_index];
        start_background_optimization =
            [planner, optimizer_start_q, optimizer_goal_q, optimizer_path]() {
              planner->BeginExecutionOptimization(
                  optimizer_start_q, optimizer_goal_q, optimizer_path);
            };
      }

      try {
        ExecuteLoopPath(scene,
                        viewer,
                        scene_path,
                        demo_title,
                        planning_start_q,
                        planning.plan,
                        planning.accepted_attempt,
                        planning.max_attempts,
                        current_index,
                        next_index,
                        live_realtime,
                        std::move(start_background_optimization));
      } catch (...) {
        if (experience_planner) {
          experience_planner->EndExecutionOptimization();
        }
        throw;
      }
      if (experience_planner) {
        experience_planner->EndExecutionOptimization();
      }
      current_index = next_index;
      ++executed_live_segments;
    }
    if (live_segment_limit.has_value()) {
      std::cout << "bounded live execution segments=" << executed_live_segments
                << " requested_segments=" << *live_segment_limit << '\n';
      if (executed_live_segments < *live_segment_limit) {
        return 1;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
