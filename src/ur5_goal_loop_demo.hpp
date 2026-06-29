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

class RrtConnectExperienceRoadmapLoopPlanner {
 public:
  explicit RrtConnectExperienceRoadmapLoopPlanner(const std::filesystem::path& scene_path)
      : scene_path_(scene_path),
        scene_(scene_path),
        space_(MakePlanningStateSpace(scene_, JointArray{}, false)),
        setup_(space_) {
    setup_.setStateValidityChecker([this](const ob::State* state) {
      return scene_.IsStateValid(StateToJoints(state));
    });
    setup_.setPlanner(MakePlanner(setup_.getSpaceInformation(), PlannerKind::kRrtConnect));
    setup_.setup();
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
    if (roadmap_path.size() < 2) {
      return;
    }
    {
      std::lock_guard lock(optimizer_mutex_);
      optimizer_request_.active = true;
      optimizer_request_.start_q = start_q;
      optimizer_request_.goal_q = goal_q;
      optimizer_request_.roadmap_path = roadmap_path;
      optimizer_request_.roadmap_cost = JointPathLength(roadmap_path);
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
    using Clock = std::chrono::steady_clock;
    SetStartAndGoal(setup_, space_, start_q, goal_q);
    setup_.getProblemDefinition()->clearSolutionPaths();

    const auto start_time = Clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(kExperiencePlanningBudgetSeconds));

    if (std::optional<std::vector<JointArray>> graph_path =
            QueryExperienceGraph(start_q, goal_q)) {
      return MakeFastExperiencePlan(*graph_path, start_q, goal_q, start_time, deadline);
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
      length += JointDistanceStatic(path[i - 1], path[i]);
    }
    return length;
  }

  double JointDistance(const JointArray& a, const JointArray& b) const {
    return JointDistanceStatic(a, b);
  }

  static double JointDistanceStatic(const JointArray& a, const JointArray& b) {
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
    ob::ScopedState<> from_state(space);
    ob::ScopedState<> to_state(space);
    FillState(from, from_state);
    FillState(to, to_state);
    return setup.getSpaceInformation()->checkMotion(from_state.get(), to_state.get());
  }

  bool MotionValid(const JointArray& from, const JointArray& to) {
    return MotionValidWith(space_, setup_, from, to);
  }

  RoadmapSnapshot SnapshotRoadmap() const {
    std::shared_lock lock(roadmap_mutex_);
    return RoadmapSnapshot{nodes_, adjacency_, roadmap_version_};
  }

  std::size_t FindOrAddNodeLocked(const JointArray& q) {
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
        candidates.push_back({JointDistance(snapshot.nodes[node_index], snapshot.nodes[i]), i});
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
    std::vector<std::size_t> path_indices;
    path_indices.reserve(path.size());
    std::optional<std::size_t> previous;
    {
      std::unique_lock lock(roadmap_mutex_);
      for (const JointArray& q : path) {
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
      candidates.push_back({JointDistance(query, nodes[i]), i});
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
      adjacency[start_index].push_back({goal_index, direct_weight});
      adjacency[goal_index].push_back({start_index, direct_weight});
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
        const double candidate = distance[current] + edge_cost;
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
    return joints;
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
  Ur5Scene scene_;
  ob::StateSpacePtr space_;
  og::SimpleSetup setup_;
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

void ExecuteLoopPath(Ur5Scene& scene,
                     LivePidViewer& viewer,
                     std::string_view demo_title,
                     const JointArray& start_q,
                     const PlanResult& plan,
                     int accepted_attempt,
                     int max_attempts,
                     std::size_t from_index,
                     std::size_t to_index,
                     bool live_realtime) {
  using Clock = std::chrono::steady_clock;
  const std::vector<std::array<double, 3>> planned_tool_path = ComputeToolPath(scene, plan.path);
  scene.ResetTo(start_q);
  viewer.SetPlannedToolPath(planned_tool_path);
  viewer.ResetClock();

  JointArray integral{};
  int step = 0;
  int contact_steps = 0;
  double max_tracking_error = 0.0;
  auto next_render_time = Clock::now();
  const double trajectory_duration = EstimateExecutionDuration(plan.path);
  const double total_execution_duration = trajectory_duration + kFinalHoldSeconds;
  scene.SetSimulationTimestep(kControlPeriodSeconds);
  const double timestep = kControlPeriodSeconds;
  const int total_control_steps = static_cast<int>(std::ceil(total_execution_duration / timestep));
  const auto control_start_wall_time = Clock::now();

  for (int control_step = 0; control_step <= total_control_steps && viewer.active(); ++control_step) {
    const double elapsed_sim_time = static_cast<double>(control_step) * timestep;
    const bool holding_final_target = elapsed_sim_time >= trajectory_duration;
    const double command_time = std::min(elapsed_sim_time, trajectory_duration);
    const double progress = trajectory_duration > 0.0 ? command_time / trajectory_duration : 1.0;
    const JointArray commanded_target =
        holding_final_target ? plan.path.back() : EvaluateSampledPath(plan.path, progress);

    scene.ApplyPidStep(commanded_target, integral);
    const double tracking_error = MaxAbsJointError(scene.CurrentConfiguration(), commanded_target);
    max_tracking_error = std::max(max_tracking_error, tracking_error);
    contact_steps += scene.ObstacleContactCount() > 0 ? 1 : 0;
    ++step;

    const auto now = Clock::now();
    if (now >= next_render_time || control_step == total_control_steps) {
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

  const double final_error = MaxAbsJointError(scene.CurrentConfiguration(), plan.path.back());
  std::cout << "Loop segment " << from_index + 1 << " -> " << to_index + 1
            << " executed path_kind=\"" << plan.path_kind << "\", accepted_attempt="
            << accepted_attempt << ", planning_wall_ms=" << plan.planning_wall_ms
            << ", final_error=" << final_error << ", max_tracking_error=" << max_tracking_error
            << ", contact_steps=" << contact_steps << '\n';

  if (final_error > 0.20) {
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
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--live-fast|--live-realtime|--check-goals|--check-transitions|--check-cache|--check-optimizer]"
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
      scene_arg.value_or(std::filesystem::path(UR5_MUJOCO_OMPL_DEFAULT_SCENE));

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
    viewer.RenderStatus(std::string(demo_title) + "\nStarting at goal 1", "Esc: stop");
    LoopPathCache path_cache;
    std::optional<RrtConnectExperienceRoadmapLoopPlanner> experience_planner;
    if (planner_kind == PlannerKind::kRrtConnectExperienceRoadmap) {
      experience_planner.emplace(scene_path);
    }

    while (viewer.active()) {
      const std::size_t next_index = (current_index + 1) % goals.size();
      viewer.SetGoalGhost(goals[next_index]);
      SharedPlanningStatus status;
      const auto planning_start = std::chrono::steady_clock::now();
      auto future = std::async(std::launch::async, [&]() {
        const LoopTransitionKey cache_key{current_index, next_index};
        return PlanWithRetries(
            scene_path,
            goals[current_index],
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
                  << " start_q=" << FormatJointArray(goals[current_index]) << '\n';
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
      if (experience_planner) {
        const std::vector<JointArray>& optimizer_path =
            planning.plan.reusable_path_knots.empty() ? planning.plan.path
                                                      : planning.plan.reusable_path_knots;
        experience_planner->BeginExecutionOptimization(
            goals[current_index], goals[next_index], optimizer_path);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      try {
        ExecuteLoopPath(scene,
                        viewer,
                        demo_title,
                        goals[current_index],
                        planning.plan,
                        planning.accepted_attempt,
                        planning.max_attempts,
                        current_index,
                        next_index,
                        live_realtime);
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
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
