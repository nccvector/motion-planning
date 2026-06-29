#pragma once

#include "ur5_clutter_core.hpp"

#include <future>
#include <limits>
#include <mutex>
#include <tuple>
#include <unordered_map>

namespace {

constexpr int kMaxLoopPlanningAttempts = 5;
constexpr double kInitialSpars2PlanningBudgetSeconds = 600.0;
constexpr double kSubsequentSpars2PlanningBudgetSeconds = 1.0;
constexpr double kSpars2GoalSampleBias = 0.02;
constexpr double kSpars2BridgeSampleBias = 0.18;
constexpr double kSpars2BridgeNoiseStdDev = 0.12;

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

class MutableGoalBiasedStateSampler final : public ob::StateSampler {
 public:
  MutableGoalBiasedStateSampler(const ob::StateSpace* space,
                                const JointArray* start_q,
                                const JointArray* goal_q,
                                double goal_bias,
                                double bridge_bias,
                                double bridge_noise_stddev)
      : ob::StateSampler(space),
        default_sampler_(space->allocDefaultStateSampler()),
        start_q_(start_q),
        goal_q_(goal_q),
        goal_bias_(goal_bias),
        bridge_bias_(bridge_bias),
        bridge_noise_stddev_(bridge_noise_stddev) {}

  void sampleUniform(ob::State* state) override {
    const double draw = rng_.uniform01();
    if (goal_q_ != nullptr && draw < goal_bias_) {
      FillState(*goal_q_, state);
      return;
    }
    if (start_q_ != nullptr && goal_q_ != nullptr && draw < goal_bias_ + bridge_bias_) {
      JointArray q{};
      const double alpha = rng_.uniform01();
      for (int i = 0; i < kDof; ++i) {
        q[i] = (*start_q_)[i] + alpha * ((*goal_q_)[i] - (*start_q_)[i]) +
               rng_.gaussian(0.0, bridge_noise_stddev_);
      }
      FillState(q, state);
      space_->enforceBounds(state);
      return;
    }
    default_sampler_->sampleUniform(state);
  }

  void sampleUniformNear(ob::State* state, const ob::State* near, double distance) override {
    default_sampler_->sampleUniformNear(state, near, distance);
  }

  void sampleGaussian(ob::State* state, const ob::State* mean, double std_dev) override {
    default_sampler_->sampleGaussian(state, mean, std_dev);
  }

 private:
  ob::StateSamplerPtr default_sampler_;
  const JointArray* start_q_ = nullptr;
  const JointArray* goal_q_ = nullptr;
  double goal_bias_ = 0.0;
  double bridge_bias_ = 0.0;
  double bridge_noise_stddev_ = 0.0;
  ompl::RNG rng_;
};

class ReusableSpars2LoopPlanner {
 public:
  explicit ReusableSpars2LoopPlanner(const std::filesystem::path& scene_path)
      : scene_(scene_path),
        space_(MakePlanningStateSpace(scene_, JointArray{}, false)),
        setup_(space_),
        planner_(MakePlanner(setup_.getSpaceInformation(), PlannerKind::kSpars2)) {
    space_->setStateSamplerAllocator([this](const ob::StateSpace* sampler_space) {
      return std::make_shared<MutableGoalBiasedStateSampler>(
          sampler_space,
          &current_start_q_,
          &current_goal_q_,
          kSpars2GoalSampleBias,
          kSpars2BridgeSampleBias,
          kSpars2BridgeNoiseStdDev);
    });
    setup_.setStateValidityChecker([this](const ob::State* state) {
      return scene_.IsStateValid(StateToJoints(state));
    });
    setup_.setPlanner(planner_);
  }

  PlanResult Plan(const JointArray& start_q, const JointArray& goal_q) {
    using Clock = std::chrono::steady_clock;
    const double planning_budget_seconds =
        first_query_ ? kInitialSpars2PlanningBudgetSeconds
                     : kSubsequentSpars2PlanningBudgetSeconds;
    current_start_q_ = start_q;
    current_goal_q_ = goal_q;
    SetStartAndGoal(setup_, space_, start_q, goal_q);
    planner_->clearQuery();
    setup_.setup();
    setup_.getProblemDefinition()->clearSolutionPaths();

    const auto start_time = Clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<Clock::duration>(
                         std::chrono::duration<double>(planning_budget_seconds));
    const double solve_budget =
        std::max(kMinOmplSolveAttemptSeconds,
                 planning_budget_seconds - kMinimumSplineAttemptBudgetSeconds);

    const auto solved = setup_.solve(solve_budget);
    first_query_ = false;
    if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
      return FinalizeGeometricPath(scene_,
                                   setup_,
                                   setup_.getSolutionPath(),
                                   PlannerKind::kSpars2,
                                   "SPARS2 reusable roadmap query",
                                   true,
                                   1,
                                   0,
                                   start_time,
                                   deadline,
                                   kSpars2GoalSampleBias);
    }

    std::optional<og::PathGeometric> roadmap_path = QueryPlannerDataPath();
    if (roadmap_path.has_value()) {
      return FinalizeGeometricPath(scene_,
                                   setup_,
                                   *roadmap_path,
                                   PlannerKind::kSpars2,
                                   "SPARS2 planner-data roadmap query",
                                   true,
                                   1,
                                   solved ? 1 : 0,
                                   start_time,
                                   deadline,
                                   kSpars2GoalSampleBias);
    }

    throw std::runtime_error(
        "SPARS2 did not connect the current query through its reusable roadmap");
  }

  unsigned int RoadmapStateCount() const {
    const auto spars2 = std::dynamic_pointer_cast<const og::SPARStwo>(planner_);
    return spars2 == nullptr ? 0U : spars2->milestoneCount();
  }

  double NextPlanningBudgetSeconds() const {
    return first_query_ ? kInitialSpars2PlanningBudgetSeconds
                        : kSubsequentSpars2PlanningBudgetSeconds;
  }

 private:
  std::optional<og::PathGeometric> QueryPlannerDataPath() {
    ob::PlannerData data(setup_.getSpaceInformation());
    planner_->getPlannerData(data);
    const unsigned int vertex_count = data.numVertices();
    if (vertex_count == 0 || data.numStartVertices() == 0 || data.numGoalVertices() == 0) {
      return std::nullopt;
    }

    const unsigned int start_index = data.getStartIndex(0);
    const unsigned int goal_index = data.getGoalIndex(0);
    if (start_index == ob::PlannerData::INVALID_INDEX ||
        goal_index == ob::PlannerData::INVALID_INDEX) {
      return std::nullopt;
    }

    const auto& space_information = setup_.getSpaceInformation();
    std::vector<std::vector<std::pair<unsigned int, double>>> adjacency(vertex_count);
    for (unsigned int v = 0; v < vertex_count; ++v) {
      std::vector<unsigned int> edges;
      data.getEdges(v, edges);
      for (const unsigned int target : edges) {
        const ob::State* from_state = data.getVertex(v).getState();
        const ob::State* to_state = data.getVertex(target).getState();
        if (from_state == nullptr || to_state == nullptr) {
          continue;
        }
        adjacency[v].push_back({target, space_information->distance(from_state, to_state)});
      }
    }

    AddTemporaryRoadmapConnectors(data, start_index, adjacency);
    AddTemporaryRoadmapConnectors(data, goal_index, adjacency);

    std::vector<double> distance(vertex_count, std::numeric_limits<double>::infinity());
    std::vector<unsigned int> previous(vertex_count, ob::PlannerData::INVALID_INDEX);
    std::vector<bool> visited(vertex_count, false);
    distance[start_index] = 0.0;

    for (unsigned int step = 0; step < vertex_count; ++step) {
      unsigned int current = ob::PlannerData::INVALID_INDEX;
      double best_distance = std::numeric_limits<double>::infinity();
      for (unsigned int v = 0; v < vertex_count; ++v) {
        if (!visited[v] && distance[v] < best_distance) {
          current = v;
          best_distance = distance[v];
        }
      }
      if (current == ob::PlannerData::INVALID_INDEX || current == goal_index) {
        break;
      }

      visited[current] = true;
      for (const auto& [target, edge_cost] : adjacency[current]) {
        const double candidate = distance[current] + edge_cost;
        if (candidate < distance[target]) {
          distance[target] = candidate;
          previous[target] = current;
        }
      }
    }

    if (previous[goal_index] == ob::PlannerData::INVALID_INDEX) {
      return std::nullopt;
    }

    std::vector<unsigned int> path_indices;
    for (unsigned int v = goal_index; v != ob::PlannerData::INVALID_INDEX; v = previous[v]) {
      path_indices.push_back(v);
      if (v == start_index) {
        break;
      }
    }
    if (path_indices.empty() || path_indices.back() != start_index) {
      return std::nullopt;
    }
    std::reverse(path_indices.begin(), path_indices.end());

    og::PathGeometric path(space_information);
    for (const unsigned int index : path_indices) {
      const ob::State* state = data.getVertex(index).getState();
      if (state == nullptr) {
        return std::nullopt;
      }
      path.append(state);
    }
    return path;
  }

  void AddTemporaryRoadmapConnectors(
      const ob::PlannerData& data,
      unsigned int query_index,
      std::vector<std::vector<std::pair<unsigned int, double>>>& adjacency) {
    const auto& space_information = setup_.getSpaceInformation();
    const ob::State* query_state = data.getVertex(query_index).getState();
    if (query_state == nullptr) {
      return;
    }

    std::vector<std::pair<double, unsigned int>> candidates;
    candidates.reserve(data.numVertices());
    for (unsigned int v = 0; v < data.numVertices(); ++v) {
      if (v == query_index) {
        continue;
      }
      const ob::State* state = data.getVertex(v).getState();
      if (state == nullptr) {
        continue;
      }
      candidates.push_back({space_information->distance(query_state, state), v});
    }
    std::sort(candidates.begin(), candidates.end());

    for (const auto& [distance, target] : candidates) {
      const ob::State* target_state = data.getVertex(target).getState();
      if (target_state == nullptr || !space_information->checkMotion(query_state, target_state)) {
        continue;
      }
      adjacency[query_index].push_back({target, distance});
      adjacency[target].push_back({query_index, distance});
    }
  }

  JointArray current_start_q_{};
  JointArray current_goal_q_{};
  bool first_query_ = true;
  Ur5Scene scene_;
  ob::StateSpacePtr space_;
  og::SimpleSetup setup_;
  ob::PlannerPtr planner_;
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
                                   ReusableSpars2LoopPlanner* spars2_planner = nullptr) {
  LoopPlanningResult result;
  if (planner_kind == PlannerKind::kSpars2 && spars2_planner != nullptr) {
    const double budget_seconds = spars2_planner->NextPlanningBudgetSeconds();
    try {
      UpdatePlanningStatus(status,
                           1,
                           "SPARS2 single query, " +
                               std::to_string(static_cast<int>(budget_seconds)) + " s budget",
                           {},
                           1,
                           budget_seconds);
      result.plan = spars2_planner->Plan(start_q, goal_q);
      result.accepted_attempt = 1;
      result.max_attempts = 1;
      result.budget_seconds = budget_seconds;
      result.roadmap_states = spars2_planner->RoadmapStateCount();
      result.used_cache = false;
      UpdatePlanningStatus(status, 1, "SPARS2 query succeeded", {}, 1, budget_seconds);
      return result;
    } catch (const std::exception& e) {
      result.max_attempts = 1;
      result.budget_seconds = budget_seconds;
      result.roadmap_states = spars2_planner->RoadmapStateCount();
      result.error = "SPARS2 exhausted " +
                     std::to_string(static_cast<int>(budget_seconds)) +
                     " s without connecting this query; roadmap states=" +
                     std::to_string(result.roadmap_states) + "; " + e.what();
      UpdatePlanningStatus(status, 1, "SPARS2 query failed", result.error, 1, budget_seconds);
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
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--live-fast|--live-realtime|--check-goals|--check-transitions|--check-cache]"
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
        std::optional<ReusableSpars2LoopPlanner> spars2_planner;
        if (planner_kind == PlannerKind::kSpars2) {
          spars2_planner.emplace(scene_path);
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
                            spars2_planner ? &*spars2_planner : nullptr);
        LoopPlanningResult second;
        if (first.accepted_attempt != 0) {
          second = PlanWithRetries(scene_path,
                                   goals[0],
                                   goals[1],
                                   second_status,
                                   planner_kind,
                                   planner_kind == PlannerKind::kRrtConnect ? &path_cache : nullptr,
                                   cache_key,
                                   spars2_planner ? &*spars2_planner : nullptr);
        }
        std::cout << "cache smoke first_used_cache=" << std::boolalpha << first.used_cache
                  << " second_used_cache=" << second.used_cache
                  << " cached_entries=" << path_cache.size();
        if (spars2_planner) {
          std::cout << " spars2_roadmap_states=" << spars2_planner->RoadmapStateCount();
        }
        std::cout << '\n';
        if (planner_kind == PlannerKind::kRrtConnect &&
            (first.accepted_attempt == 0 || second.accepted_attempt == 0 || first.used_cache ||
             !second.used_cache || path_cache.size() != 1)) {
          return 1;
        }
        if (planner_kind == PlannerKind::kSpars2 &&
            (first.accepted_attempt == 0 || second.accepted_attempt == 0 || first.used_cache ||
             second.used_cache || path_cache.size() != 0 || spars2_planner->RoadmapStateCount() == 0)) {
          return 1;
        }
        if (planner_kind != PlannerKind::kRrtConnect && planner_kind != PlannerKind::kSpars2 &&
            second.used_cache) {
          return 1;
        }
      }
      if (check_transitions) {
        std::array<int, kWindowSpecs.size()> transition_window_hits{};
        LoopPathCache path_cache;
        std::optional<ReusableSpars2LoopPlanner> spars2_planner;
        if (planner_kind == PlannerKind::kSpars2) {
          spars2_planner.emplace(scene_path);
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
                              spars2_planner ? &*spars2_planner : nullptr);
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
    std::optional<ReusableSpars2LoopPlanner> spars2_planner;
    if (planner_kind == PlannerKind::kSpars2) {
      spars2_planner.emplace(scene_path);
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
            spars2_planner ? &*spars2_planner : nullptr);
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
        if (planner_kind == PlannerKind::kSpars2) {
          left << "SPARS2 budget exhausted\n"
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
      current_index = next_index;
    }
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
