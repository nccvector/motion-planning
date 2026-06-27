#include "ur5_clutter_core.hpp"

#include <future>
#include <mutex>
#include <tuple>

namespace {

constexpr int kMaxLoopPlanningAttempts = 5;

struct SharedPlanningStatus {
  std::mutex mutex;
  int attempt = 0;
  std::string phase = "Queued";
  std::string last_error;
};

struct LoopPlanningResult {
  PlanResult plan;
  int accepted_attempt = 0;
  std::string error;
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
  if (tool[0] > kRingPlaneX + 0.08) {
    return "needle-side";
  }
  if (tool[0] < kRingPlaneX - 0.12) {
    return "shelf-side";
  }
  return "ring-plane";
}

void UpdatePlanningStatus(SharedPlanningStatus& status,
                          int attempt,
                          std::string phase,
                          std::string last_error = {}) {
  std::lock_guard lock(status.mutex);
  status.attempt = attempt;
  status.phase = std::move(phase);
  status.last_error = std::move(last_error);
}

std::tuple<int, std::string, std::string> ReadPlanningStatus(SharedPlanningStatus& status) {
  std::lock_guard lock(status.mutex);
  return {status.attempt, status.phase, status.last_error};
}

LoopPlanningResult PlanWithRetries(const std::filesystem::path& scene_path,
                                   const JointArray& start_q,
                                   const JointArray& goal_q,
                                   SharedPlanningStatus& status) {
  LoopPlanningResult result;
  for (int attempt = 1; attempt <= kMaxLoopPlanningAttempts; ++attempt) {
    try {
      UpdatePlanningStatus(status,
                           attempt,
                           "Attempt " + std::to_string(attempt) + "/" +
                               std::to_string(kMaxLoopPlanningAttempts) + ": planning");
      Ur5Scene planning_scene(scene_path);
      result.plan = PlanPath(planning_scene, start_q, goal_q);
      result.accepted_attempt = attempt;
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
  // Alternating Monte Carlo goals from `ur5_goal_monte_carlo`: approach-side
  // states near the red-ring opening, then shelf-side states beyond the ring.
  return {
      {-0.714914, -1.939293, 2.269405, -0.508076, 0.166652, -0.189688},
      {-0.096216, -1.086041, 1.345283, -1.369316, -1.593129, -0.056186},
      {-0.488742, -1.850435, 2.224668, -0.677578, 0.183500, 0.344208},
      {-0.213614, -1.168882, 1.405913, -1.811450, -1.326496, 0.059855},
      {-0.579322, -1.900877, 2.381239, -0.864813, 0.576803, -0.000548},
      {-0.212407, -0.921127, 1.200219, -1.217947, -1.369968, 0.385050},
      {-0.525574, -1.736350, 2.037490, -0.546228, -0.395963, 0.083717},
      {-0.136404, -0.941480, 0.967699, -0.939386, -1.808800, 0.047236},
      {-0.226010, -1.767807, 2.188181, -0.837804, 0.089762, -0.196217},
      {-0.164826, -1.259996, 1.572580, -1.862192, -1.829618, 0.224681},
      {-0.156460, -1.832423, 2.193700, -0.559384, 0.554822, -0.169042},
      {-0.203541, -0.951775, 0.972427, -0.957470, -1.275979, -0.097386},
      {-0.071468, -2.141271, 2.357518, -0.722724, 0.216219, 0.152743},
      {-0.126698, -1.120590, 1.403398, -1.422206, -2.124343, 0.130138},
      {-0.401901, -1.666936, 2.104648, -0.287268, -0.044137, -0.311984},
      {-0.288330, -1.078661, 1.220104, -1.394153, -0.917961, -0.020799},
      {-0.427890, -2.052211, 2.320187, -1.226118, 0.217986, -0.019324},
      {-0.126085, -0.838311, 1.190743, -1.201558, -1.653662, -0.543379},
      {-0.552808, -1.723637, 2.222826, -0.145194, 0.608786, -0.027155},
      {-0.168457, -1.256952, 1.584793, -2.052556, -1.807655, -0.353801},
  };
}

void RenderPlanningProgress(LivePidViewer& viewer,
                            SharedPlanningStatus& status,
                            std::size_t from_index,
                            std::size_t to_index,
                            double elapsed_seconds) {
  const auto [attempt, phase, last_error] = ReadPlanningStatus(status);
  const double budget_seconds =
      static_cast<double>(kMaxLoopPlanningAttempts) * kPlanningTimeBudgetSeconds;
  const int percent = std::min(99, static_cast<int>(100.0 * elapsed_seconds / budget_seconds));

  std::ostringstream left;
  left << "UR5e endless goal loop\n"
       << "Planning to next goal: " << percent << "%\n"
       << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << phase;
  if (attempt > 0) {
    left << "\nAttempt " << attempt << "/" << kMaxLoopPlanningAttempts;
  }
  if (!last_error.empty()) {
    left << "\nLast error: " << last_error;
  }

  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

void RenderExecutionStatus(LivePidViewer& viewer,
                           std::size_t from_index,
                           std::size_t to_index,
                           const PlanResult& plan,
                           int accepted_attempt,
                           double command_time,
                           double trajectory_duration,
                           int step) {
  const double percent =
      trajectory_duration > 0.0 ? 100.0 * std::min(command_time / trajectory_duration, 1.0) : 100.0;
  std::ostringstream left;
  left << "UR5e endless goal loop\n"
       << "Executing path (PID): " << std::fixed << std::setprecision(0) << percent << "%\n"
       << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << "Path: " << plan.path_kind << "\n"
       << "Spline fit: " << (plan.used_c2_spline ? "success" : "failed; using linear fallback")
       << "\n"
       << "Accepted attempt: " << accepted_attempt << "/" << kMaxLoopPlanningAttempts << "\n"
       << "Planning time: " << std::setprecision(1) << plan.planning_wall_ms << " ms\n"
       << "OMPL solve attempts: " << plan.solve_attempts << "\n"
       << "Spline repair iterations: " << plan.spline_repair_iterations << "\n"
       << "Control step: " << step;
  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

void RenderPlanAcceptedStatus(LivePidViewer& viewer,
                              std::size_t from_index,
                              std::size_t to_index,
                              const PlanResult& plan,
                              int accepted_attempt) {
  std::ostringstream left;
  left << "UR5e endless goal loop\n"
       << "Successfully planned geometric path\n"
       << "Simplified geometric path\n";
  if (plan.used_c2_spline) {
    left << "Smoothed final path\n";
  } else {
    left << "Failed to smooth path\n"
         << "Using fallback: " << plan.path_kind << "\n";
  }
  left << "Segment " << from_index + 1 << " -> " << to_index + 1 << "\n"
       << "Accepted attempt: " << accepted_attempt << "/" << kMaxLoopPlanningAttempts << "\n"
       << "Planning time: " << std::fixed << std::setprecision(1) << plan.planning_wall_ms << " ms";

  constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: stop";
  viewer.RenderStatus(left.str(), right);
}

void ExecuteLoopPath(Ur5Scene& scene,
                     LivePidViewer& viewer,
                     const JointArray& start_q,
                     const PlanResult& plan,
                     int accepted_attempt,
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
                            from_index,
                            to_index,
                            plan,
                            accepted_attempt,
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

int main(int argc, char** argv) {
  bool live_realtime = true;
  bool check_goals = false;
  bool check_transitions = false;
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
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--live-fast|--live-realtime|--check-goals|--check-transitions]"
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
      all_goals_valid = valid && all_goals_valid;
      alternating_sides = (previous_side.empty() || previous_side != side) && alternating_sides;
      previous_side = side;
      std::cout << "  goal " << i + 1 << " side=" << side << " valid=" << std::boolalpha
                << valid << " q=" << FormatJointArray(goals[i])
                << " tool=" << FormatToolPosition(scene, goals[i]) << '\n';
    }
    std::cout << "Needle-side loop goals: " << needle_side_goals << '\n';
    std::cout << "Shelf-side loop goals: " << shelf_side_goals << '\n';
    std::cout << "Loop goals alternate sides: " << std::boolalpha << alternating_sides << '\n';
    std::cout << "Loop goals valid: " << std::boolalpha << all_goals_valid << '\n';

    if (check_goals) {
      if (goals.size() != 20 || needle_side_goals != 10 || shelf_side_goals != 10 ||
          !alternating_sides || !all_goals_valid) {
        return 1;
      }
      if (check_transitions) {
        for (std::size_t i = 0; i < goals.size(); ++i) {
          const std::size_t next = (i + 1) % goals.size();
          SharedPlanningStatus status;
          const LoopPlanningResult result = PlanWithRetries(scene_path, goals[i], goals[next], status);
          if (result.accepted_attempt == 0 || result.plan.path.empty()) {
            std::cerr << "transition " << i + 1 << " -> " << next + 1
                      << " failed: " << result.error << '\n';
            return 1;
          }
          std::cout << "transition " << i + 1 << " -> " << next + 1
                    << " planned path=\"" << result.plan.path_kind
                    << "\" accepted_attempt=" << result.accepted_attempt
                    << " planning_wall_ms=" << result.plan.planning_wall_ms << '\n';
        }
      }
      return 0;
    }

    std::size_t current_index = 0;
    scene.ResetTo(goals[current_index]);
    LivePidViewer viewer(scene, {}, true);
    viewer.RenderStatus("UR5e endless goal loop\nStarting at goal 1", "Esc: stop");

    while (viewer.active()) {
      const std::size_t next_index = (current_index + 1) % goals.size();
      viewer.SetGoalGhost(goals[next_index]);
      SharedPlanningStatus status;
      const auto planning_start = std::chrono::steady_clock::now();
      auto future = std::async(std::launch::async, [&]() {
        return PlanWithRetries(scene_path, goals[current_index], goals[next_index], status);
      });

      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
             viewer.active()) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - planning_start)
                .count();
        RenderPlanningProgress(viewer, status, current_index, next_index, elapsed);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
      }
      if (!viewer.active()) {
        break;
      }

      LoopPlanningResult planning = future.get();
      if (planning.accepted_attempt == 0 || planning.plan.path.empty()) {
        std::cerr << "error: exhausted loop planning retries\n";
        std::cerr << "start_index=" << current_index + 1
                  << " start_q=" << FormatJointArray(goals[current_index]) << '\n';
        std::cerr << "goal_index=" << next_index + 1
                  << " goal_q=" << FormatJointArray(goals[next_index]) << '\n';
        std::cerr << "last_error=" << planning.error << '\n';
        viewer.RenderStatus("UR5e endless goal loop\nRetry limit exhausted\nSee terminal for start/goal q",
                            "Esc: close");
        while (viewer.active()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          viewer.RenderStatus(
              "UR5e endless goal loop\nRetry limit exhausted\nSee terminal for start/goal q",
              "Esc: close");
        }
        return 1;
      }

      RenderPlanAcceptedStatus(viewer,
                               current_index,
                               next_index,
                               planning.plan,
                               planning.accepted_attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      ExecuteLoopPath(scene,
                      viewer,
                      goals[current_index],
                      planning.plan,
                      planning.accepted_attempt,
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
