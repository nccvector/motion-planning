#pragma once

#include "ur5_clutter_core.hpp"

#include <future>
#include <limits>
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
                                   SharedPlanningStatus& status,
                                   PlannerKind planner_kind) {
  LoopPlanningResult result;
  for (int attempt = 1; attempt <= kMaxLoopPlanningAttempts; ++attempt) {
    try {
      UpdatePlanningStatus(status,
                           attempt,
                           "Attempt " + std::to_string(attempt) + "/" +
                               std::to_string(kMaxLoopPlanningAttempts) + ": planning");
      Ur5Scene planning_scene(scene_path);
      result.plan = PlanPath(planning_scene, start_q, goal_q, planner_kind);
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
  const auto [attempt, phase, last_error] = ReadPlanningStatus(status);
  const double budget_seconds =
      static_cast<double>(kMaxLoopPlanningAttempts) * kPlanningTimeBudgetSeconds;
  const int percent = std::min(99, static_cast<int>(100.0 * elapsed_seconds / budget_seconds));

  std::ostringstream left;
  left << demo_title << "\n"
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
                           std::string_view demo_title,
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
  left << demo_title << "\n"
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
                              std::string_view demo_title,
                              std::size_t from_index,
                              std::size_t to_index,
                              const PlanResult& plan,
                              int accepted_attempt) {
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
       << "Accepted attempt: " << accepted_attempt << "/" << kMaxLoopPlanningAttempts << "\n"
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
      if (check_transitions) {
        std::array<int, kWindowSpecs.size()> transition_window_hits{};
        for (std::size_t i = 0; i < goals.size(); ++i) {
          const std::size_t next = (i + 1) % goals.size();
          SharedPlanningStatus status;
          const LoopPlanningResult result =
              PlanWithRetries(scene_path, goals[i], goals[next], status, planner_kind);
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

    while (viewer.active()) {
      const std::size_t next_index = (current_index + 1) % goals.size();
      viewer.SetGoalGhost(goals[next_index]);
      SharedPlanningStatus status;
      const auto planning_start = std::chrono::steady_clock::now();
      auto future = std::async(std::launch::async, [&]() {
        return PlanWithRetries(
            scene_path, goals[current_index], goals[next_index], status, planner_kind);
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
        std::cerr << "error: exhausted loop planning retries\n";
        std::cerr << "start_index=" << current_index + 1
                  << " start_q=" << FormatJointArray(goals[current_index]) << '\n';
        std::cerr << "goal_index=" << next_index + 1
                  << " goal_q=" << FormatJointArray(goals[next_index]) << '\n';
        std::cerr << "last_error=" << planning.error << '\n';
        viewer.RenderStatus(std::string(demo_title) +
                                "\nRetry limit exhausted\nSee terminal for start/goal q",
                            "Esc: close");
        while (viewer.active()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          viewer.RenderStatus(
              std::string(demo_title) +
                  "\nRetry limit exhausted\nSee terminal for start/goal q",
              "Esc: close");
        }
        return 1;
      }

      RenderPlanAcceptedStatus(viewer,
                               demo_title,
                               current_index,
                               next_index,
                               planning.plan,
                               planning.accepted_attempt);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));

      ExecuteLoopPath(scene,
                      viewer,
                      demo_title,
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
