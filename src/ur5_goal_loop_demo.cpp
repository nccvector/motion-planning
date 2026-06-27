#include "ur5_clutter_core.hpp"

#include <future>
#include <mutex>
#include <tuple>

namespace {

constexpr int kMaxLoopPlanningAttempts = 4;

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
      UpdatePlanningStatus(status, attempt, "Attempt " + std::to_string(attempt) + "/4: planning");
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
  return {
      kHomeCandidates[0],
      kGoalCandidates[12],
      kHomeCandidates[5],
      kGoalCandidates[16],
      kHomeCandidates[8],
      kGoalCandidates[12],
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
  const double timestep = scene.model()->opt.timestep;
  const int total_control_steps = static_cast<int>(std::ceil(total_execution_duration / timestep));

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
      viewer.Pace(scene.data()->time);
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
  std::optional<std::filesystem::path> scene_arg;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--live-fast") {
      live_realtime = false;
    } else if (arg == "--live-realtime") {
      live_realtime = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [--live-fast|--live-realtime] [scene.xml]\n";
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
    for (std::size_t i = 0; i < goals.size(); ++i) {
      std::cout << "  goal " << i + 1 << " q=" << FormatJointArray(goals[i])
                << " tool=" << FormatToolPosition(scene, goals[i]) << '\n';
    }

    std::size_t current_index = 0;
    scene.ResetTo(goals[current_index]);
    LivePidViewer viewer(scene, {}, true);
    viewer.RenderStatus("UR5e endless goal loop\nStarting at goal 1", "Esc: stop");

    while (viewer.active()) {
      const std::size_t next_index = (current_index + 1) % goals.size();
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
