#include "ur5_clutter_core.hpp"

int main(int argc, char** argv) {
  bool live_view = true;
  bool live_realtime = true;
  std::optional<std::filesystem::path> scene_arg;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--no-live") {
      live_view = false;
      live_realtime = false;
    } else if (arg == "--live") {
      live_view = true;
    } else if (arg == "--live-realtime") {
      live_view = true;
      live_realtime = true;
    } else if (arg == "--live-fast") {
      live_view = true;
      live_realtime = false;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--live|--live-fast|--live-realtime|--no-live] [scene.xml]\n";
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
    std::cout << "Loaded scene: " << scene_path << '\n';
    std::cout << "Robot model: " << scene.robot_label() << '\n';
    std::cout << "Control mode: " << scene.ControlModeLabel() << '\n';
    std::cout << "Live PID viewer: " << std::boolalpha << live_view << '\n';
    std::cout << "Live PID realtime pacing: " << std::boolalpha << live_realtime << '\n';
    std::cout << "MuJoCo model: nq=" << scene.model()->nq << " nv=" << scene.model()->nv
              << " nu=" << scene.model()->nu << '\n';

    const JointArray home = SelectHome(scene);
    PrintPose("Selected home", scene, home);
    const JointArray goal = SelectGoal(scene);
    PrintPose("Selected goal", scene, goal);

    const auto plan = PlanPath(scene, home, goal);
    const auto& path = plan.path;
    std::cout << "Planned path states: " << path.size() << '\n';
    ValidatePlannedPath(scene, path);
    ReportRingPassage(scene, path);

    WritePlannedPath("planned_path.csv", scene, path);
    ExecutePath("executed_trace.csv", scene, home, path, live_view, live_realtime);

    const auto final_tool = scene.ToolPosition();
    std::cout << "Final tool position: [" << std::fixed << std::setprecision(3)
              << final_tool[0] << ", " << final_tool[1] << ", " << final_tool[2] << "]\n";
    std::cout << "Wrote planned_path.csv and executed_trace.csv\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
