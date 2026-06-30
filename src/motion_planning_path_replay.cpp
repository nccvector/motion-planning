#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "ur5_visualization.hpp"

namespace {

constexpr int kDof = 6;
constexpr double kFps = 60.0;

using JointArray = std::array<double, kDof>;

struct MjModelDeleter {
  void operator()(mjModel* model) const { mj_deleteModel(model); }
};

struct MjDataDeleter {
  void operator()(mjData* data) const { mj_deleteData(data); }
};

struct JointBinding {
  std::array<int, kDof> qpos_addr{};
  int tool_site_id = -1;
};

struct ReplayState {
  std::vector<JointArray> path;
  std::vector<std::array<double, 3>> tool_path;
  std::size_t frame = 0;
  bool playing = true;
  bool show_collision_model = true;
  double speed = 1.0;
};

mjvCamera g_camera;
mjvOption g_option;
mjvScene g_scene;
ur5vis::InteractiveCamera g_interactive_camera;
ReplayState* g_replay = nullptr;

std::vector<std::string> SplitCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

int FindField(const std::vector<std::string>& header, std::string_view name) {
  const auto it = std::find(header.begin(), header.end(), name);
  if (it == header.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(header.begin(), it));
}

std::vector<JointArray> ReadJointCsv(const std::filesystem::path& path_file) {
  std::ifstream input(path_file);
  if (!input) {
    throw std::runtime_error("Could not open path CSV: " + path_file.string());
  }

  std::string line;
  if (!std::getline(input, line)) {
    throw std::runtime_error("Path CSV is empty: " + path_file.string());
  }

  const std::vector<std::string> header = SplitCsvLine(line);
  std::array<int, kDof> q_columns{};
  for (int i = 0; i < kDof; ++i) {
    q_columns[i] = FindField(header, "q" + std::to_string(i));
    if (q_columns[i] < 0) {
      throw std::runtime_error("Path CSV is missing q" + std::to_string(i));
    }
  }

  std::vector<JointArray> path;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const std::vector<std::string> fields = SplitCsvLine(line);
    JointArray q{};
    for (int i = 0; i < kDof; ++i) {
      const int column = q_columns[i];
      if (column >= static_cast<int>(fields.size())) {
        throw std::runtime_error("Short row in path CSV: " + line);
      }
      q[i] = std::stod(fields[column]);
    }
    path.push_back(q);
  }

  if (path.empty()) {
    throw std::runtime_error("Path CSV has no trajectory rows: " + path_file.string());
  }
  return path;
}

bool TryBindNames(mjModel* model,
                  const std::array<const char*, kDof>& joints,
                  const char* tool_site,
                  JointBinding& binding) {
  const int tool_site_id = mj_name2id(model, mjOBJ_SITE, tool_site);
  if (tool_site_id < 0) {
    return false;
  }

  std::array<int, kDof> qpos_addr{};
  for (int i = 0; i < kDof; ++i) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joints[i]);
    if (joint_id < 0) {
      return false;
    }
    qpos_addr[i] = model->jnt_qposadr[joint_id];
  }

  binding.qpos_addr = qpos_addr;
  binding.tool_site_id = tool_site_id;
  return true;
}

JointBinding BindRobot(mjModel* model) {
  JointBinding binding;
  constexpr std::array<const char*, kDof> kMenagerieJoints = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  constexpr std::array<const char*, kDof> kSimpleJoints = {
      "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"};

  if (TryBindNames(model, kMenagerieJoints, "attachment_site", binding) ||
      TryBindNames(model, kSimpleJoints, "tool0", binding)) {
    return binding;
  }

  throw std::runtime_error("Scene does not expose a recognized UR5/UR5e joint naming scheme");
}

void SetConfiguration(mjModel* model,
                      mjData* data,
                      const JointBinding& binding,
                      const JointArray& q) {
  for (int i = 0; i < kDof; ++i) {
    data->qpos[binding.qpos_addr[i]] = q[i];
  }
  std::fill(data->qvel, data->qvel + model->nv, 0.0);
  mj_forward(model, data);
}

std::array<double, 3> ToolPosition(const mjData* data, const JointBinding& binding) {
  const double* p = data->site_xpos + 3 * binding.tool_site_id;
  return {p[0], p[1], p[2]};
}

std::vector<std::array<double, 3>> ComputeToolPath(mjModel* model,
                                                   mjData* data,
                                                   const JointBinding& binding,
                                                   const std::vector<JointArray>& path) {
  std::vector<std::array<double, 3>> tool_path;
  tool_path.reserve(path.size());
  for (const JointArray& q : path) {
    SetConfiguration(model, data, binding, q);
    tool_path.push_back(ToolPosition(data, binding));
  }
  return tool_path;
}

void Keyboard(GLFWwindow* window, int key, int, int action, int) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT) {
    return;
  }

  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  }
  if (g_replay == nullptr || g_replay->path.empty()) {
    return;
  }

  if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
    g_replay->playing = !g_replay->playing;
  } else if (key == GLFW_KEY_R && action == GLFW_PRESS) {
    g_replay->frame = 0;
    g_replay->playing = true;
  } else if (key == GLFW_KEY_RIGHT) {
    g_replay->playing = false;
    g_replay->frame = std::min(g_replay->frame + 1, g_replay->path.size() - 1);
  } else if (key == GLFW_KEY_LEFT) {
    g_replay->playing = false;
    g_replay->frame = g_replay->frame == 0 ? 0 : g_replay->frame - 1;
  } else if (key == GLFW_KEY_UP) {
    g_replay->speed = std::min(g_replay->speed * 1.25, 4.0);
  } else if (key == GLFW_KEY_DOWN) {
    g_replay->speed = std::max(g_replay->speed / 1.25, 0.05);
  } else if (key == GLFW_KEY_C && action == GLFW_PRESS) {
    g_replay->show_collision_model = !g_replay->show_collision_model;
  }
}

void RenderOverlay(mjrRect viewport, const ReplayState& replay, mjrContext* context) {
  std::ostringstream left;
  left << "UR5e OMPL path replay\n"
       << (replay.playing ? "playing" : "paused") << " | frame " << replay.frame + 1 << "/"
       << replay.path.size() << " | speed " << std::fixed << std::setprecision(2)
       << replay.speed << "x | robot view "
       << (replay.show_collision_model ? "collision" : "mesh");

  constexpr const char* right =
      "Space: play/pause\nLeft/Right: step\nUp/Down: speed\nC: mesh/collision\nR: restart\nEsc: close";

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left.str().c_str(), right, context);
}

}  // namespace

int main(int argc, char** argv) {
  const bool check_only = argc > 1 && std::string_view(argv[1]) == "--check";
  const int arg_offset = check_only ? 1 : 0;
  const std::filesystem::path scene_path =
      argc > 1 + arg_offset ? std::filesystem::path(argv[1 + arg_offset])
               : std::filesystem::path(MOTION_PLANNING_DEFAULT_SCENE);
  const std::filesystem::path path_file =
      argc > 2 + arg_offset ? std::filesystem::path(argv[2 + arg_offset])
                            : std::filesystem::path("planned_path.csv");

  try {
    char error[1024] = {};
    std::unique_ptr<mjModel, MjModelDeleter> model(
        mj_loadXML(scene_path.string().c_str(), nullptr, error, sizeof(error)));
    if (model == nullptr) {
      throw std::runtime_error("MuJoCo failed to load scene: " + std::string(error));
    }

    std::unique_ptr<mjData, MjDataDeleter> data(mj_makeData(model.get()));
    if (data == nullptr) {
      throw std::runtime_error("MuJoCo failed to allocate mjData");
    }

    JointBinding binding = BindRobot(model.get());
    ReplayState replay;
    replay.path = ReadJointCsv(path_file);
    replay.tool_path = ComputeToolPath(model.get(), data.get(), binding, replay.path);
    SetConfiguration(model.get(), data.get(), binding, replay.path.front());

    if (check_only) {
      const auto first_tool = replay.tool_path.front();
      const auto last_tool = replay.tool_path.back();
      std::cout << "Loaded scene: " << scene_path << '\n';
      std::cout << "Loaded path: " << path_file << " (" << replay.path.size() << " frames)\n";
      std::cout << "First tool position: [" << std::fixed << std::setprecision(3)
                << first_tool[0] << ", " << first_tool[1] << ", " << first_tool[2] << "]\n";
      std::cout << "Last tool position: [" << last_tool[0] << ", " << last_tool[1] << ", "
                << last_tool[2] << "]\n";
      return EXIT_SUCCESS;
    }

    if (!glfwInit()) {
      throw std::runtime_error("Could not initialize GLFW");
    }

    GLFWwindow* window = glfwCreateWindow(1280, 900, "UR5e OMPL path replay", nullptr, nullptr);
    if (window == nullptr) {
      glfwTerminate();
      throw std::runtime_error("Could not create GLFW window");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    g_replay = &replay;

    mjv_defaultCamera(&g_camera);
    mjv_defaultOption(&g_option);
    ur5vis::ApplyRobotGeometryMode(&g_option, replay.show_collision_model);
    mjv_defaultScene(&g_scene);
    mjrContext context;
    mjr_defaultContext(&context);

    ur5vis::InitializeCamera(model.get(), &g_camera);

    mjv_makeScene(model.get(), &g_scene, 4000);
    mjr_makeContext(model.get(), &context, mjFONTSCALE_150);
    g_interactive_camera.Attach(window, model.get(), &g_scene, &g_camera);

    glfwSetKeyCallback(window, Keyboard);

    std::cout << "Loaded scene: " << scene_path << '\n';
    std::cout << "Loaded path: " << path_file << " (" << replay.path.size() << " frames)\n";
    std::cout
        << "Controls: Space play/pause, Left/Right step, Up/Down speed, C mesh/collision, R restart, Esc close\n";

    double frame_accumulator = 0.0;
    auto previous = std::chrono::steady_clock::now();
    while (!glfwWindowShouldClose(window)) {
      const auto now = std::chrono::steady_clock::now();
      const double dt = std::chrono::duration<double>(now - previous).count();
      previous = now;

      if (replay.playing) {
        frame_accumulator += dt * kFps * replay.speed;
        while (frame_accumulator >= 1.0) {
          replay.frame = (replay.frame + 1) % replay.path.size();
          frame_accumulator -= 1.0;
        }
      } else {
        frame_accumulator = 0.0;
      }

      SetConfiguration(model.get(), data.get(), binding, replay.path[replay.frame]);

      mjrRect viewport = {0, 0, 0, 0};
      glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
      ur5vis::ApplyRobotGeometryMode(&g_option, replay.show_collision_model);
      mjv_updateScene(model.get(), data.get(), &g_option, nullptr, &g_camera, mjCAT_ALL, &g_scene);
      ur5vis::AddToolPathMarkers(&g_scene, replay.tool_path, &replay.tool_path[replay.frame]);
      mjr_render(viewport, &g_scene, &context);
      RenderOverlay(viewport, replay, &context);

      glfwSwapBuffers(window);
      glfwPollEvents();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    mjv_freeScene(&g_scene);
    mjr_freeContext(&context);
    glfwDestroyWindow(window);
    glfwTerminate();
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
