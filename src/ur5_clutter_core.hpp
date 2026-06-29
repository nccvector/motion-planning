#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "ur5_visualization.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

constexpr int kDof = 6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRingPlaneX = -0.45;
constexpr double kShelfColumnPlaneX = -0.72;
constexpr double kShelfFinishLineX = kShelfColumnPlaneX;
constexpr double kShelfGoalExtraAfterFinish = 0.010;
constexpr double kShelfGoalTargetX = kShelfFinishLineX - kShelfGoalExtraAfterFinish;
constexpr double kShelfWindowPlaneX = kShelfFinishLineX;
constexpr double kRingCenterZ = 0.36;
constexpr double kRingHalfOpeningY = 0.235;
constexpr double kRingHalfOpeningZ = 0.170;
constexpr double kShelfWindowHalfOpeningY = 0.095;
constexpr double kShelfWindowCenterZ = 0.34;
constexpr double kShelfFinishHalfOpeningZ = 0.220;
constexpr double kWindowPlaneTolerance = 0.050;
// Plan with cushion; validate with the hard floor we actually require.
constexpr double kRequiredObstacleClearance = 0.010;
constexpr double kOmplObstacleClearance = 0.015;
constexpr double kPlanningObstacleClearance = 0.015;
constexpr std::size_t kInterpolatedPathStates = 480;
constexpr double kPlanningTimeBudgetSeconds = 3.000;
constexpr double kGoalSampleBias = 0.25;
constexpr double kMaxOmplSolveAttemptSeconds = 0.180;
constexpr double kMinOmplSolveAttemptSeconds = 0.020;
constexpr int kMaxC2RepairIterations = 8;
constexpr std::size_t kMaxC2SplineKnots = 96;
constexpr double kMinimumSplineAttemptBudgetSeconds = 0.080;
constexpr double kNominalJointSpeedRadPerSecond = 0.65;
constexpr double kMinimumExecutionDurationSeconds = 4.0;
constexpr double kFinalHoldSeconds = 1.0;
constexpr int kExecutionTraceStride = 16;
constexpr double kLiveRenderHz = 60.0;
constexpr double kControlHz = 60.0;
constexpr double kControlPeriodSeconds = 1.0 / kControlHz;

using JointArray = std::array<double, kDof>;

struct WindowSpec {
  const char* name;
  double plane_x;
  double center_y;
  double center_z;
  double half_opening_y;
  double half_opening_z;
};

constexpr std::size_t kShelfWindowStartIndex = 1;
constexpr std::size_t kShelfWindowCount = 3;

constexpr std::array<WindowSpec, 4> kWindowSpecs = {{
    {"primary_ring", kRingPlaneX, 0.0, kRingCenterZ, kRingHalfOpeningY, kRingHalfOpeningZ},
    {"shelf_gap_a", kShelfWindowPlaneX, -0.30, kShelfWindowCenterZ,
     kShelfWindowHalfOpeningY, kShelfFinishHalfOpeningZ},
    {"shelf_gap_b", kShelfWindowPlaneX, -0.10, kShelfWindowCenterZ,
     kShelfWindowHalfOpeningY, kShelfFinishHalfOpeningZ},
    {"shelf_gap_c", kShelfWindowPlaneX, 0.12, kShelfWindowCenterZ,
     kShelfWindowHalfOpeningY, kShelfFinishHalfOpeningZ},
}};

constexpr std::array<double, 2> kWindowLayerPlanes = {{
    kRingPlaneX,
    kShelfWindowPlaneX,
}};

struct C2SplineResult {
  std::vector<JointArray> path;
  std::vector<double> sample_parameters;
  double max_knot_position_error = 0.0;
  double max_c1_discontinuity = 0.0;
  double max_c2_discontinuity = 0.0;
};

struct C2AttemptResult {
  C2SplineResult spline;
  int planning_clearance_violations = 0;
  std::vector<std::size_t> violating_sample_indices;
};

struct PlanResult {
  std::vector<JointArray> path;
  std::vector<JointArray> reusable_path_knots;
  bool used_c2_spline = false;
  std::string path_kind = "unknown";
  std::string plan_source = "unknown";
  int solve_attempts = 0;
  int approximate_attempts_rejected = 0;
  int spline_repair_iterations = 0;
  std::size_t raw_state_count = 0;
  std::size_t simplified_state_count = 0;
  double planning_wall_ms = 0.0;
};

enum class PlannerKind {
  kRrtConnect,
  kInformedRrtStar,
  kExperienceRrtConnect,
};

std::string_view PlannerName(PlannerKind planner_kind) {
  switch (planner_kind) {
    case PlannerKind::kRrtConnect:
      return "RRTConnect";
    case PlannerKind::kInformedRrtStar:
      return "InformedRRTstar";
    case PlannerKind::kExperienceRrtConnect:
      return "RRTConnectExperience";
  }
  return "unknown";
}

struct RobotNames {
  const char* label;
  std::array<const char*, kDof> joints;
  std::array<const char*, kDof> actuators;
  const char* tool_site;
};

const RobotNames kMenagerieUr5eNames = {
    "MuJoCo Menagerie UR5e",
    {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
     "wrist_2_joint", "wrist_3_joint"},
    {"shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"},
    "attachment_site"};

const RobotNames kSimpleUr5LikeNames = {
    "simplified UR5-like",
    {"shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"},
    {"shoulder_pan_motor", "shoulder_lift_motor", "elbow_motor", "wrist_1_motor",
     "wrist_2_motor", "wrist_3_motor"},
    "tool0"};

const std::array<JointArray, 12> kHomeCandidates = {{
    {-0.583032, -1.817003, 2.178682, -0.769127, 0.401531, -0.084894},
    {-0.658288, -1.824524, 2.127059, -0.623686, -0.122959, 0.002327},
    {-0.678173, -1.926212, 2.259926, -0.963480, 0.134084, 0.338435},
    {-0.634096, -1.836295, 2.155486, -0.423529, 0.282597, 0.325637},
    {-0.620929, -1.815655, 2.208364, -1.058377, 0.041404, -0.394379},
    {-0.610830, -1.727607, 2.010051, -0.260588, 0.230487, -0.205209},
    {-0.618327, -1.947966, 2.381882, -1.266605, 0.477733, -0.180852},
    {-0.524556, -1.889583, 2.272900, -0.835037, 0.808309, 0.256985},
    {-0.855268, -2.218835, 2.374038, -0.780203, 0.157284, -0.043131},
    {-0.758462, -2.071718, 2.301036, -0.487947, 0.397963, -0.461046},
    {-0.780227, -2.218195, 2.395407, -0.494435, 0.643286, 0.022863},
    {-0.623562, -1.871818, 2.210439, -1.323817, -0.237983, 0.058322},
}};

const std::array<JointArray, 12> kGoalCandidates = {{
    {0.181099, -0.834922, 1.194771, -1.182946, 1.054233, 0.014927},
    {-0.272575, -0.767697, 1.033570, -1.239197, 0.364753, -0.525439},
    {-0.353023, -0.834006, 1.059281, -1.259558, -0.014792, -0.148578},
    {0.182364, -0.833842, 1.252389, -1.429893, 1.210342, 0.526325},
    {-0.278323, -0.793324, 1.001705, -1.911068, 0.323109, 0.283218},
    {-0.346598, -0.881528, 1.170102, -1.202129, 0.318140, -0.647455},
    {0.185796, -0.865629, 1.137453, -0.767655, 1.076431, -0.698241},
    {-0.081339, -1.030858, 1.346229, -0.776390, 1.276983, -0.607144},
    {-0.362126, -0.821684, 1.084173, -1.102080, 0.490087, 0.057136},
    {0.148336, -0.892706, 1.085891, -0.505721, 1.140027, 0.082147},
    {-0.068444, -0.993667, 1.269892, -0.684621, 1.357602, 0.005784},
    {-0.360138, -0.899815, 1.264117, -1.706853, 0.388415, 0.139379},
}};

struct MjModelDeleter {
  void operator()(mjModel* model) const { mj_deleteModel(model); }
};

struct MjDataDeleter {
  void operator()(mjData* data) const { mj_deleteData(data); }
};

enum class ActuatorMode {
  kTorque,
  kPosition,
};

bool IsObstacleName(std::string_view name) {
  return name.starts_with("shelf_") || name.starts_with("ring_");
}

class Ur5Scene {
 public:
  explicit Ur5Scene(const std::filesystem::path& scene_path) {
    char error[1024] = {};
    model_.reset(mj_loadXML(scene_path.string().c_str(), nullptr, error, sizeof(error)));
    if (model_ == nullptr) {
      throw std::runtime_error("MuJoCo failed to load scene: " + std::string(error));
    }

    data_.reset(mj_makeData(model_.get()));
    if (data_ == nullptr) {
      throw std::runtime_error("MuJoCo failed to allocate mjData");
    }

    if (!TryResolveNames(kMenagerieUr5eNames) && !TryResolveNames(kSimpleUr5LikeNames)) {
      throw std::runtime_error("Scene does not expose a recognized UR5/UR5e joint naming scheme");
    }

    actuator_mode_ = DetectActuatorMode();
    CollectCollisionGeoms();
  }

  mjModel* model() { return model_.get(); }
  mjData* data() { return data_.get(); }

  const std::string& robot_label() const { return robot_label_; }

  const char* ControlModeLabel() const {
    return actuator_mode_ == ActuatorMode::kPosition ? "position servo" : "torque PID";
  }

  std::pair<double, double> JointBounds(int joint_index) const {
    const int joint_id = joint_ids_[joint_index];
    if (model_->jnt_limited[joint_id]) {
      return {model_->jnt_range[2 * joint_id], model_->jnt_range[2 * joint_id + 1]};
    }
    return {-2.0 * kPi, 2.0 * kPi};
  }

  void ResetTo(const JointArray& q) {
    mj_resetData(model_.get(), data_.get());
    for (int i = 0; i < kDof; ++i) {
      data_->qpos[qpos_addr_[i]] = q[i];
      data_->qvel[qvel_addr_[i]] = 0.0;
      data_->ctrl[actuator_ids_[i]] =
          actuator_mode_ == ActuatorMode::kPosition ? ClipControl(i, q[i]) : 0.0;
    }
    mj_forward(model_.get(), data_.get());
  }

  void SetConfiguration(const JointArray& q) {
    SetConfigurationOnData(q, data_.get());
  }

  JointArray CurrentConfiguration() const {
    JointArray q{};
    for (int i = 0; i < kDof; ++i) {
      q[i] = data_->qpos[qpos_addr_[i]];
    }
    return q;
  }

  JointArray CurrentVelocity() const {
    JointArray qd{};
    for (int i = 0; i < kDof; ++i) {
      qd[i] = data_->qvel[qvel_addr_[i]];
    }
    return qd;
  }

  std::array<double, 3> ToolPosition() const {
    const double* p = data_->site_xpos + 3 * tool_site_id_;
    return {p[0], p[1], p[2]};
  }

  bool IsStateValid(const JointArray& q) {
    SetConfiguration(q);
    return ObstacleContactCount() == 0 &&
           ObstacleClearanceViolationCount(kOmplObstacleClearance) == 0;
  }

  int ObstacleContactCount() const {
    int count = 0;
    for (int i = 0; i < data_->ncon; ++i) {
      const mjContact& contact = data_->contact[i];
      if (IsObstacleGeom(contact.geom1) || IsObstacleGeom(contact.geom2)) {
        ++count;
      }
    }
    return count;
  }

  std::vector<std::string> ObstacleContactPairs(const JointArray& q) {
    SetConfiguration(q);
    std::vector<std::string> pairs;
    for (int i = 0; i < data_->ncon; ++i) {
      const mjContact& contact = data_->contact[i];
      if (!IsObstacleGeom(contact.geom1) && !IsObstacleGeom(contact.geom2)) {
        continue;
      }
      const char* g1 = mj_id2name(model_.get(), mjOBJ_GEOM, contact.geom1);
      const char* g2 = mj_id2name(model_.get(), mjOBJ_GEOM, contact.geom2);
      pairs.push_back(std::string(g1 != nullptr ? g1 : "<unnamed>") + " <-> " +
                      std::string(g2 != nullptr ? g2 : "<unnamed>"));
    }
    return pairs;
  }

  int ObstacleClearanceViolationCount(
      double required_clearance = kRequiredObstacleClearance) const {
    int count = 0;
    for (const int robot_geom : robot_collision_geoms_) {
      for (const int obstacle_geom : obstacle_geoms_) {
        const mjtNum distance = mj_geomDistance(model_.get(), data_.get(), robot_geom,
                                                obstacle_geom, required_clearance,
                                                nullptr);
        if (distance < required_clearance) {
          ++count;
        }
      }
    }
    return count;
  }

  double MinimumObstacleClearance(double cutoff = kRequiredObstacleClearance) const {
    double min_distance = cutoff;
    for (const int robot_geom : robot_collision_geoms_) {
      for (const int obstacle_geom : obstacle_geoms_) {
        const mjtNum distance = mj_geomDistance(model_.get(), data_.get(), robot_geom,
                                                obstacle_geom, cutoff,
                                                nullptr);
        min_distance = std::min(min_distance, static_cast<double>(distance));
      }
    }
    return min_distance;
  }

  std::vector<std::string> ObstacleClearancePairs(
      const JointArray& q,
      double required_clearance = kRequiredObstacleClearance) {
    SetConfiguration(q);
    std::vector<std::string> pairs;
    for (const int robot_geom : robot_collision_geoms_) {
      for (const int obstacle_geom : obstacle_geoms_) {
        const mjtNum distance = mj_geomDistance(model_.get(), data_.get(), robot_geom,
                                                obstacle_geom, required_clearance,
                                                nullptr);
        if (distance >= required_clearance) {
          continue;
        }
        pairs.push_back(GeomLabel(robot_geom) + " <-> " + GeomLabel(obstacle_geom) +
                        ", clearance=" + std::to_string(static_cast<double>(distance)) + " m");
      }
    }
    return pairs;
  }

  void SetSimulationTimestep(double timestep) { model_->opt.timestep = timestep; }

  void SetConfigurationOnData(const JointArray& q, mjData* data) const {
    mj_resetData(model_.get(), data);
    for (int i = 0; i < kDof; ++i) {
      data->qpos[qpos_addr_[i]] = q[i];
      data->qvel[qvel_addr_[i]] = 0.0;
    }
    mj_forward(model_.get(), data);
  }

  const std::vector<int>& robot_collision_geoms() const { return robot_collision_geoms_; }

  void ApplyPidStep(const JointArray& target, JointArray& integral) {
    const double dt = model_->opt.timestep;

    if (actuator_mode_ == ActuatorMode::kPosition) {
      for (int i = 0; i < kDof; ++i) {
        data_->ctrl[actuator_ids_[i]] = ClipControl(i, target[i]);
      }
    } else {
      constexpr JointArray kp = {65.0, 80.0, 70.0, 28.0, 22.0, 18.0};
      constexpr JointArray kd = {7.5, 8.0, 7.0, 3.0, 2.5, 2.0};
      constexpr JointArray ki = {0.05, 0.06, 0.05, 0.02, 0.02, 0.01};
      for (int i = 0; i < kDof; ++i) {
        const double q = data_->qpos[qpos_addr_[i]];
        const double qd = data_->qvel[qvel_addr_[i]];
        const double error = target[i] - q;
        integral[i] = std::clamp(integral[i] + error * dt, -0.5, 0.5);
        const double ctrl = kp[i] * error - kd[i] * qd + ki[i] * integral[i];
        data_->ctrl[actuator_ids_[i]] = ClipControl(i, ctrl);
      }
    }

    mj_step(model_.get(), data_.get());
  }

 private:
  bool TryResolveNames(const RobotNames& names) {
    const int tool_site_id = mj_name2id(model_.get(), mjOBJ_SITE, names.tool_site);
    if (tool_site_id < 0) {
      return false;
    }

    std::array<int, kDof> joint_ids{};
    std::array<int, kDof> actuator_ids{};
    std::array<int, kDof> qpos_addr{};
    std::array<int, kDof> qvel_addr{};

    for (int i = 0; i < kDof; ++i) {
      joint_ids[i] = mj_name2id(model_.get(), mjOBJ_JOINT, names.joints[i]);
      actuator_ids[i] = mj_name2id(model_.get(), mjOBJ_ACTUATOR, names.actuators[i]);
      if (joint_ids[i] < 0 || actuator_ids[i] < 0) {
        return false;
      }
      qpos_addr[i] = model_->jnt_qposadr[joint_ids[i]];
      qvel_addr[i] = model_->jnt_dofadr[joint_ids[i]];
    }

    tool_site_id_ = tool_site_id;
    joint_ids_ = joint_ids;
    actuator_ids_ = actuator_ids;
    qpos_addr_ = qpos_addr;
    qvel_addr_ = qvel_addr;
    robot_label_ = names.label;
    return true;
  }

  ActuatorMode DetectActuatorMode() const {
    double max_abs_ctrl = 0.0;
    for (const int actuator_id : actuator_ids_) {
      max_abs_ctrl = std::max(max_abs_ctrl, std::abs(model_->actuator_ctrlrange[2 * actuator_id]));
      max_abs_ctrl = std::max(max_abs_ctrl, std::abs(model_->actuator_ctrlrange[2 * actuator_id + 1]));
    }
    return max_abs_ctrl <= 10.0 ? ActuatorMode::kPosition : ActuatorMode::kTorque;
  }

  double ClipControl(int joint_index, double value) const {
    const int actuator_id = actuator_ids_[joint_index];
    const double lo = model_->actuator_ctrlrange[2 * actuator_id];
    const double hi = model_->actuator_ctrlrange[2 * actuator_id + 1];
    return std::clamp(value, lo, hi);
  }

	  bool IsObstacleGeom(int geom_id) const {
	    const char* name = mj_id2name(model_.get(), mjOBJ_GEOM, geom_id);
	    return name != nullptr && IsObstacleName(name);
	  }

  bool IsRobotCollisionGeom(int geom_id) const {
    return model_->geom_group[geom_id] == 3 && !IsObstacleGeom(geom_id);
  }

  void CollectCollisionGeoms() {
    for (int geom_id = 0; geom_id < model_->ngeom; ++geom_id) {
      if (IsRobotCollisionGeom(geom_id)) {
        robot_collision_geoms_.push_back(geom_id);
      }
      if (IsObstacleGeom(geom_id)) {
        obstacle_geoms_.push_back(geom_id);
      }
    }
    if (robot_collision_geoms_.empty()) {
      throw std::runtime_error("No robot collision geoms found in MuJoCo group 3");
    }
    if (obstacle_geoms_.empty()) {
      throw std::runtime_error("No shelf/ring obstacle geoms found");
    }
  }

  std::string GeomLabel(int geom_id) const {
    const char* geom_name = mj_id2name(model_.get(), mjOBJ_GEOM, geom_id);
    const int body_id = model_->geom_bodyid[geom_id];
    const char* body_name = mj_id2name(model_.get(), mjOBJ_BODY, body_id);
    return std::string(geom_name != nullptr ? geom_name : "<unnamed>") + " [body=" +
           std::string(body_name != nullptr ? body_name : "<unnamed>") + "]";
  }

  std::unique_ptr<mjModel, MjModelDeleter> model_;
  std::unique_ptr<mjData, MjDataDeleter> data_;
  std::array<int, kDof> joint_ids_{};
  std::array<int, kDof> actuator_ids_{};
  std::array<int, kDof> qpos_addr_{};
  std::array<int, kDof> qvel_addr_{};
  std::string robot_label_;
  int tool_site_id_ = -1;
  ActuatorMode actuator_mode_ = ActuatorMode::kTorque;
  std::vector<int> robot_collision_geoms_;
  std::vector<int> obstacle_geoms_;
};

std::vector<std::array<double, 3>> ComputeToolPath(Ur5Scene& scene,
                                                   const std::vector<JointArray>& path) {
  std::vector<std::array<double, 3>> tool_path;
  tool_path.reserve(path.size());
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    tool_path.push_back(scene.ToolPosition());
  }
  return tool_path;
}

class LivePidViewer {
 public:
  LivePidViewer(Ur5Scene& scene,
                std::vector<std::array<double, 3>> planned_tool_path,
                bool enabled)
      : scene_(scene), planned_tool_path_(std::move(planned_tool_path)) {
    if (!enabled) {
      return;
    }
    if (!glfwInit()) {
      std::cerr << "warning: live PID viewer disabled because GLFW could not initialize\n";
      return;
    }

    window_ = glfwCreateWindow(1280, 900, "UR5e live PID execution", nullptr, nullptr);
    if (window_ == nullptr) {
      std::cerr << "warning: live PID viewer disabled because GLFW could not create a window\n";
      glfwTerminate();
      return;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);
    ur5vis::ApplyRobotGeometryMode(&option_, show_collision_model_);
    mjv_defaultScene(&mjv_scene_);
    mjr_defaultContext(&context_);

    ur5vis::InitializeCamera(scene_.model(), &camera_);

    mjv_makeScene(scene_.model(), &mjv_scene_, 4000);
    mjr_makeContext(scene_.model(), &context_, mjFONTSCALE_150);
    interactive_camera_.Attach(window_, scene_.model(), &mjv_scene_, &camera_);
    goal_ghost_data_ = mj_makeData(scene_.model());
    if (goal_ghost_data_ == nullptr) {
      std::cerr << "warning: goal ghost disabled because MuJoCo data allocation failed\n";
    }
    start_wall_time_ = Clock::now();
    active_ = true;
  }

  ~LivePidViewer() {
    if (goal_ghost_data_ != nullptr) {
      mj_deleteData(goal_ghost_data_);
      goal_ghost_data_ = nullptr;
    }
    if (!active_) {
      return;
    }
    mjv_freeScene(&mjv_scene_);
    mjr_freeContext(&context_);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

  bool active() const { return active_ && window_ != nullptr && !glfwWindowShouldClose(window_); }

  void ResetClock() { start_wall_time_ = Clock::now(); }

  void Pace(double simulation_time) const {
    if (!active()) {
      return;
    }
    const auto target_time =
        start_wall_time_ +
        std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(simulation_time));
    const auto now = Clock::now();
    if (target_time > now) {
      std::this_thread::sleep_until(target_time);
    }
  }

  void Render(std::size_t target_index, int step) {
    if (!active()) {
      return;
    }
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      glfwSetWindowShouldClose(window_, GLFW_TRUE);
      return;
    }
    const bool c_down = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
    if (c_down && !c_was_down_) {
      show_collision_model_ = !show_collision_model_;
    }
    c_was_down_ = c_down;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    ur5vis::ApplyRobotGeometryMode(&option_, show_collision_model_);
    mjv_updateScene(scene_.model(), scene_.data(), &option_, nullptr, &camera_, mjCAT_ALL,
                    &mjv_scene_);
    AddGoalGhost();
    const auto current_tool = scene_.ToolPosition();
    ur5vis::AddToolPathMarkers(&mjv_scene_, planned_tool_path_, &current_tool);
    mjr_render(viewport, &mjv_scene_, &context_);
    RenderOverlay(viewport, target_index, step);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  void RenderStatus(const std::string& left, const std::string& right) {
    if (!active()) {
      return;
    }
    if (glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      glfwSetWindowShouldClose(window_, GLFW_TRUE);
      return;
    }
    const bool c_down = glfwGetKey(window_, GLFW_KEY_C) == GLFW_PRESS;
    if (c_down && !c_was_down_) {
      show_collision_model_ = !show_collision_model_;
    }
    c_was_down_ = c_down;

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    ur5vis::ApplyRobotGeometryMode(&option_, show_collision_model_);
    mjv_updateScene(scene_.model(), scene_.data(), &option_, nullptr, &camera_, mjCAT_ALL,
                    &mjv_scene_);
    AddGoalGhost();
    const auto current_tool = scene_.ToolPosition();
    ur5vis::AddToolPathMarkers(&mjv_scene_, planned_tool_path_, &current_tool);
    mjr_render(viewport, &mjv_scene_, &context_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left.c_str(), right.c_str(), &context_);
    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  void SetPlannedToolPath(std::vector<std::array<double, 3>> planned_tool_path) {
    planned_tool_path_ = std::move(planned_tool_path);
  }

  void SetGoalGhost(const std::optional<JointArray>& goal_q) { goal_ghost_q_ = goal_q; }

 private:
  using Clock = std::chrono::steady_clock;

  void AddGoalGhost() {
    if (!goal_ghost_q_.has_value() || goal_ghost_data_ == nullptr) {
      return;
    }
    scene_.SetConfigurationOnData(*goal_ghost_q_, goal_ghost_data_);
    ur5vis::AddGoalGhostGeoms(&mjv_scene_,
                              scene_.model(),
                              goal_ghost_data_,
                              scene_.robot_collision_geoms());
  }

  void RenderOverlay(mjrRect viewport, std::size_t target_index, int step) {
    std::ostringstream left;
    left << "UR5e live PID execution\n"
         << "target " << target_index + 1 << "/" << planned_tool_path_.size() << " | step "
         << step << " | sim " << std::fixed << std::setprecision(2) << scene_.data()->time
         << " s | robot view " << (show_collision_model_ ? "collision" : "mesh");

    constexpr const char* right = "C: mesh/collision\nMouse: rotate/pan/zoom\nEsc: close viewer";
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, left.str().c_str(), right, &context_);
  }

  Ur5Scene& scene_;
  std::vector<std::array<double, 3>> planned_tool_path_;
  GLFWwindow* window_ = nullptr;
  mjvCamera camera_{};
  mjvOption option_{};
  mjvScene mjv_scene_{};
  mjrContext context_{};
  ur5vis::InteractiveCamera interactive_camera_;
  mjData* goal_ghost_data_ = nullptr;
  std::optional<JointArray> goal_ghost_q_;
  Clock::time_point start_wall_time_{};
  bool active_ = false;
  bool show_collision_model_ = true;
  bool c_was_down_ = false;
};

JointArray StateToJoints(const ob::State* state) {
  const auto* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  JointArray q{};
  for (int i = 0; i < kDof; ++i) {
    q[i] = values[i];
  }
  return q;
}

void FillState(const JointArray& q, ob::ScopedState<>& state) {
  for (int i = 0; i < kDof; ++i) {
    state[i] = q[i];
  }
}

void FillState(const JointArray& q, ob::State* state) {
  auto* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
  for (int i = 0; i < kDof; ++i) {
    values[i] = q[i];
  }
}

class GoalBiasedStateSampler final : public ob::StateSampler {
 public:
  GoalBiasedStateSampler(const ob::StateSpace* space, JointArray goal_q, double goal_bias)
      : ob::StateSampler(space),
        default_sampler_(space->allocDefaultStateSampler()),
        goal_q_(goal_q),
        goal_bias_(goal_bias) {}

  void sampleUniform(ob::State* state) override {
    if (rng_.uniform01() < goal_bias_) {
      FillState(goal_q_, state);
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
  JointArray goal_q_;
  double goal_bias_ = 0.0;
  ompl::RNG rng_;
};

double MaxAbsJointError(const JointArray& a, const JointArray& b) {
  double error = 0.0;
  for (int i = 0; i < kDof; ++i) {
    error = std::max(error, std::abs(a[i] - b[i]));
  }
  return error;
}

double JointDistance(const JointArray& a, const JointArray& b) {
  double sum = 0.0;
  for (int i = 0; i < kDof; ++i) {
    const double delta = b[i] - a[i];
    sum += delta * delta;
  }
  return std::sqrt(sum);
}

JointArray BlendJoints(const JointArray& a, const JointArray& b, double t) {
  JointArray q{};
  for (int i = 0; i < kDof; ++i) {
    q[i] = a[i] + t * (b[i] - a[i]);
  }
  return q;
}

double MaxJointDelta(const JointArray& a, const JointArray& b) {
  double result = 0.0;
  for (int i = 0; i < kDof; ++i) {
    result = std::max(result, std::abs(b[i] - a[i]));
  }
  return result;
}

double EstimateExecutionDuration(const std::vector<JointArray>& path) {
  double duration = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    duration += MaxJointDelta(path[i - 1], path[i]) / kNominalJointSpeedRadPerSecond;
  }
  return std::max(duration, kMinimumExecutionDurationSeconds);
}

void OrientPathToRequestedEndpoints(std::vector<JointArray>& path,
                                    std::vector<JointArray>& reusable_path_knots,
                                    const JointArray& requested_start,
                                    const JointArray& requested_goal) {
  constexpr double kEndpointTolerance = 0.08;
  if (path.empty()) {
    throw std::runtime_error("Planner returned an empty executable path");
  }

  const double start_error = MaxJointDelta(path.front(), requested_start);
  const double goal_error = MaxJointDelta(path.back(), requested_goal);
  if (start_error <= kEndpointTolerance && goal_error <= kEndpointTolerance) {
    return;
  }

  const double reversed_start_error = MaxJointDelta(path.front(), requested_goal);
  const double reversed_goal_error = MaxJointDelta(path.back(), requested_start);
  if (reversed_start_error <= kEndpointTolerance && reversed_goal_error <= kEndpointTolerance) {
    std::reverse(path.begin(), path.end());
    std::reverse(reusable_path_knots.begin(), reusable_path_knots.end());
    return;
  }

  throw std::runtime_error(
      "Planner executable path endpoints do not match requested query; start_error=" +
      std::to_string(start_error) + ", goal_error=" + std::to_string(goal_error) +
      ", reversed_start_error=" + std::to_string(reversed_start_error) +
      ", reversed_goal_error=" + std::to_string(reversed_goal_error));
}

JointArray EvaluateSampledPath(const std::vector<JointArray>& path, double progress) {
  if (path.empty()) {
    throw std::runtime_error("Cannot evaluate empty path");
  }
  if (path.size() == 1) {
    return path.front();
  }
  const double scaled =
      std::clamp(progress, 0.0, 1.0) * static_cast<double>(path.size() - 1);
  const std::size_t segment =
      std::min<std::size_t>(static_cast<std::size_t>(scaled), path.size() - 2);
  const double alpha = scaled - static_cast<double>(segment);
  return BlendJoints(path[segment], path[segment + 1], alpha);
}

std::vector<JointArray> PathStatesToJoints(const og::PathGeometric& path) {
  std::vector<JointArray> result;
  result.reserve(path.getStateCount());
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    result.push_back(StateToJoints(path.getState(i)));
  }
  return result;
}

og::PathGeometric JointsToPathGeometric(const ob::SpaceInformationPtr& space_information,
                                        const std::vector<JointArray>& knots) {
  if (knots.size() < 2) {
    throw std::runtime_error("Cannot build an OMPL path from fewer than two joint knots");
  }
  og::PathGeometric path(space_information);
  for (const JointArray& q : knots) {
    ob::ScopedState<> state(space_information->getStateSpace());
    FillState(q, state);
    path.append(state.get());
  }
  return path;
}

std::vector<double> ChordLengthParameters(const std::vector<JointArray>& knots) {
  std::vector<double> parameters(knots.size(), 0.0);
  for (std::size_t i = 1; i < knots.size(); ++i) {
    parameters[i] = parameters[i - 1] + JointDistance(knots[i - 1], knots[i]);
  }
  if (parameters.back() <= 1.0e-12) {
    throw std::runtime_error("Cannot build a C2 spline through a zero-length path");
  }
  return parameters;
}

std::vector<JointArray> NaturalSplineSecondDerivatives(const std::vector<JointArray>& knots,
                                                       const std::vector<double>& parameters) {
  const std::size_t n = knots.size();
  std::vector<JointArray> second_derivatives(n, JointArray{});
  if (n <= 2) {
    return second_derivatives;
  }

  const std::size_t interior_count = n - 2;
  for (int joint = 0; joint < kDof; ++joint) {
    std::vector<double> lower(interior_count, 0.0);
    std::vector<double> diagonal(interior_count, 0.0);
    std::vector<double> upper(interior_count, 0.0);
    std::vector<double> rhs(interior_count, 0.0);

    for (std::size_t row = 0; row < interior_count; ++row) {
      const std::size_t i = row + 1;
      const double h_prev = parameters[i] - parameters[i - 1];
      const double h_next = parameters[i + 1] - parameters[i];
      lower[row] = row == 0 ? 0.0 : h_prev;
      diagonal[row] = 2.0 * (h_prev + h_next);
      upper[row] = row + 1 == interior_count ? 0.0 : h_next;
      rhs[row] = 6.0 * ((knots[i + 1][joint] - knots[i][joint]) / h_next -
                        (knots[i][joint] - knots[i - 1][joint]) / h_prev);
    }

    for (std::size_t row = 1; row < interior_count; ++row) {
      const double factor = lower[row] / diagonal[row - 1];
      diagonal[row] -= factor * upper[row - 1];
      rhs[row] -= factor * rhs[row - 1];
    }

    std::vector<double> solution(interior_count, 0.0);
    solution.back() = rhs.back() / diagonal.back();
    for (std::size_t reverse_row = interior_count - 1; reverse_row > 0; --reverse_row) {
      const std::size_t row = reverse_row - 1;
      solution[row] = (rhs[row] - upper[row] * solution[row + 1]) / diagonal[row];
    }

    for (std::size_t row = 0; row < interior_count; ++row) {
      second_derivatives[row + 1][joint] = solution[row];
    }
  }

  return second_derivatives;
}

std::size_t SplineSegmentFor(const std::vector<double>& parameters, double s) {
  if (s <= parameters.front()) {
    return 0;
  }
  if (s >= parameters.back()) {
    return parameters.size() - 2;
  }
  const auto upper = std::upper_bound(parameters.begin(), parameters.end(), s);
  return static_cast<std::size_t>(std::distance(parameters.begin(), upper) - 1);
}

JointArray EvaluateNaturalSpline(const std::vector<JointArray>& knots,
                                 const std::vector<double>& parameters,
                                 const std::vector<JointArray>& second_derivatives,
                                 double s) {
  const std::size_t segment = SplineSegmentFor(parameters, s);
  const double h = parameters[segment + 1] - parameters[segment];
  const double u = s - parameters[segment];
  JointArray q{};
  for (int joint = 0; joint < kDof; ++joint) {
    const double y0 = knots[segment][joint];
    const double y1 = knots[segment + 1][joint];
    const double m0 = second_derivatives[segment][joint];
    const double m1 = second_derivatives[segment + 1][joint];
    const double a = y0;
    const double b = (y1 - y0) / h - h * (2.0 * m0 + m1) / 6.0;
    const double c = m0 / 2.0;
    const double d = (m1 - m0) / (6.0 * h);
    q[joint] = a + b * u + c * u * u + d * u * u * u;
  }
  return q;
}

JointArray EvaluateLinearPath(const std::vector<JointArray>& knots,
                              const std::vector<double>& parameters,
                              double s) {
  const std::size_t segment = SplineSegmentFor(parameters, s);
  const double h = parameters[segment + 1] - parameters[segment];
  const double alpha = (s - parameters[segment]) / h;
  JointArray q{};
  for (int joint = 0; joint < kDof; ++joint) {
    q[joint] = knots[segment][joint] +
               alpha * (knots[segment + 1][joint] - knots[segment][joint]);
  }
  return q;
}

std::vector<JointArray> SampleLinearPath(const std::vector<JointArray>& knots,
                                         std::size_t sample_count) {
  if (knots.size() < 2) {
    throw std::runtime_error("Need at least two knots for linear path sampling");
  }
  const std::vector<double> parameters = ChordLengthParameters(knots);
  std::vector<JointArray> result;
  result.reserve(sample_count);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const double alpha = static_cast<double>(sample) / static_cast<double>(sample_count - 1);
    const double s = parameters.back() * alpha;
    result.push_back(EvaluateLinearPath(knots, parameters, s));
  }
  return result;
}

JointArray EvaluateNaturalSplineDerivative(const std::vector<JointArray>& knots,
                                           const std::vector<double>& parameters,
                                           const std::vector<JointArray>& second_derivatives,
                                           std::size_t segment,
                                           double u) {
  const double h = parameters[segment + 1] - parameters[segment];
  JointArray qd{};
  for (int joint = 0; joint < kDof; ++joint) {
    const double y0 = knots[segment][joint];
    const double y1 = knots[segment + 1][joint];
    const double m0 = second_derivatives[segment][joint];
    const double m1 = second_derivatives[segment + 1][joint];
    const double b = (y1 - y0) / h - h * (2.0 * m0 + m1) / 6.0;
    const double c = m0 / 2.0;
    const double d = (m1 - m0) / (6.0 * h);
    qd[joint] = b + 2.0 * c * u + 3.0 * d * u * u;
  }
  return qd;
}

JointArray EvaluateNaturalSplineSecondDerivative(const std::vector<double>& parameters,
                                                 const std::vector<JointArray>& second_derivatives,
                                                 std::size_t segment,
                                                 double u) {
  const double h = parameters[segment + 1] - parameters[segment];
  JointArray qdd{};
  for (int joint = 0; joint < kDof; ++joint) {
    const double m0 = second_derivatives[segment][joint];
    const double m1 = second_derivatives[segment + 1][joint];
    const double c = m0 / 2.0;
    const double d = (m1 - m0) / (6.0 * h);
    qdd[joint] = 2.0 * c + 6.0 * d * u;
  }
  return qdd;
}

C2SplineResult SampleNaturalCubicSpline(const std::vector<JointArray>& knots,
                                        std::size_t sample_count) {
  if (knots.size() < 2) {
    throw std::runtime_error("Need at least two knots for C2 spline sampling");
  }
  if (sample_count < 2) {
    throw std::runtime_error("Need at least two samples for C2 spline sampling");
  }

  const std::vector<double> parameters = ChordLengthParameters(knots);
  const std::vector<JointArray> second_derivatives =
      NaturalSplineSecondDerivatives(knots, parameters);

  C2SplineResult result;
  result.path.reserve(sample_count);
  result.sample_parameters.reserve(sample_count);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    const double alpha = static_cast<double>(sample) / static_cast<double>(sample_count - 1);
    const double s = parameters.back() * alpha;
    result.sample_parameters.push_back(s);
    result.path.push_back(EvaluateNaturalSpline(knots, parameters, second_derivatives, s));
  }

  for (std::size_t i = 0; i < knots.size(); ++i) {
    const JointArray q = EvaluateNaturalSpline(knots, parameters, second_derivatives, parameters[i]);
    result.max_knot_position_error =
        std::max(result.max_knot_position_error, MaxAbsJointError(q, knots[i]));
  }

  for (std::size_t knot = 1; knot + 1 < knots.size(); ++knot) {
    const double h_left = parameters[knot] - parameters[knot - 1];
    const JointArray left_velocity =
        EvaluateNaturalSplineDerivative(knots, parameters, second_derivatives, knot - 1, h_left);
    const JointArray right_velocity =
        EvaluateNaturalSplineDerivative(knots, parameters, second_derivatives, knot, 0.0);
    const JointArray left_acceleration =
        EvaluateNaturalSplineSecondDerivative(parameters, second_derivatives, knot - 1, h_left);
    const JointArray right_acceleration =
        EvaluateNaturalSplineSecondDerivative(parameters, second_derivatives, knot, 0.0);
    result.max_c1_discontinuity =
        std::max(result.max_c1_discontinuity, MaxAbsJointError(left_velocity, right_velocity));
    result.max_c2_discontinuity =
        std::max(result.max_c2_discontinuity,
                 MaxAbsJointError(left_acceleration, right_acceleration));
  }

  return result;
}

int JointPathClearanceViolationCount(Ur5Scene& scene,
                                     const std::vector<JointArray>& path,
                                     double required_clearance) {
  int violations = 0;
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    if (scene.ObstacleClearanceViolationCount(required_clearance) > 0) {
      ++violations;
    }
  }
  return violations;
}

bool ToolAlignedWithWindow(const std::array<double, 3>& tool,
                           const WindowSpec& window,
                           double scale = 1.0) {
  return std::abs(tool[1] - window.center_y) < window.half_opening_y * scale &&
         std::abs(tool[2] - window.center_z) < window.half_opening_z * scale;
}

bool ToolInsideWindow(const std::array<double, 3>& tool, const WindowSpec& window) {
  return std::abs(tool[0] - window.plane_x) < kWindowPlaneTolerance &&
         ToolAlignedWithWindow(tool, window);
}

bool ToolPathCrossesPlane(const std::vector<std::array<double, 3>>& tools, double plane_x) {
  if (tools.empty()) {
    return false;
  }
  double min_x = tools.front()[0];
  double max_x = tools.front()[0];
  for (const auto& tool : tools) {
    min_x = std::min(min_x, tool[0]);
    max_x = std::max(max_x, tool[0]);
  }
  return min_x <= plane_x + kWindowPlaneTolerance && max_x >= plane_x - kWindowPlaneTolerance;
}

std::vector<std::array<double, 3>> ComputeToolPathSamples(Ur5Scene& scene,
                                                          const std::vector<JointArray>& path) {
  std::vector<std::array<double, 3>> tools;
  tools.reserve(path.size());
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    tools.push_back(scene.ToolPosition());
  }
  return tools;
}

std::array<int, kWindowSpecs.size()> WindowHitCounts(
    const std::vector<std::array<double, 3>>& tools) {
  std::array<int, kWindowSpecs.size()> counts{};
  for (const auto& tool : tools) {
    for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
      if (ToolInsideWindow(tool, kWindowSpecs[i])) {
        ++counts[i];
      }
    }
  }
  return counts;
}

std::array<int, kWindowSpecs.size()> WindowHitCounts(Ur5Scene& scene,
                                                     const std::vector<JointArray>& path) {
  return WindowHitCounts(ComputeToolPathSamples(scene, path));
}

bool PathPassesRingOpening(Ur5Scene& scene, const std::vector<JointArray>& path) {
  const std::vector<std::array<double, 3>> tools = ComputeToolPathSamples(scene, path);
  const std::array<int, kWindowSpecs.size()> counts = WindowHitCounts(tools);

  for (const double layer_plane : kWindowLayerPlanes) {
    if (!ToolPathCrossesPlane(tools, layer_plane)) {
      continue;
    }
    bool layer_has_hit = false;
    for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
      if (std::abs(kWindowSpecs[i].plane_x - layer_plane) < 1e-9 && counts[i] > 0) {
        layer_has_hit = true;
      }
    }
    if (!layer_has_hit) {
      return false;
    }
  }
  return true;
}

C2AttemptResult TryC2Spline(Ur5Scene& scene,
                            const std::vector<JointArray>& knots,
                            double required_clearance) {
  C2AttemptResult result;
  result.spline = SampleNaturalCubicSpline(knots, kInterpolatedPathStates);
  for (std::size_t i = 0; i < result.spline.path.size(); ++i) {
    scene.SetConfiguration(result.spline.path[i]);
    if (scene.ObstacleClearanceViolationCount(required_clearance) > 0) {
      ++result.planning_clearance_violations;
      result.violating_sample_indices.push_back(i);
    }
  }
  return result;
}

std::vector<JointArray> AddViolatingSamplesAsKnots(const std::vector<JointArray>& knots,
                                                   const C2AttemptResult& attempt) {
  if (attempt.violating_sample_indices.empty()) {
    return knots;
  }

  const std::vector<double> knot_parameters = ChordLengthParameters(knots);
  struct CandidateKnot {
    double parameter = 0.0;
    JointArray q{};
  };

  std::vector<CandidateKnot> candidates;
  candidates.reserve(knots.size() + attempt.violating_sample_indices.size());
  for (std::size_t i = 0; i < knots.size(); ++i) {
    candidates.push_back({knot_parameters[i], knots[i]});
  }
  for (const std::size_t sample_index : attempt.violating_sample_indices) {
    candidates.push_back({attempt.spline.sample_parameters[sample_index],
                          attempt.spline.path[sample_index]});
  }

  std::sort(candidates.begin(), candidates.end(), [](const CandidateKnot& a,
                                                     const CandidateKnot& b) {
    return a.parameter < b.parameter;
  });

  std::vector<JointArray> repaired_knots;
  repaired_knots.reserve(candidates.size());
  for (const CandidateKnot& candidate : candidates) {
    if (!repaired_knots.empty() &&
        JointDistance(repaired_knots.back(), candidate.q) < 1.0e-5) {
      continue;
    }
    repaired_knots.push_back(candidate.q);
  }
  return repaired_knots;
}

void PrintPose(const std::string& label, Ur5Scene& scene, const JointArray& q) {
  scene.SetConfiguration(q);
  const auto tool = scene.ToolPosition();
  std::cout << label << " tool position: [" << std::fixed << std::setprecision(3)
            << tool[0] << ", " << tool[1] << ", " << tool[2] << "]\n";
}

JointArray SelectHome(Ur5Scene& scene) {
  std::optional<JointArray> selected;
  for (std::size_t i = 0; i < kHomeCandidates.size(); ++i) {
    const JointArray& candidate = kHomeCandidates[i];
    scene.SetConfiguration(candidate);
    const auto tool = scene.ToolPosition();
    const auto contacts = scene.ObstacleContactPairs(candidate);
    const auto clearance_violations =
        scene.ObstacleClearancePairs(candidate, kPlanningObstacleClearance);
    const bool before_ring = tool[0] > kRingPlaneX + 0.04;
    const bool valid = contacts.empty() && clearance_violations.empty() && before_ring;
    std::cout << "Home candidate " << i << " tool position: [" << std::fixed
              << std::setprecision(3) << tool[0] << ", " << tool[1] << ", " << tool[2]
              << "], valid=" << std::boolalpha << valid << '\n';
    if (!before_ring) {
      std::cout << "  rejected: tool is not on the home side of the ring plane\n";
    }
    for (const auto& contact : contacts) {
      std::cout << "  rejected obstacle contact: " << contact << '\n';
    }
    for (const auto& clearance : clearance_violations) {
      std::cout << "  rejected obstacle clearance: " << clearance << '\n';
    }
    if (valid && !selected.has_value()) {
      selected = candidate;
    }
  }
  if (selected.has_value()) {
    return *selected;
  }
  throw std::runtime_error("No collision-free home candidate before the ring found");
}

JointArray SelectGoal(Ur5Scene& scene) {
  std::optional<JointArray> selected;
  for (std::size_t i = 0; i < kGoalCandidates.size(); ++i) {
    const JointArray& candidate = kGoalCandidates[i];
    scene.SetConfiguration(candidate);
    const auto tool = scene.ToolPosition();
    const auto contacts = scene.ObstacleContactPairs(candidate);
    const auto clearance_violations =
        scene.ObstacleClearancePairs(candidate, kPlanningObstacleClearance);
    bool shelf_window_aligned = false;
    for (const WindowSpec& window : kWindowSpecs) {
      if (std::abs(window.plane_x - kShelfWindowPlaneX) < 1e-9 &&
          ToolAlignedWithWindow(tool, window)) {
        shelf_window_aligned = true;
      }
    }
    const bool after_ring = tool[0] < kShelfGoalTargetX;
    const bool shelf_height = tool[2] > 0.10 && tool[2] < 0.65;
    const bool valid = contacts.empty() && clearance_violations.empty() && after_ring &&
                       shelf_height && shelf_window_aligned;
    std::cout << "Goal candidate " << i << " tool position: [" << std::fixed
              << std::setprecision(3) << tool[0] << ", " << tool[1] << ", " << tool[2]
              << "], valid=" << std::boolalpha << valid << '\n';
    if (!after_ring) {
      std::cout << "  rejected: tool has not crossed the shelf finish line deeply enough\n";
    }
    if (!shelf_height) {
      std::cout << "  rejected: tool is outside the shelf height band\n";
    }
    if (!shelf_window_aligned) {
      std::cout << "  rejected: tool is not aligned with a shelf gap finish line\n";
    }
    for (const auto& contact : contacts) {
      std::cout << "  rejected obstacle contact: " << contact << '\n';
    }
    for (const auto& clearance : clearance_violations) {
      std::cout << "  rejected obstacle clearance: " << clearance << '\n';
    }
    if (valid && !selected.has_value()) {
      selected = candidate;
    }
  }
  if (selected.has_value()) {
    return *selected;
  }
  throw std::runtime_error("No collision-free shelf-side goal candidate found");
}

std::shared_ptr<ob::RealVectorStateSpace> MakePlanningStateSpace(
    Ur5Scene& scene,
    const JointArray& goal_q,
    bool use_goal_biased_sampler = true) {
  auto space = std::make_shared<ob::RealVectorStateSpace>(kDof);
  ob::RealVectorBounds bounds(kDof);
  for (int i = 0; i < kDof; ++i) {
    const auto [lo, hi] = scene.JointBounds(i);
    bounds.setLow(i, lo);
    bounds.setHigh(i, hi);
  }
  space->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.001);
  if (use_goal_biased_sampler) {
    space->setStateSamplerAllocator([goal_q](const ob::StateSpace* sampler_space) {
      return std::make_shared<GoalBiasedStateSampler>(sampler_space, goal_q, kGoalSampleBias);
    });
  }
  return space;
}

ob::PlannerPtr MakePlanner(const ob::SpaceInformationPtr& space_information,
                           PlannerKind planner_kind) {
  if (planner_kind == PlannerKind::kInformedRrtStar) {
    auto rrt_star = std::make_shared<og::InformedRRTstar>(space_information);
    rrt_star->setRange(0.35);
    rrt_star->setGoalBias(kGoalSampleBias);
    rrt_star->setDelayCC(true);
    return rrt_star;
  }
  auto rrt_connect = std::make_shared<og::RRTConnect>(space_information);
  rrt_connect->setRange(0.35);
  return rrt_connect;
}

void SetStartAndGoal(og::SimpleSetup& setup,
                     const ob::StateSpacePtr& space,
                     const JointArray& home_q,
                     const JointArray& goal_q) {
  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  FillState(home_q, start);
  FillState(goal_q, goal);
  setup.setStartAndGoalStates(start, goal, 0.04);
}

PlanResult FinalizeGeometricPath(Ur5Scene& scene,
                                 og::SimpleSetup& setup,
                                 const og::PathGeometric& raw_path,
                                 const JointArray& requested_start,
                                 const JointArray& requested_goal,
                                 PlannerKind planner_kind,
                                 std::string_view path_source,
                                 bool exact_solution,
                                 int solve_attempts,
                                 int approximate_attempts,
                                 std::chrono::steady_clock::time_point start_time,
                                 std::chrono::steady_clock::time_point deadline,
                                 double sampler_goal_bias = kGoalSampleBias) {
  using Clock = std::chrono::steady_clock;
  auto remaining_seconds = [&]() {
    return std::chrono::duration<double>(deadline - Clock::now()).count();
  };

  const std::size_t raw_state_count = raw_path.getStateCount();
  og::PathGeometric path(raw_path);

  setup.getPathSimplifier()->reduceVertices(path);
  if (remaining_seconds() > 0.12) {
    setup.getPathSimplifier()->ropeShortcutPath(path, 0.20, 0.10);
  }
  if (remaining_seconds() > 0.05) {
    setup.getPathSimplifier()->partialShortcutPath(path, 80, 24, 1.0);
  }
  const auto [still_valid, repaired] = path.checkAndRepair(64);
  bool shortcut_valid = still_valid || repaired;
  if (!still_valid && !repaired) {
    path = raw_path;
  }

  const std::size_t simplified_state_count = path.getStateCount();
  const std::vector<JointArray> shortcut_knots = PathStatesToJoints(path);
  const std::vector<JointArray> raw_knots = PathStatesToJoints(raw_path);
  const std::vector<JointArray> shortcut_linear =
      SampleLinearPath(shortcut_knots, kInterpolatedPathStates);
  const std::vector<JointArray> raw_linear =
      SampleLinearPath(raw_knots, kInterpolatedPathStates);
  const bool shortcut_passes_ring = PathPassesRingOpening(scene, shortcut_linear);
  const bool raw_passes_ring = PathPassesRingOpening(scene, raw_linear);
  std::vector<JointArray> spline_knots = shortcut_passes_ring ? shortcut_knots : raw_knots;
  std::vector<JointArray> reusable_path_knots = spline_knots;
  C2AttemptResult best_attempt;
  bool using_c2 = false;
  std::string fallback_source = "none";
  int repair_iterations = 0;
  std::size_t initial_knot_count = spline_knots.size();

  while (remaining_seconds() > kMinimumSplineAttemptBudgetSeconds &&
         spline_knots.size() <= kMaxC2SplineKnots &&
         repair_iterations <= kMaxC2RepairIterations) {
    best_attempt = TryC2Spline(scene, spline_knots, kPlanningObstacleClearance);
    if (best_attempt.planning_clearance_violations == 0) {
      using_c2 = true;
      break;
    }

    const std::vector<JointArray> repaired_knots =
        AddViolatingSamplesAsKnots(spline_knots, best_attempt);
    if (repaired_knots.size() == spline_knots.size()) {
      break;
    }
    spline_knots = repaired_knots;
    ++repair_iterations;
  }

  std::vector<JointArray> planned_path;
  if (using_c2) {
    planned_path = best_attempt.spline.path;
  } else {
    const int shortcut_planning_violations =
        JointPathClearanceViolationCount(scene, shortcut_linear, kPlanningObstacleClearance);
    const int raw_planning_violations =
        JointPathClearanceViolationCount(scene, raw_linear, kPlanningObstacleClearance);
    const int shortcut_hard_violations =
        JointPathClearanceViolationCount(scene, shortcut_linear, kRequiredObstacleClearance);
    const int raw_hard_violations =
        JointPathClearanceViolationCount(scene, raw_linear, kRequiredObstacleClearance);

    if (raw_passes_ring && raw_planning_violations == 0) {
      planned_path = raw_linear;
      fallback_source = "raw OMPL linear";
    } else if (shortcut_passes_ring && shortcut_planning_violations == 0) {
      planned_path = shortcut_linear;
      fallback_source = "shortcut linear";
    } else if (raw_passes_ring && raw_hard_violations == 0) {
      planned_path = raw_linear;
      fallback_source = "raw OMPL linear hard-clearance only";
    } else if (shortcut_passes_ring && shortcut_hard_violations == 0) {
      planned_path = shortcut_linear;
      fallback_source = "shortcut linear hard-clearance only";
    } else {
      throw std::runtime_error(
          "No fallback linear path preserved ring passage and hard clearance; "
          "raw_passes_ring=" +
          std::string(raw_passes_ring ? "true" : "false") +
          ", raw_hard_violations=" + std::to_string(raw_hard_violations) +
          ", shortcut_passes_ring=" + std::string(shortcut_passes_ring ? "true" : "false") +
          ", shortcut_hard_violations=" + std::to_string(shortcut_hard_violations));
    }
  }

  OrientPathToRequestedEndpoints(planned_path, reusable_path_knots, requested_start, requested_goal);

  const int final_planning_clearance_violations =
      JointPathClearanceViolationCount(scene, planned_path, kPlanningObstacleClearance);
  const int final_hard_clearance_violations =
      JointPathClearanceViolationCount(scene, planned_path, kRequiredObstacleClearance);
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start_time).count();
  const double budget_seconds = std::chrono::duration<double>(deadline - start_time).count();

  std::cout << "OMPL planner: " << PlannerName(planner_kind) << '\n';
  std::cout << "OMPL path source: " << path_source << '\n';
  std::cout << "OMPL goal sample bias: " << sampler_goal_bias << '\n';
  std::cout << "OMPL raw path states: " << raw_state_count << '\n';
  std::cout << "OMPL exact solution: " << std::boolalpha << exact_solution << '\n';
  std::cout << "OMPL solve attempts: " << solve_attempts << '\n';
  std::cout << "OMPL approximate attempts rejected: " << approximate_attempts << '\n';
  std::cout << "OMPL shortcut-simplified hard knots: " << simplified_state_count << '\n';
  std::cout << "OMPL shortcut path accepted: " << std::boolalpha << shortcut_valid << '\n';
  std::cout << "Shortcut linear path passes ring opening: " << std::boolalpha
            << shortcut_passes_ring << '\n';
  std::cout << "Raw linear path passes ring opening: " << std::boolalpha << raw_passes_ring
            << '\n';
  std::cout << "Spline initial hard knots: " << initial_knot_count << '\n';
  std::cout << "Spline final hard knots: " << spline_knots.size() << '\n';
  std::cout << "Spline repair iterations: " << repair_iterations << '\n';
  std::cout << "Planner used C2 spline: " << std::boolalpha << using_c2 << '\n';
  std::cout << "Planning time budget: " << budget_seconds << " s\n";
  std::cout << "Planning wall time: " << elapsed_ms << " ms\n";
  if (using_c2) {
    std::cout << "C2 natural cubic spline samples: " << best_attempt.spline.path.size() << '\n';
    std::cout << "C2 spline planning-clearance violation states: "
              << best_attempt.planning_clearance_violations << '\n';
    std::cout << std::scientific << std::setprecision(3)
              << "C2 spline max knot position error: "
              << best_attempt.spline.max_knot_position_error << " rad\n"
              << "C2 spline max C1 discontinuity: "
              << best_attempt.spline.max_c1_discontinuity << " rad/path-unit\n"
              << "C2 spline max C2 discontinuity: "
              << best_attempt.spline.max_c2_discontinuity << " rad/path-unit^2\n"
              << std::defaultfloat << std::setprecision(6);
  } else {
    std::cout << "Planner fallback: " << fallback_source << '\n';
  }
  std::cout << "Final planned path planning-clearance violation states: "
            << final_planning_clearance_violations << '\n';
  std::cout << "Final planned path hard-clearance violation states: "
            << final_hard_clearance_violations << '\n';

  if (using_c2 && final_planning_clearance_violations > 0) {
    throw std::runtime_error("Final planned path violates planning clearance");
  }
  if (final_hard_clearance_violations > 0) {
    throw std::runtime_error("Final planned path violates hard clearance");
  }

  PlanResult result;
  result.path = std::move(planned_path);
  result.reusable_path_knots = std::move(reusable_path_knots);
  result.used_c2_spline = using_c2;
  result.path_kind = using_c2 ? "C2 spline smoothed path" : fallback_source;
  result.plan_source = std::string(path_source);
  result.solve_attempts = solve_attempts;
  result.approximate_attempts_rejected = approximate_attempts;
  result.spline_repair_iterations = repair_iterations;
  result.raw_state_count = raw_state_count;
  result.simplified_state_count = simplified_state_count;
  result.planning_wall_ms = elapsed_ms;
  return result;
}

PlanResult RunPlanPipeline(Ur5Scene& scene,
                           og::SimpleSetup& setup,
                           const ob::PlannerPtr& planner,
                           const JointArray& requested_start,
                           const JointArray& requested_goal,
                           PlannerKind planner_kind,
                           bool preserve_planner_between_attempts = false,
                           std::string_view path_source = "fresh OMPL solve",
                           double planning_budget_seconds = kPlanningTimeBudgetSeconds,
                           double sampler_goal_bias = kGoalSampleBias) {
  using Clock = std::chrono::steady_clock;
  const auto start_time = Clock::now();
  const auto deadline =
      start_time + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(planning_budget_seconds));
  auto remaining_seconds = [&]() {
    return std::chrono::duration<double>(deadline - Clock::now()).count();
  };

  setup.getProblemDefinition()->clearSolutionPaths();

  bool exact_solution = false;
  int solve_attempts = 0;
  int approximate_attempts = 0;
  while (remaining_seconds() > kMinimumSplineAttemptBudgetSeconds) {
    const double usable_time = remaining_seconds() - kMinimumSplineAttemptBudgetSeconds;
    if (usable_time < kMinOmplSolveAttemptSeconds) {
      break;
    }
    const double solve_budget =
        preserve_planner_between_attempts
            ? usable_time
            : std::min(kMaxOmplSolveAttemptSeconds,
                       std::max(kMinOmplSolveAttemptSeconds, usable_time));
    if (solve_attempts > 0) {
      setup.getProblemDefinition()->clearSolutionPaths();
      if (!preserve_planner_between_attempts) {
        planner->clear();
      }
    }
    ++solve_attempts;
    const auto solved = setup.solve(solve_budget);
    if (solved == ob::PlannerStatus::EXACT_SOLUTION) {
      exact_solution = true;
      break;
    }
    if (solved) {
      ++approximate_attempts;
    }
  }
  if (!exact_solution) {
    std::ostringstream message;
    message << "OMPL did not find an exact path inside the " << std::fixed
            << std::setprecision(0) << planning_budget_seconds * 1000.0
            << " ms budget; refusing partial fallback";
    throw std::runtime_error(message.str());
  }

  return FinalizeGeometricPath(scene,
                               setup,
                               setup.getSolutionPath(),
                               requested_start,
                               requested_goal,
                               planner_kind,
                               path_source,
                               exact_solution,
                               solve_attempts,
                               approximate_attempts,
                               start_time,
                               deadline,
                               sampler_goal_bias);
}

PlanResult PlanPath(Ur5Scene& scene,
                    const JointArray& home_q,
                    const JointArray& goal_q,
                    PlannerKind planner_kind = PlannerKind::kRrtConnect,
                    std::string_view path_source = "fresh OMPL solve",
                    double planning_budget_seconds = kPlanningTimeBudgetSeconds,
                    bool preserve_planner_between_attempts = false) {
  auto space =
      MakePlanningStateSpace(scene, goal_q, planner_kind != PlannerKind::kExperienceRrtConnect);
  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&scene](const ob::State* state) {
    return scene.IsStateValid(StateToJoints(state));
  });
  const ob::PlannerPtr planner = MakePlanner(setup.getSpaceInformation(), planner_kind);
  setup.setPlanner(planner);
  SetStartAndGoal(setup, space, home_q, goal_q);
  setup.setup();
  const double sampler_goal_bias =
      planner_kind == PlannerKind::kExperienceRrtConnect ? 0.0 : kGoalSampleBias;
  return RunPlanPipeline(scene,
                         setup,
                         planner,
                         home_q,
                         goal_q,
                         planner_kind,
                         preserve_planner_between_attempts,
                         path_source,
                         planning_budget_seconds,
                         sampler_goal_bias);
}

PlanResult PlanFromReusablePath(Ur5Scene& scene,
                                const JointArray& home_q,
                                const JointArray& goal_q,
                                const std::vector<JointArray>& reusable_path_knots,
                                PlannerKind planner_kind = PlannerKind::kRrtConnect) {
  using Clock = std::chrono::steady_clock;
  const auto start_time = Clock::now();
  const auto deadline =
      start_time + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(kPlanningTimeBudgetSeconds));

  auto space = MakePlanningStateSpace(scene, goal_q);
  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&scene](const ob::State* state) {
    return scene.IsStateValid(StateToJoints(state));
  });
  setup.setPlanner(MakePlanner(setup.getSpaceInformation(), planner_kind));
  SetStartAndGoal(setup, space, home_q, goal_q);
  setup.setup();

  og::PathGeometric raw_path =
      JointsToPathGeometric(setup.getSpaceInformation(), reusable_path_knots);
  const auto [still_valid, repaired] = raw_path.checkAndRepair(64);
  if (!still_valid && !repaired) {
    throw std::runtime_error("Cached OMPL path is no longer valid");
  }

  return FinalizeGeometricPath(scene,
                               setup,
                               raw_path,
                               home_q,
                               goal_q,
                               planner_kind,
                               "cached reusable path",
                               true,
                               0,
                               0,
                               start_time,
                               deadline);
}


void ValidatePlannedPath(Ur5Scene& scene, const std::vector<JointArray>& path) {
  int contact_states = 0;
  int clearance_violation_states = 0;
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    if (scene.ObstacleContactCount() > 0) {
      ++contact_states;
    }
    if (scene.ObstacleClearanceViolationCount() > 0) {
      ++clearance_violation_states;
    }
  }
  std::cout << "Required obstacle clearance: " << kRequiredObstacleClearance << " m\n";
  std::cout << "OMPL obstacle clearance: " << kOmplObstacleClearance << " m\n";
  std::cout << "Planning obstacle clearance: " << kPlanningObstacleClearance << " m\n";
  std::cout << "Planned path obstacle-contact states: " << contact_states << '\n';
  std::cout << "Planned path clearance-violation states: " << clearance_violation_states << '\n';
  if (contact_states > 0 || clearance_violation_states > 0) {
    throw std::runtime_error("Planned path violates obstacle contact/clearance constraints");
  }
}

void WritePlannedPath(const std::filesystem::path& path_file,
                      Ur5Scene& scene,
                      const std::vector<JointArray>& path) {
  std::ofstream out(path_file);
  out << "index,q0,q1,q2,q3,q4,q5,tool_x,tool_y,tool_z,obstacle_contacts,"
         "clearance_violations,min_obstacle_clearance\n";
  for (std::size_t i = 0; i < path.size(); ++i) {
    scene.SetConfiguration(path[i]);
    const int obstacle_contacts = scene.ObstacleContactCount();
    const int clearance_violations = scene.ObstacleClearanceViolationCount();
    const double min_obstacle_clearance = scene.MinimumObstacleClearance();
    const auto tool = scene.ToolPosition();
    out << i;
    for (const double q : path[i]) {
      out << ',' << q;
    }
    out << ',' << tool[0] << ',' << tool[1] << ',' << tool[2] << ',' << obstacle_contacts << ','
        << clearance_violations << ',' << min_obstacle_clearance << '\n';
  }
}

void ExecutePath(const std::filesystem::path& trace_file,
                 Ur5Scene& scene,
                 const JointArray& home_q,
                 const std::vector<JointArray>& path,
                 bool live_view,
                 bool live_realtime) {
  using Clock = std::chrono::steady_clock;
  const auto execute_start = Clock::now();
  const std::vector<std::array<double, 3>> planned_tool_path = ComputeToolPath(scene, path);
  const auto tool_path_ready = Clock::now();
  scene.ResetTo(home_q);
  LivePidViewer viewer(scene, planned_tool_path, live_view);
  const auto viewer_ready = Clock::now();
  viewer.Render(0, 0);
  viewer.ResetClock();
  JointArray integral{};
  std::ofstream out(trace_file);
  out << "step,target_index,q0,q1,q2,q3,q4,q5,tool_x,tool_y,tool_z,obstacle_contacts,"
         "clearance_violations,min_obstacle_clearance\n";
  const auto trace_ready = Clock::now();

  int step = 0;
  int contact_steps = 0;
  int clearance_violation_steps = 0;
  int trace_rows = 0;
  int rendered_frames = viewer.active() ? 1 : 0;
  double max_tracking_error = 0.0;
  double sum_tracking_error = 0.0;
  double render_seconds = 0.0;
  double pace_seconds = 0.0;
  double csv_seconds = 0.0;
  double pid_step_seconds = 0.0;
  double diagnostics_seconds = 0.0;
  auto next_progress_log = Clock::now() + std::chrono::seconds(1);
  auto next_render_time = Clock::now();
  const double trajectory_duration = EstimateExecutionDuration(path);
  const double total_execution_duration = trajectory_duration + kFinalHoldSeconds;
  scene.SetSimulationTimestep(kControlPeriodSeconds);
  const double timestep = kControlPeriodSeconds;
  const int total_control_steps =
      static_cast<int>(std::ceil(total_execution_duration / timestep));
  const auto control_start_wall_time = Clock::now();
  for (int control_step = 0; control_step <= total_control_steps; ++control_step) {
    const double elapsed_sim_time = static_cast<double>(control_step) * timestep;
    const bool holding_final_target = elapsed_sim_time >= trajectory_duration;
    const double command_time = std::min(elapsed_sim_time, trajectory_duration);
    const double progress = trajectory_duration > 0.0 ? command_time / trajectory_duration : 1.0;
    const JointArray commanded_target =
        holding_final_target ? path.back() : EvaluateSampledPath(path, progress);
    const std::size_t target_index = std::min<std::size_t>(
        static_cast<std::size_t>(progress * static_cast<double>(path.size() - 1)),
        path.size() - 1);

    const auto pid_step_start = Clock::now();
    scene.ApplyPidStep(commanded_target, integral);
    const auto pid_step_end = Clock::now();
    pid_step_seconds += std::chrono::duration<double>(pid_step_end - pid_step_start).count();

    const auto diagnostics_start = Clock::now();
    const auto q = scene.CurrentConfiguration();
    const double tracking_error = MaxAbsJointError(q, commanded_target);
    max_tracking_error = std::max(max_tracking_error, tracking_error);
    sum_tracking_error += tracking_error;
    const int obstacle_contacts = scene.ObstacleContactCount();
    contact_steps += obstacle_contacts > 0 ? 1 : 0;
    const auto diagnostics_end = Clock::now();
    diagnostics_seconds +=
        std::chrono::duration<double>(diagnostics_end - diagnostics_start).count();

    const bool final_step = control_step == total_control_steps;
    const bool should_record_trace =
        step % kExecutionTraceStride == 0 || obstacle_contacts > 0 || final_step;
    if (should_record_trace) {
      const auto csv_start = Clock::now();
      const int clearance_violations = scene.ObstacleClearanceViolationCount();
      const double min_obstacle_clearance = scene.MinimumObstacleClearance();
      clearance_violation_steps += clearance_violations > 0 ? 1 : 0;
      const auto tool = scene.ToolPosition();
      out << step << ',' << target_index;
      for (const double value : q) {
        out << ',' << value;
      }
      out << ',' << tool[0] << ',' << tool[1] << ',' << tool[2] << ',' << obstacle_contacts
          << ',' << clearance_violations << ',' << min_obstacle_clearance << '\n';
      ++trace_rows;
      const auto csv_end = Clock::now();
      csv_seconds += std::chrono::duration<double>(csv_end - csv_start).count();
    }

    ++step;

    const auto now = Clock::now();
    if (viewer.active() && now >= next_render_time) {
      const auto render_start = Clock::now();
      viewer.Render(target_index, step);
      const auto render_end = Clock::now();
      if (viewer.active()) {
        ++rendered_frames;
      }
      render_seconds += std::chrono::duration<double>(render_end - render_start).count();
      next_render_time = now + std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(1.0 / kLiveRenderHz));
    }

    if (live_realtime) {
      const auto pace_start = Clock::now();
      const auto target_time = control_start_wall_time +
                               std::chrono::duration_cast<Clock::duration>(
                                   std::chrono::duration<double>(
                                       static_cast<double>(control_step + 1) * timestep));
      const auto before_sleep = Clock::now();
      if (target_time > before_sleep) {
        std::this_thread::sleep_until(target_time);
      }
      const auto pace_end = Clock::now();
      pace_seconds += std::chrono::duration<double>(pace_end - pace_start).count();
    }

    if (now >= next_progress_log) {
      std::cout << "PID progress: target " << (target_index + 1) << "/" << path.size()
                << ", trajectory time " << std::fixed << std::setprecision(2) << command_time
                << "/" << trajectory_duration << " s, executed steps " << step << ", sim time "
                << scene.data()->time << " s\n"
                << std::defaultfloat << std::setprecision(6);
      next_progress_log = now + std::chrono::seconds(1);
    }
  }

  const auto execute_end = Clock::now();
  const double final_error = MaxAbsJointError(scene.CurrentConfiguration(), path.back());
  const double mean_tracking_error = step > 0 ? sum_tracking_error / step : 0.0;
  const double tool_path_ms =
      std::chrono::duration<double, std::milli>(tool_path_ready - execute_start).count();
  const double viewer_setup_ms =
      std::chrono::duration<double, std::milli>(viewer_ready - tool_path_ready).count();
  const double trace_setup_ms =
      std::chrono::duration<double, std::milli>(trace_ready - viewer_ready).count();
  const double total_ms =
      std::chrono::duration<double, std::milli>(execute_end - execute_start).count();
  std::cout << "PID timing total: " << total_ms << " ms\n";
  std::cout << "PID timing planned tool path: " << tool_path_ms << " ms\n";
  std::cout << "PID timing live viewer setup: " << viewer_setup_ms << " ms\n";
  std::cout << "PID timing trace setup: " << trace_setup_ms << " ms\n";
  std::cout << "PID timing mj_step: " << (pid_step_seconds * 1000.0) << " ms\n";
  std::cout << "PID timing clearance/diagnostics: " << (diagnostics_seconds * 1000.0) << " ms\n";
  std::cout << "PID timing CSV writes: " << (csv_seconds * 1000.0) << " ms\n";
  std::cout << "PID timing live render: " << (render_seconds * 1000.0) << " ms\n";
  std::cout << "PID timing realtime pacing sleep: " << (pace_seconds * 1000.0) << " ms\n";
  std::cout << "PID trajectory duration: " << trajectory_duration << " s\n";
  std::cout << "PID final hold duration: " << kFinalHoldSeconds << " s\n";
  std::cout << "PID execution steps: " << step << '\n';
  std::cout << "PID trace rows written: " << trace_rows << '\n';
  std::cout << "PID trace stride: " << kExecutionTraceStride << '\n';
  std::cout << "PID simulated time: " << scene.data()->time << " s\n";
  std::cout << "MuJoCo timestep: " << scene.model()->opt.timestep << " s\n";
  std::cout << "PID rendered frames: " << rendered_frames << '\n';
  std::cout << "PID live realtime pacing: " << std::boolalpha << live_realtime << '\n';
  std::cout << "PID final max joint error: " << final_error << " rad\n";
  std::cout << "PID mean waypoint tracking error: " << mean_tracking_error << " rad\n";
  std::cout << "PID max waypoint tracking error: " << max_tracking_error << " rad\n";
  std::cout << "PID obstacle-contact steps: " << contact_steps << '\n';
  std::cout << "PID clearance-violation trace rows: " << clearance_violation_steps << '\n';

  if (final_error > 0.20) {
    throw std::runtime_error("PID controller did not reach the final planned target closely enough");
  }
  if (contact_steps > 0) {
    throw std::runtime_error("Executed trajectory made obstacle contact");
  }
}

void ReportRingPassage(Ur5Scene& scene, const std::vector<JointArray>& path) {
  const std::vector<std::array<double, 3>> tools = ComputeToolPathSamples(scene, path);
  const std::array<int, kWindowSpecs.size()> counts = WindowHitCounts(tools);

  for (const double layer_plane : kWindowLayerPlanes) {
    int plane_samples = 0;
    for (const auto& tool : tools) {
      if (std::abs(tool[0] - layer_plane) < kWindowPlaneTolerance) {
        ++plane_samples;
      }
    }

    int layer_opening_samples = 0;
    for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
      if (std::abs(kWindowSpecs[i].plane_x - layer_plane) < 1e-9) {
        layer_opening_samples += counts[i];
      }
    }

    std::cout << "Window layer x=" << layer_plane << " plane samples: " << plane_samples
              << '\n';
    std::cout << "Window layer x=" << layer_plane
              << " opening samples: " << layer_opening_samples << '\n';

    if (ToolPathCrossesPlane(tools, layer_plane) && plane_samples == 0) {
      throw std::runtime_error("Path crossed a window layer but had no layer-plane samples");
    }
    if (ToolPathCrossesPlane(tools, layer_plane) && layer_opening_samples == 0) {
      throw std::runtime_error("Path crossed a window layer but not through an opening");
    }
  }

  for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
    std::cout << "Window " << kWindowSpecs[i].name << " path samples: " << counts[i] << '\n';
  }
}

}  // namespace
