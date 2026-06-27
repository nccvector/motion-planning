#include <mujoco/mujoco.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

constexpr int kDof = 6;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRingPlaneX = -0.45;
constexpr double kRingCenterZ = 0.36;
constexpr double kRingHalfOpeningY = 0.560;
constexpr double kRingHalfOpeningZ = 0.400;
// Plan with cushion; validate with the hard floor we actually require.
constexpr double kRequiredObstacleClearance = 0.010;
constexpr double kPlanningObstacleClearance = 0.025;

using JointArray = std::array<double, kDof>;

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

const std::array<JointArray, 10> kHomeCandidates = {{
    {-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0},
    {-1.20, -1.70, 1.75, -1.60, -1.5708, 0.0},
    {-0.85, -1.70, 1.80, -1.65, -1.5708, 0.0},
    {-0.40, -1.60, 1.70, -1.65, -1.5708, 0.0},
    {0.0, -1.60, 1.70, -1.65, -1.5708, 0.0},
    {0.0, -1.60, 2.40, -0.70, 0.25, 0.0},
    {0.0, -1.45, 2.30, -0.75, 0.25, 0.0},
    {0.0, -1.30, 2.15, -0.75, 0.25, 0.0},
    {0.15, -1.50, 2.35, -0.75, 0.20, 0.0},
    {-0.15, -1.50, 2.35, -0.75, 0.20, 0.0},
}};

const std::array<JointArray, 18> kGoalCandidates = {{
    {0.0, -0.45, 0.95, -0.75, -1.5708, 0.0},
    {0.0, -0.70, 1.20, -0.65, -1.5708, 0.0},
    {0.0, -0.85, 1.35, -0.55, -1.5708, 0.0},
    {0.12, -0.82, 1.32, -0.55, -1.5708, 0.0},
    {-0.12, -0.82, 1.32, -0.55, -1.5708, 0.0},
    {-0.35, -1.05, 1.55, -1.05, -1.5708, 0.0},
    {-0.55, -1.05, 1.55, -1.05, -1.5708, 0.0},
    {-0.80, -1.05, 1.60, -1.10, -1.5708, 0.0},
    {-1.05, -1.00, 1.55, -1.10, -1.5708, 0.0},
    {-1.25, -0.95, 1.45, -1.05, -1.5708, 0.0},
    {-1.45, -0.95, 1.45, -1.05, -1.5708, 0.0},
    {0.0, -0.45, 0.95, -0.75, 0.20, 0.0},
    {0.0, -0.70, 1.20, -0.65, 0.20, 0.0},
    {0.0, -0.85, 1.35, -0.55, 0.20, 0.0},
    {0.12, -0.82, 1.32, -0.55, 0.18, 0.0},
    {-0.12, -0.82, 1.32, -0.55, 0.18, 0.0},
    {0.0, -0.95, 1.45, -0.50, 0.20, 0.0},
    {-0.60, -0.95, 1.45, -0.65, 0.20, 0.0},
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
    for (int i = 0; i < kDof; ++i) {
      data_->qpos[qpos_addr_[i]] = q[i];
      data_->qvel[qvel_addr_[i]] = 0.0;
    }
    mj_forward(model_.get(), data_.get());
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
           ObstacleClearanceViolationCount(kPlanningObstacleClearance) == 0;
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

double MaxAbsJointError(const JointArray& a, const JointArray& b) {
  double error = 0.0;
  for (int i = 0; i < kDof; ++i) {
    error = std::max(error, std::abs(a[i] - b[i]));
  }
  return error;
}

double MaxAbs(const JointArray& values) {
  double result = 0.0;
  for (const double value : values) {
    result = std::max(result, std::abs(value));
  }
  return result;
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
    const bool before_ring = tool[0] > kRingPlaneX + 0.08;
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
    const bool after_ring = tool[0] < kRingPlaneX - 0.12;
    const bool shelf_height = tool[2] > 0.10 && tool[2] < 0.65;
    const bool valid = contacts.empty() && clearance_violations.empty() && after_ring && shelf_height;
    std::cout << "Goal candidate " << i << " tool position: [" << std::fixed
              << std::setprecision(3) << tool[0] << ", " << tool[1] << ", " << tool[2]
              << "], valid=" << std::boolalpha << valid << '\n';
    if (!after_ring) {
      std::cout << "  rejected: tool is not beyond the ring toward the shelf\n";
    }
    if (!shelf_height) {
      std::cout << "  rejected: tool is outside the shelf height band\n";
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

std::vector<JointArray> PlanPath(Ur5Scene& scene,
                                 const JointArray& home_q,
                                 const JointArray& goal_q) {
  auto space = std::make_shared<ob::RealVectorStateSpace>(kDof);
  ob::RealVectorBounds bounds(kDof);
  for (int i = 0; i < kDof; ++i) {
    const auto [lo, hi] = scene.JointBounds(i);
    bounds.setLow(i, lo);
    bounds.setHigh(i, hi);
  }
  space->setBounds(bounds);
  space->setLongestValidSegmentFraction(0.004);

  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&scene](const ob::State* state) {
    return scene.IsStateValid(StateToJoints(state));
  });

  auto planner = std::make_shared<og::RRTConnect>(setup.getSpaceInformation());
  planner->setRange(0.16);
  setup.setPlanner(planner);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  FillState(home_q, start);
  FillState(goal_q, goal);
  setup.setStartAndGoalStates(start, goal, 0.04);
  setup.setup();

  const auto solved = setup.solve(8.0);
  if (!solved) {
    throw std::runtime_error("OMPL did not find a collision-free path");
  }

  const std::size_t raw_state_count = setup.getSolutionPath().getStateCount();
  setup.simplifySolution(1.0);

  og::PathGeometric path = setup.getSolutionPath();
  const std::size_t simplified_state_count = path.getStateCount();
  path.interpolate(240);
  std::cout << "OMPL raw path states: " << raw_state_count << '\n';
  std::cout << "OMPL simplified path states: " << simplified_state_count << '\n';

  std::vector<JointArray> result;
  result.reserve(path.getStateCount());
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    result.push_back(StateToJoints(path.getState(i)));
  }
  return result;
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
                 const std::vector<JointArray>& path) {
  scene.ResetTo(home_q);
  JointArray integral{};
  std::ofstream out(trace_file);
  out << "step,target_index,q0,q1,q2,q3,q4,q5,tool_x,tool_y,tool_z,obstacle_contacts,"
         "clearance_violations,min_obstacle_clearance\n";

  int step = 0;
  int contact_steps = 0;
  int clearance_violation_steps = 0;
  double max_tracking_error = 0.0;
  double sum_tracking_error = 0.0;
  for (std::size_t target_index = 0; target_index < path.size(); ++target_index) {
    const JointArray& target = path[target_index];
    for (int local_step = 0; local_step < 240; ++local_step) {
      scene.ApplyPidStep(target, integral);
      const auto q = scene.CurrentConfiguration();
      const double tracking_error = MaxAbsJointError(q, target);
      max_tracking_error = std::max(max_tracking_error, tracking_error);
      sum_tracking_error += tracking_error;
      const int obstacle_contacts = scene.ObstacleContactCount();
      const int clearance_violations = scene.ObstacleClearanceViolationCount();
      const double min_obstacle_clearance = scene.MinimumObstacleClearance();
      contact_steps += obstacle_contacts > 0 ? 1 : 0;
      clearance_violation_steps += clearance_violations > 0 ? 1 : 0;
      const auto tool = scene.ToolPosition();
      out << step << ',' << target_index;
      for (const double value : q) {
        out << ',' << value;
      }
      out << ',' << tool[0] << ',' << tool[1] << ',' << tool[2] << ',' << obstacle_contacts << ','
          << clearance_violations << ',' << min_obstacle_clearance << '\n';
      ++step;
      if (MaxAbsJointError(q, target) < 0.010 && MaxAbs(scene.CurrentVelocity()) < 0.05) {
        break;
      }
    }
  }

  const double final_error = MaxAbsJointError(scene.CurrentConfiguration(), path.back());
  const double mean_tracking_error = step > 0 ? sum_tracking_error / step : 0.0;
  std::cout << "PID execution steps: " << step << '\n';
  std::cout << "PID final max joint error: " << final_error << " rad\n";
  std::cout << "PID mean waypoint tracking error: " << mean_tracking_error << " rad\n";
  std::cout << "PID max waypoint tracking error: " << max_tracking_error << " rad\n";
  std::cout << "PID obstacle-contact steps: " << contact_steps << '\n';
  std::cout << "PID clearance-violation steps: " << clearance_violation_steps << '\n';

  if (final_error > 0.20) {
    throw std::runtime_error("PID controller did not reach the final planned target closely enough");
  }
  if (contact_steps > 0) {
    throw std::runtime_error("Executed trajectory made obstacle contact");
  }
  if (clearance_violation_steps > 0) {
    throw std::runtime_error("Executed trajectory violated obstacle clearance");
  }
}

void ReportRingPassage(Ur5Scene& scene, const std::vector<JointArray>& path) {
  int plane_samples = 0;
  int opening_samples = 0;
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    const auto tool = scene.ToolPosition();
    const bool near_ring_plane = std::abs(tool[0] - kRingPlaneX) < 0.05;
    const bool inside_opening = std::abs(tool[1]) < kRingHalfOpeningY &&
                                std::abs(tool[2] - kRingCenterZ) < kRingHalfOpeningZ;
    if (near_ring_plane) {
      ++plane_samples;
      if (inside_opening) {
        ++opening_samples;
      }
    }
  }
  std::cout << "Ring-plane path samples: " << plane_samples << '\n';
  std::cout << "Ring-opening path samples: " << opening_samples << '\n';
  if (plane_samples == 0) {
    throw std::runtime_error("Path did not pass through the ring plane");
  }
  if (opening_samples == 0) {
    throw std::runtime_error("Path crossed the ring plane but not through the opening");
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path scene_path =
      argc > 1 ? std::filesystem::path(argv[1])
               : std::filesystem::path(UR5_MUJOCO_OMPL_DEFAULT_SCENE);

  try {
    Ur5Scene scene(scene_path);
    std::cout << "Loaded scene: " << scene_path << '\n';
    std::cout << "Robot model: " << scene.robot_label() << '\n';
    std::cout << "Control mode: " << scene.ControlModeLabel() << '\n';
    std::cout << "MuJoCo model: nq=" << scene.model()->nq << " nv=" << scene.model()->nv
              << " nu=" << scene.model()->nu << '\n';

    const JointArray home = SelectHome(scene);
    PrintPose("Selected home", scene, home);
    const JointArray goal = SelectGoal(scene);
    PrintPose("Selected goal", scene, goal);

    const auto path = PlanPath(scene, home, goal);
    std::cout << "OMPL path states after interpolation: " << path.size() << '\n';
    ValidatePlannedPath(scene, path);
    ReportRingPassage(scene, path);

    WritePlannedPath("planned_path.csv", scene, path);
    ExecutePath("executed_trace.csv", scene, home, path);

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
