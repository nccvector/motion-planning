#include <mujoco/mujoco.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

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
constexpr double kOmplObstacleClearance = 0.015;
constexpr double kPlanningObstacleClearance = 0.015;
constexpr std::size_t kInterpolatedPathStates = 480;
constexpr double kPlanningTimeBudgetSeconds = 0.500;
constexpr int kMaxC2RepairIterations = 8;
constexpr std::size_t kMaxC2SplineKnots = 96;
constexpr double kMinimumSplineAttemptBudgetSeconds = 0.080;
constexpr int kCommandSubsteps = 4;
constexpr int kMaxSettleSteps = 240;

using JointArray = std::array<double, kDof>;

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

std::vector<JointArray> PathStatesToJoints(const og::PathGeometric& path) {
  std::vector<JointArray> result;
  result.reserve(path.getStateCount());
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    result.push_back(StateToJoints(path.getState(i)));
  }
  return result;
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

bool PathPassesRingOpening(Ur5Scene& scene, const std::vector<JointArray>& path) {
  for (const JointArray& q : path) {
    scene.SetConfiguration(q);
    const auto tool = scene.ToolPosition();
    const bool near_ring_plane = std::abs(tool[0] - kRingPlaneX) < 0.05;
    const bool inside_opening = std::abs(tool[1]) < kRingHalfOpeningY &&
                                std::abs(tool[2] - kRingCenterZ) < kRingHalfOpeningZ;
    if (near_ring_plane && inside_opening) {
      return true;
    }
  }
  return false;
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
  using Clock = std::chrono::steady_clock;
  const auto start_time = Clock::now();
  const auto deadline =
      start_time + std::chrono::duration<double>(kPlanningTimeBudgetSeconds);
  auto remaining_seconds = [&]() {
    return std::chrono::duration<double>(deadline - Clock::now()).count();
  };

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
  planner->setRange(0.35);
  setup.setPlanner(planner);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  FillState(home_q, start);
  FillState(goal_q, goal);
  setup.setStartAndGoalStates(start, goal, 0.04);
  setup.setup();

  const double solve_budget = std::max(0.05, remaining_seconds() * 0.98);
  const auto solved = setup.solve(solve_budget);
  if (!solved) {
    throw std::runtime_error("OMPL did not find a path inside the 500 ms budget");
  }
  const bool exact_solution = solved == ob::PlannerStatus::EXACT_SOLUTION;

  const std::size_t raw_state_count = setup.getSolutionPath().getStateCount();
  og::PathGeometric raw_path = setup.getSolutionPath();
  og::PathGeometric path(raw_path);

  setup.getPathSimplifier()->reduceVertices(path);
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
      throw std::runtime_error("No fallback linear path preserved ring passage and hard clearance");
    }
  }

  const int final_planning_clearance_violations =
      JointPathClearanceViolationCount(scene, planned_path, kPlanningObstacleClearance);
  const int final_hard_clearance_violations =
      JointPathClearanceViolationCount(scene, planned_path, kRequiredObstacleClearance);
  const double elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start_time).count();

  std::cout << "OMPL raw path states: " << raw_state_count << '\n';
  std::cout << "OMPL exact solution: " << std::boolalpha << exact_solution << '\n';
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
  std::cout << "Planning time budget: " << kPlanningTimeBudgetSeconds << " s\n";
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

  return planned_path;
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
  JointArray previous_target = home_q;
  for (std::size_t target_index = 0; target_index < path.size(); ++target_index) {
    const JointArray& target = path[target_index];
    for (int substep = 1; substep <= kCommandSubsteps; ++substep) {
      const JointArray commanded_target =
          BlendJoints(previous_target, target, static_cast<double>(substep) / kCommandSubsteps);
      for (int local_step = 0; local_step < kMaxSettleSteps; ++local_step) {
        scene.ApplyPidStep(commanded_target, integral);
        const auto q = scene.CurrentConfiguration();
        const double tracking_error = MaxAbsJointError(q, commanded_target);
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
        out << ',' << tool[0] << ',' << tool[1] << ',' << tool[2] << ',' << obstacle_contacts
            << ',' << clearance_violations << ',' << min_obstacle_clearance << '\n';
        ++step;
        if (MaxAbsJointError(q, commanded_target) < 0.004 &&
            MaxAbs(scene.CurrentVelocity()) < 0.025) {
          break;
        }
      }
    }
    previous_target = target;
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
    std::cout << "Planned path states: " << path.size() << '\n';
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
