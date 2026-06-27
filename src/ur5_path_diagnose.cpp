#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

constexpr int kDof = 6;
constexpr mjtNum kDistanceCutoff = 0.30;

using JointArray = std::array<double, kDof>;

struct MjModelDeleter {
  void operator()(mjModel* model) const { mj_deleteModel(model); }
};

struct MjDataDeleter {
  void operator()(mjData* data) const { mj_deleteData(data); }
};

struct JointBinding {
  std::array<int, kDof> qpos_addr{};
};

struct SampleLocation {
  std::size_t row = 0;
  double alpha_to_next = 0.0;
};

struct ClosestPair {
  double distance = std::numeric_limits<double>::infinity();
  SampleLocation location;
  int robot_geom = -1;
  int ring_geom = -1;
  std::array<mjtNum, 6> fromto{};
};

struct ContactHit {
  SampleLocation location;
  int geom1 = -1;
  int geom2 = -1;
  double distance = 0.0;
};

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
                  JointBinding& binding) {
  std::array<int, kDof> qpos_addr{};
  for (int i = 0; i < kDof; ++i) {
    const int joint_id = mj_name2id(model, mjOBJ_JOINT, joints[i]);
    if (joint_id < 0) {
      return false;
    }
    qpos_addr[i] = model->jnt_qposadr[joint_id];
  }
  binding.qpos_addr = qpos_addr;
  return true;
}

JointBinding BindRobot(mjModel* model) {
  JointBinding binding;
  constexpr std::array<const char*, kDof> kMenagerieJoints = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  constexpr std::array<const char*, kDof> kSimpleJoints = {
      "shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"};

  if (TryBindNames(model, kMenagerieJoints, binding) ||
      TryBindNames(model, kSimpleJoints, binding)) {
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

const char* NameOrUnnamed(const mjModel* model, mjtObj object_type, int id) {
  const char* name = mj_id2name(model, object_type, id);
  return name == nullptr ? "<unnamed>" : name;
}

std::string GeomLabel(const mjModel* model, int geom_id) {
  const int body_id = model->geom_bodyid[geom_id];
  std::ostringstream out;
  out << NameOrUnnamed(model, mjOBJ_GEOM, geom_id) << " [body="
      << NameOrUnnamed(model, mjOBJ_BODY, body_id) << ", group=" << model->geom_group[geom_id]
      << ", contype=" << model->geom_contype[geom_id]
      << ", conaffinity=" << model->geom_conaffinity[geom_id] << ']';
  return out.str();
}

bool IsRedRingGeom(const mjModel* model, int geom_id) {
  const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
  if (name == nullptr) {
    return false;
  }
  const std::string_view geom_name{name};
  return geom_name == "ring_left_bar" || geom_name == "ring_right_bar" ||
         geom_name == "ring_top_bar" || geom_name == "ring_bottom_bar";
}

bool IsRobotCollisionGeom(const mjModel* model, int geom_id) {
  return model->geom_group[geom_id] == 3 && !IsRedRingGeom(model, geom_id);
}

bool IsRobotRingPair(const mjModel* model, int geom1, int geom2) {
  return (IsRobotCollisionGeom(model, geom1) && IsRedRingGeom(model, geom2)) ||
         (IsRobotCollisionGeom(model, geom2) && IsRedRingGeom(model, geom1));
}

JointArray Lerp(const JointArray& a, const JointArray& b, double alpha) {
  JointArray result{};
  for (int i = 0; i < kDof; ++i) {
    result[i] = a[i] + alpha * (b[i] - a[i]);
  }
  return result;
}

std::string FormatLocation(const SampleLocation& location) {
  std::ostringstream out;
  out << "row " << location.row;
  if (location.alpha_to_next > 0.0) {
    out << " + " << std::fixed << std::setprecision(3) << location.alpha_to_next
        << " toward next row";
  }
  return out.str();
}

void AnalyzeSample(mjModel* model,
                   mjData* data,
                   const JointBinding& binding,
                   const std::vector<int>& robot_geoms,
                   const std::vector<int>& red_ring_geoms,
                   const JointArray& q,
                   SampleLocation location,
                   ClosestPair& closest,
                   int& samples_with_actual_contacts,
                   int& samples_with_negative_distance,
                   std::vector<ContactHit>& first_hits) {
  SetConfiguration(model, data, binding, q);

  bool actual_contact = false;
  for (int i = 0; i < data->ncon; ++i) {
    const mjContact& contact = data->contact[i];
    if (!IsRobotRingPair(model, contact.geom1, contact.geom2)) {
      continue;
    }
    actual_contact = true;
    if (first_hits.size() < 12) {
      first_hits.push_back({location, contact.geom1, contact.geom2, contact.dist});
    }
  }
  if (actual_contact) {
    ++samples_with_actual_contacts;
  }

  bool negative_distance = false;
  for (const int robot_geom : robot_geoms) {
    for (const int ring_geom : red_ring_geoms) {
      mjtNum fromto[6] = {};
      const mjtNum distance =
          mj_geomDistance(model, data, robot_geom, ring_geom, kDistanceCutoff, fromto);
      if (distance < 0.0) {
        negative_distance = true;
      }
      if (distance < closest.distance) {
        closest.distance = distance;
        closest.location = location;
        closest.robot_geom = robot_geom;
        closest.ring_geom = ring_geom;
        std::copy(fromto, fromto + 6, closest.fromto.begin());
      }
    }
  }
  if (negative_distance) {
    ++samples_with_negative_distance;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path scene_path =
      argc > 1 ? std::filesystem::path(argv[1])
               : std::filesystem::path(UR5_MUJOCO_OMPL_DEFAULT_SCENE);
  const std::filesystem::path path_file =
      argc > 2 ? std::filesystem::path(argv[2]) : std::filesystem::path("planned_path.csv");
  const int substeps_per_segment = argc > 3 ? std::max(1, std::stoi(argv[3])) : 16;

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

    const JointBinding binding = BindRobot(model.get());
    const std::vector<JointArray> path = ReadJointCsv(path_file);

    std::vector<int> robot_geoms;
    std::vector<int> red_ring_geoms;
    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
      if (IsRobotCollisionGeom(model.get(), geom_id)) {
        robot_geoms.push_back(geom_id);
      }
      if (IsRedRingGeom(model.get(), geom_id)) {
        red_ring_geoms.push_back(geom_id);
      }
    }

    if (robot_geoms.empty()) {
      throw std::runtime_error("No robot collision geoms found in group 3");
    }
    if (red_ring_geoms.empty()) {
      throw std::runtime_error("No red ring geoms found");
    }

    ClosestPair closest;
    int total_samples = 0;
    int samples_with_actual_contacts = 0;
    int samples_with_negative_distance = 0;
    std::vector<ContactHit> first_hits;

    for (std::size_t row = 0; row < path.size(); ++row) {
      AnalyzeSample(model.get(), data.get(), binding, robot_geoms, red_ring_geoms, path[row],
                    {.row = row, .alpha_to_next = 0.0}, closest,
                    samples_with_actual_contacts, samples_with_negative_distance, first_hits);
      ++total_samples;

      if (row + 1 >= path.size()) {
        continue;
      }
      for (int step = 1; step < substeps_per_segment; ++step) {
        const double alpha = static_cast<double>(step) / substeps_per_segment;
        const JointArray q = Lerp(path[row], path[row + 1], alpha);
        AnalyzeSample(model.get(), data.get(), binding, robot_geoms, red_ring_geoms, q,
                      {.row = row, .alpha_to_next = alpha}, closest,
                      samples_with_actual_contacts, samples_with_negative_distance, first_hits);
        ++total_samples;
      }
    }

    std::cout << "Loaded scene: " << scene_path << '\n';
    std::cout << "Loaded path: " << path_file << " (" << path.size() << " rows)\n";
    std::cout << "Substeps per segment: " << substeps_per_segment << '\n';
    std::cout << "Analyzed samples: " << total_samples << '\n';
    std::cout << "Robot collision geoms: " << robot_geoms.size() << '\n';
    std::cout << "Red ring geoms: " << red_ring_geoms.size() << '\n';
    for (const int ring_geom : red_ring_geoms) {
      std::cout << "  ring: " << GeomLabel(model.get(), ring_geom) << '\n';
    }

    std::cout << "Samples with actual MuJoCo robot-ring contacts: "
              << samples_with_actual_contacts << '\n';
    std::cout << "Samples with negative robot-ring geom distance: "
              << samples_with_negative_distance << '\n';
    std::cout << "Closest robot-ring distance: " << std::fixed << std::setprecision(6)
              << closest.distance << " m at " << FormatLocation(closest.location) << '\n';
    std::cout << "Closest pair:\n";
    std::cout << "  robot: " << GeomLabel(model.get(), closest.robot_geom) << '\n';
    std::cout << "  ring:  " << GeomLabel(model.get(), closest.ring_geom) << '\n';
    std::cout << "  witness robot/ring points: [" << closest.fromto[0] << ", "
              << closest.fromto[1] << ", " << closest.fromto[2] << "] -> ["
              << closest.fromto[3] << ", " << closest.fromto[4] << ", "
              << closest.fromto[5] << "]\n";

    if (!first_hits.empty()) {
      std::cout << "First actual contacts:\n";
      for (const ContactHit& hit : first_hits) {
        std::cout << "  " << FormatLocation(hit.location) << ": "
                  << GeomLabel(model.get(), hit.geom1) << " <-> "
                  << GeomLabel(model.get(), hit.geom2) << ", dist=" << hit.distance << '\n';
      }
    }

    return EXIT_SUCCESS;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return EXIT_FAILURE;
  }
}
