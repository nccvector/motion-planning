#include "ur5_clutter_core.hpp"

#include <charconv>
#include <limits>
#include <random>

namespace {

constexpr std::uint64_t kDefaultSeed = 20260627;
constexpr int kDefaultSamples = 200000;
constexpr int kKeepPerSide = 10;
constexpr double kMinJointDiversity = 0.55;
constexpr double kAnchorNoiseStddev = 0.22;

struct Candidate {
  JointArray q{};
  std::array<double, 3> tool{};
  double score = 0.0;
};

constexpr std::array<JointArray, 4> kNeedleLoopAnchors = {{
    {0.0, -1.60, 2.40, -0.70, 0.25, 0.0},
    {0.0, -1.45, 2.30, -0.75, 0.25, 0.0},
    {0.15, -1.50, 2.35, -0.75, 0.20, 0.0},
    {-0.15, -1.50, 2.35, -0.75, 0.20, 0.0},
}};

constexpr std::array<JointArray, 2> kShelfLoopAnchors = {{
    {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0},
    {0.0, -1.20, 1.35, -1.35, -1.5708, 0.0},
}};

template <std::size_t N>
double NearestAnchorDistance(const JointArray& q, const std::array<JointArray, N>& anchors) {
  double best = std::numeric_limits<double>::infinity();
  for (const JointArray& anchor : anchors) {
    best = std::min(best, JointDistance(q, anchor));
  }
  return best;
}

double NeedleSideScore(const std::array<double, 3>& tool) {
  const double desired_x = kRingPlaneX + 0.22;
  const double desired_y = 0.0;
  const double desired_z = kRingCenterZ;
  const double opening_y_margin = kRingHalfOpeningY - std::abs(tool[1]);
  const double opening_z_margin = kRingHalfOpeningZ - std::abs(tool[2] - kRingCenterZ);
  const double shelf_height_bonus = tool[2] > 0.12 && tool[2] < 0.62 ? 0.3 : 0.0;
  return 3.0 * opening_y_margin + 3.0 * opening_z_margin + shelf_height_bonus -
         1.2 * std::abs(tool[0] - desired_x) - 0.4 * std::abs(tool[1] - desired_y) -
         0.5 * std::abs(tool[2] - desired_z);
}

double ShelfSideScore(const std::array<double, 3>& tool) {
  const double desired_x = -0.68;
  const double desired_y = -0.23;
  const double desired_z = 0.20;
  const double shelf_reach_bonus = tool[0] < -0.60 && tool[2] > 0.10 && tool[2] < 0.40 ? 1.0 : 0.0;
  const double opening_y_margin = kRingHalfOpeningY - std::abs(tool[1]);
  const double opening_z_margin = kRingHalfOpeningZ - std::abs(tool[2] - kRingCenterZ);
  return shelf_reach_bonus + 1.5 * opening_y_margin + 1.0 * opening_z_margin -
         1.8 * std::abs(tool[0] - desired_x) - 0.9 * std::abs(tool[1] - desired_y) -
         0.8 * std::abs(tool[2] - desired_z);
}

bool IsNeedleSideGoal(const std::array<double, 3>& tool) {
  return tool[0] > kRingPlaneX + 0.08 && std::abs(tool[1]) < kRingHalfOpeningY * 0.85 &&
         std::abs(tool[2] - kRingCenterZ) < kRingHalfOpeningZ * 0.85 && tool[2] > 0.10 &&
         tool[2] < 0.70;
}

bool IsShelfSideGoal(const std::array<double, 3>& tool) {
  return tool[0] < kRingPlaneX - 0.12 && std::abs(tool[1]) < kRingHalfOpeningY * 0.95 &&
         tool[2] > 0.08 && tool[2] < 0.62;
}

std::vector<Candidate> SelectDiverse(std::vector<Candidate> candidates, int count) {
  std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
    return a.score > b.score;
  });

  std::vector<Candidate> selected;
  selected.reserve(count);
  double diversity = kMinJointDiversity;
  while (static_cast<int>(selected.size()) < count && diversity >= 0.10) {
    for (const Candidate& candidate : candidates) {
      if (static_cast<int>(selected.size()) >= count) {
        break;
      }
      const bool far_enough = std::all_of(selected.begin(), selected.end(), [&](const Candidate& s) {
        return JointDistance(candidate.q, s.q) >= diversity;
      });
      if (far_enough) {
        selected.push_back(candidate);
      }
    }
    diversity *= 0.75;
  }
  return selected;
}

void PrintCandidate(const Candidate& candidate, int index, std::string_view label) {
  std::cout << label << ' ' << index << " score=" << std::fixed << std::setprecision(3)
            << candidate.score << " tool=[" << candidate.tool[0] << ", " << candidate.tool[1]
            << ", " << candidate.tool[2] << "] q={";
  for (int i = 0; i < kDof; ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << std::setprecision(6) << candidate.q[i];
  }
  std::cout << "}\n";
}

void PrintCppInitializer(const std::vector<Candidate>& needle_side,
                         const std::vector<Candidate>& shelf_side) {
  std::cout << "\nC++ alternating loop-goal initializer:\n";
  std::cout << "std::vector<JointArray> BuildLoopGoals() {\n";
  std::cout << "  return {\n";
  for (int i = 0; i < kKeepPerSide; ++i) {
    const Candidate* pair[2] = {&needle_side[i], &shelf_side[i]};
    for (const Candidate* candidate : pair) {
      std::cout << "      {";
      for (int j = 0; j < kDof; ++j) {
        if (j > 0) {
          std::cout << ", ";
        }
        std::cout << std::fixed << std::setprecision(6) << candidate->q[j];
      }
      std::cout << "},\n";
    }
  }
  std::cout << "  };\n";
  std::cout << "}\n";
}

int ParseInt(std::string_view text, int fallback) {
  int value = fallback;
  const auto* begin = text.data();
  const auto* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec != std::errc{} || ptr != end) {
    return fallback;
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  int samples = kDefaultSamples;
  std::uint64_t seed = kDefaultSeed;
  std::optional<std::filesystem::path> scene_arg;

  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0] << " [--samples N] [--seed N] [scene.xml]\n";
      return 0;
    }
    if (arg == "--samples" && i + 1 < argc) {
      samples = ParseInt(argv[++i], samples);
      continue;
    }
    if (arg == "--seed" && i + 1 < argc) {
      seed = static_cast<std::uint64_t>(ParseInt(argv[++i], static_cast<int>(seed)));
      continue;
    }
    if (!scene_arg.has_value()) {
      scene_arg = std::filesystem::path(arg);
      continue;
    }
    std::cerr << "error: unexpected argument: " << arg << '\n';
    return 1;
  }

  try {
    const std::filesystem::path scene_path =
        scene_arg.value_or(std::filesystem::path(UR5_MUJOCO_OMPL_DEFAULT_SCENE));
    Ur5Scene scene(scene_path);

    std::array<std::uniform_real_distribution<double>, kDof> joint_distributions;
    std::array<std::pair<double, double>, kDof> joint_bounds;
    for (int i = 0; i < kDof; ++i) {
      const auto [lo, hi] = scene.JointBounds(i);
      joint_bounds[i] = {std::max(lo, -kPi), std::min(hi, kPi)};
      joint_distributions[i] =
          std::uniform_real_distribution<double>(joint_bounds[i].first, joint_bounds[i].second);
    }

    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> anchor_noise(0.0, kAnchorNoiseStddev);
    std::uniform_int_distribution<std::size_t> home_anchor(0, kHomeCandidates.size() - 1);
    std::uniform_int_distribution<std::size_t> goal_anchor(0, kGoalCandidates.size() - 1);
    std::vector<Candidate> needle_side;
    std::vector<Candidate> shelf_side;
    int valid = 0;
    for (int sample = 0; sample < samples; ++sample) {
      JointArray q{};
      const double proposal = unit(rng);
      const JointArray* anchor = nullptr;
      if (proposal < 0.45) {
        anchor = &kHomeCandidates[home_anchor(rng)];
      } else if (proposal < 0.90) {
        anchor = &kGoalCandidates[goal_anchor(rng)];
      }

      if (anchor != nullptr) {
        for (int joint = 0; joint < kDof; ++joint) {
          q[joint] = std::clamp((*anchor)[joint] + anchor_noise(rng),
                                joint_bounds[joint].first,
                                joint_bounds[joint].second);
        }
      } else {
        for (int joint = 0; joint < kDof; ++joint) {
          q[joint] = joint_distributions[joint](rng);
        }
      }
      if (!scene.IsStateValid(q)) {
        continue;
      }
      ++valid;
      const auto tool = scene.ToolPosition();
      if (IsNeedleSideGoal(tool)) {
        needle_side.push_back(
            {q, tool, NeedleSideScore(tool) - 0.8 * NearestAnchorDistance(q, kNeedleLoopAnchors)});
      } else if (IsShelfSideGoal(tool)) {
        shelf_side.push_back(
            {q, tool, ShelfSideScore(tool) - 0.8 * NearestAnchorDistance(q, kShelfLoopAnchors)});
      }
    }

    const std::vector<Candidate> selected_needle = SelectDiverse(needle_side, kKeepPerSide);
    const std::vector<Candidate> selected_shelf = SelectDiverse(shelf_side, kKeepPerSide);

    std::cout << "Scene: " << scene_path << '\n';
    std::cout << "Seed: " << seed << '\n';
    std::cout << "Samples: " << samples << '\n';
    std::cout << "Valid collision-clear states: " << valid << '\n';
    std::cout << "Needle-side candidates: " << needle_side.size() << '\n';
    std::cout << "Shelf-side candidates: " << shelf_side.size() << '\n';
    std::cout << "Selected needle-side goals: " << selected_needle.size() << '\n';
    std::cout << "Selected shelf-side goals: " << selected_shelf.size() << '\n';

    if (selected_needle.size() < kKeepPerSide || selected_shelf.size() < kKeepPerSide) {
      std::cerr << "error: not enough valid goals; increase --samples\n";
      return 1;
    }

    for (int i = 0; i < kKeepPerSide; ++i) {
      PrintCandidate(selected_needle[i], i + 1, "needle");
      PrintCandidate(selected_shelf[i], i + 1, "shelf ");
    }
    PrintCppInitializer(selected_needle, selected_shelf);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
