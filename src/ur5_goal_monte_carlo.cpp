#include "ur5_clutter_core.hpp"

#include <charconv>
#include <limits>
#include <random>

namespace {

constexpr std::uint64_t kDefaultSeed = 20260627;
constexpr int kDefaultSamples = 500000;
constexpr int kKeepPerSide = 12;
constexpr int kKeepPerShelfWindow = kKeepPerSide / static_cast<int>(kShelfWindowCount);
constexpr double kMinJointDiversity = 0.55;
constexpr double kAnchorNoiseStddev = 0.22;

struct Candidate {
  JointArray q{};
  std::array<double, 3> tool{};
  double score = 0.0;
  std::size_t window_index = 0;
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

double WindowAlignmentScore(const std::array<double, 3>& tool, const WindowSpec& window) {
  const double opening_y_margin = window.half_opening_y - std::abs(tool[1] - window.center_y);
  const double opening_z_margin = window.half_opening_z - std::abs(tool[2] - window.center_z);
  return 3.0 * opening_y_margin + 3.0 * opening_z_margin;
}

double NeedleSideScore(const std::array<double, 3>& tool) {
  const WindowSpec& window = kWindowSpecs[0];
  const double desired_x = window.plane_x + 0.16;
  const double shelf_height_bonus = tool[2] > 0.12 && tool[2] < 0.62 ? 0.3 : 0.0;
  return WindowAlignmentScore(tool, window) + shelf_height_bonus -
         1.2 * std::abs(tool[0] - desired_x);
}

double ShelfSideScore(const std::array<double, 3>& tool, const WindowSpec& window) {
  const double desired_x = kShelfGoalTargetX;
  const double shelf_reach_bonus =
      tool[0] < kShelfGoalTargetX ? 1.4 : 0.0;
  return shelf_reach_bonus + WindowAlignmentScore(tool, window) -
         1.0 * std::abs(tool[0] - desired_x);
}

bool IsNeedleSideGoal(const std::array<double, 3>& tool) {
  return tool[0] > kRingPlaneX + 0.04 && ToolAlignedWithWindow(tool, kWindowSpecs[0], 1.10) &&
         tool[2] > 0.10 && tool[2] < 0.70;
}

std::optional<std::size_t> ShelfWindowIndexForTool(const std::array<double, 3>& tool) {
  std::optional<std::size_t> best_index;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < kWindowSpecs.size(); ++i) {
    const WindowSpec& window = kWindowSpecs[i];
    if (std::abs(window.plane_x - kShelfWindowPlaneX) > 1e-9) {
      continue;
    }
    if (!ToolAlignedWithWindow(tool, window)) {
      continue;
    }
    const double distance = std::abs(tool[1] - window.center_y) +
                            std::abs(tool[2] - window.center_z);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = i;
    }
  }
  return best_index;
}

bool IsShelfSideGoal(const std::array<double, 3>& tool) {
  return tool[0] < kShelfGoalTargetX && tool[2] > 0.08 &&
         tool[2] < 0.66 &&
         ShelfWindowIndexForTool(tool).has_value();
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
            << ", " << candidate.tool[2] << "] window="
            << kWindowSpecs[candidate.window_index].name << " q={";
  for (int i = 0; i < kDof; ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << std::setprecision(6) << candidate.q[i];
  }
  std::cout << "}\n";
}

void PrintCppInitializer(const std::vector<Candidate>& needle_side,
                         const std::array<std::vector<Candidate>, kShelfWindowCount>& shelf_windows) {
  std::cout << "\nC++ alternating loop-goal initializer:\n";
  std::cout << "std::vector<JointArray> BuildLoopGoals() {\n";
  std::cout << "  // Alternating Monte Carlo goals from `ur5_goal_monte_carlo`: approach-side\n";
  std::cout << "  // states near the primary red-ring opening, then shelf-side states that\n";
  std::cout << "  // cross the logical finish lines behind the three pole gaps.\n";
  std::cout << "  return {\n";
  for (int i = 0; i < kKeepPerSide; ++i) {
    const std::size_t shelf_bucket = static_cast<std::size_t>(i) % kShelfWindowCount;
    const std::vector<Candidate>& shelf_side = shelf_windows[shelf_bucket];
    const Candidate* pair[2] = {&needle_side[i], &shelf_side[i / kShelfWindowCount]};
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
    std::array<std::vector<Candidate>, kShelfWindowCount> shelf_windows;
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
            {q,
             tool,
             NeedleSideScore(tool) - 0.8 * NearestAnchorDistance(q, kNeedleLoopAnchors),
             0});
      } else if (IsShelfSideGoal(tool)) {
        const std::optional<std::size_t> shelf_window_index = ShelfWindowIndexForTool(tool);
        if (!shelf_window_index.has_value()) {
          continue;
        }
        const std::size_t shelf_bucket = *shelf_window_index - kShelfWindowStartIndex;
        shelf_windows[shelf_bucket].push_back(
            {q,
             tool,
             ShelfSideScore(tool, kWindowSpecs[*shelf_window_index]) -
                 0.8 * NearestAnchorDistance(q, kShelfLoopAnchors),
             *shelf_window_index});
      }
    }

    const std::vector<Candidate> selected_needle = SelectDiverse(needle_side, kKeepPerSide);
    std::array<std::vector<Candidate>, kShelfWindowCount> selected_shelf_windows;
    for (std::size_t i = 0; i < kShelfWindowCount; ++i) {
      selected_shelf_windows[i] = SelectDiverse(shelf_windows[i], kKeepPerShelfWindow);
    }

    std::cout << "Scene: " << scene_path << '\n';
    std::cout << "Seed: " << seed << '\n';
    std::cout << "Samples: " << samples << '\n';
    std::cout << "Valid collision-clear states: " << valid << '\n';
    std::cout << "Needle-side candidates: " << needle_side.size() << '\n';
    for (std::size_t i = 0; i < kShelfWindowCount; ++i) {
      std::cout << "Shelf " << kWindowSpecs[kShelfWindowStartIndex + i].name
                << " candidates: " << shelf_windows[i].size() << '\n';
    }
    std::cout << "Selected needle-side goals: " << selected_needle.size() << '\n';
    for (std::size_t i = 0; i < kShelfWindowCount; ++i) {
      std::cout << "Selected " << kWindowSpecs[kShelfWindowStartIndex + i].name
                << " goals: " << selected_shelf_windows[i].size() << '\n';
    }

    bool enough_shelf_goals = true;
    for (const auto& selected_window : selected_shelf_windows) {
      enough_shelf_goals = enough_shelf_goals &&
                           static_cast<int>(selected_window.size()) >= kKeepPerShelfWindow;
    }
    if (selected_needle.size() < kKeepPerSide || !enough_shelf_goals) {
      std::cerr << "error: not enough valid goals; increase --samples\n";
      return 1;
    }

    for (int i = 0; i < kKeepPerSide; ++i) {
      PrintCandidate(selected_needle[i], i + 1, "needle");
      const std::size_t shelf_bucket = static_cast<std::size_t>(i) % kShelfWindowCount;
      const std::vector<Candidate>& shelf_side = selected_shelf_windows[shelf_bucket];
      PrintCandidate(shelf_side[i / kShelfWindowCount],
                     i + 1,
                     kWindowSpecs[kShelfWindowStartIndex + shelf_bucket].name);
    }
    PrintCppInitializer(selected_needle, selected_shelf_windows);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }

  return 0;
}
