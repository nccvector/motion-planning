# Progress

## 2026-06-27

- Created local `ur5-mujoco-ompl` project scaffold outside the ROS Docker workspace.
- Chosen C++ standard: C++23 for project code.
- Pinned dependencies for CMake FetchContent:
  - MuJoCo `3.10.0`
  - OMPL `2.0.1`
  - Eigen `5.0.1`
  - Boost `1.91.0-1` CMake release, with `serialization`, `program_options`,
    `graph`, and `math`
- Added first cluttered UR5-like MuJoCo scene and C++ planning/control executable.
- Configured OMPL as an embedded subproject with package/export rules disabled
  for this local executable build.
- Added Boost modular include directories explicitly so OMPL can see Boost.Graph
  and Boost.Math headers from the fetched Boost source.
- Added MuJoCo Menagerie as a CMake `FetchContent` dependency and switched the
  default scene to the official `universal_robots_ur5e` model.
- Generate `build/scenes/ur5e_absolute_assets.xml` at configure time so MuJoCo
  resolves Menagerie mesh assets correctly from the fetched source tree.
- Generate `build/scenes/ur5e_clutter.xml` from `scenes/ur5e_clutter.xml.in`
  with local shelf and ring obstacle geometry.
- Updated the C++ runner to:
  - detect official Menagerie UR5e names and the earlier simplified names,
  - use model joint limits for OMPL bounds,
  - count shelf/ring obstacle contacts only,
  - validate that the planned path has zero obstacle-contact states,
  - execute the path with Menagerie position actuators.
- Verified end-to-end with:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  - `cmake --build build -j`
  - `./build/ur5_clutter_plan`
- Latest successful baseline run before the harder scene:
  - Robot model: MuJoCo Menagerie UR5e
  - Control mode: position servo
  - OMPL path states after interpolation: 240
  - Planned path obstacle-contact states: 0
  - Ring-plane path samples: 28
  - Ring-opening path samples: 28
  - PID final max joint error: 0.019 rad
  - PID obstacle-contact steps: 0
  - Wrote `planned_path.csv` and `executed_trace.csv`
- Added `ur5_path_replay`, a native MuJoCo/GLFW viewer that replays
  `planned_path.csv` or `executed_trace.csv` against the same UR5e clutter scene.
  It also has a `--check` mode for non-interactive loader validation.
- Verified the replay target with:
  - `cmake --build build --target ur5_path_replay -j`
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv`
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`
- Tightened the clutter scene without changing planner or replay code:
  - ring aperture reduced from bars at `y=+/-0.62`, `z=+/-0.44` to
    `y=+/-0.50`, `z=+/-0.34`,
  - shelf moved farther in `-x`,
  - added blue shelf/ring-prefixed clutter around approach, ring exit, and
    shelf entry,
  - added a small shelf-face blocker to reject shallower goal candidates so the
    selected goal is the farther built-in candidate.
- Verified the harder scene with the existing executables:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`
  - `./build/ur5_clutter_plan`
  - selected goal tool position: `[-0.704, -0.232, 0.153]`
  - OMPL path states after interpolation: 240
  - Planned path obstacle-contact states: 0
  - Ring-plane path samples: 31
  - Ring-opening path samples: 31
  - PID final max joint error: 0.020 rad
  - PID obstacle-contact steps: 0
  - Final tool position: `[-0.702, -0.231, 0.136]`
- Verified replay loading for the harder scene:
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv`
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`
- Updated `ur5_path_replay` to toggle between robot visual mesh view and robot
  collision-geometry view with `C`, leaving the scene obstacles visible.
- Root-caused the apparent red-ring collision:
  - the old planner only rejected actual MuJoCo contacts reported in `data->ncon`,
  - dense diagnostic sampling found no actual robot-ring contacts, but the old
    path grazed the red ring with effectively zero clearance between saved rows,
  - this was a clearance/padding bug, not a failure to parse red ring geoms.
- Added `ur5_path_diagnose` to report robot-collision-vs-red-ring contacts and
  closest distance, including samples interpolated between adjacent CSV rows.
- Updated `ur5_clutter_plan` to use two clearance thresholds:
  - OMPL state validity and home/goal selection require 2.5 cm clearance,
  - post-plan and PID validation enforce a 1 cm hard clearance floor.
- Regenerated `planned_path.csv` and `executed_trace.csv` with the split-margin checker:
  - `./build/ur5_clutter_plan`
  - OMPL path states after interpolation: 240
  - Required obstacle clearance: `0.010 m`
  - Planning obstacle clearance: `0.025 m`
  - Planned path obstacle-contact states: 0
  - Planned path clearance-violation states: 0
  - Ring-plane path samples: 38
  - Ring-opening path samples: 38
  - PID final max joint error: `0.017 rad`
  - PID obstacle-contact steps: 0
  - PID clearance-violation steps: 0
- Reproduced the stochastic failure class after the 1 cm-only checker:
  - a sampled OMPL path could pass while the position-servo trace entered the
    1 cm clearance band,
  - the same narrow margin also allowed occasional post-interpolation path
    validation failures.
- Verified 5 consecutive `./build/ur5_clutter_plan` runs after adding the
  2.5 cm planning cushion; all had 0 planned clearance violations and 0 PID
  clearance-violation steps.
- Verified dense red-ring diagnostics after padding:
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative distances, closest red-ring distance `0.025384 m`.
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml executed_trace.csv 4`
    found 0 contacts, 0 negative distances, closest red-ring distance `0.021725 m`.
  - CSV summaries showed 0 clearance-violation rows for both planned and executed traces.
- Added OMPL path simplification before interpolation:
  - `setup.simplifySolution(1.0)` runs after RRTConnect solves,
  - the planner reports raw, simplified, and interpolated path state counts,
  - validation still runs after simplification/interpolation.
- Added PID waypoint-tracking diagnostics:
  - final max joint error,
  - mean waypoint tracking error,
  - max waypoint tracking error.
- Fixed `ur5_path_diagnose` closest-distance reporting for cases where MuJoCo
  returned `0` while witness points were nonzero-distance apart.
- Verified 3 consecutive `./build/ur5_clutter_plan` runs after simplification:
  - simplified paths had 6-7 waypoints before interpolation,
  - all had 0 planned clearance violations and 0 PID clearance-violation steps,
  - observed max waypoint tracking error range was `0.020-0.047 rad`.
- Latest dense red-ring diagnostics after simplification:
  - planned path: 0 contacts, 0 negative distances, closest red-ring distance
    `0.026301 m`,
  - executed trace: 0 contacts, 0 negative distances, closest red-ring distance
    `0.028734 m`.

## Next

- Open the interactive replay window with `./build/ur5_path_replay` and press
  `C` to switch between robot mesh view and collision-geometry view.
- Use `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
  when a visual pass looks suspicious; trust this over eyeballing perspective.
