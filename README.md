# ur5-mujoco-ompl

Local C++ lab for planning and executing UR5e manipulator motion in a cluttered
MuJoCo scene with OMPL.

This project is intentionally separate from `projects/ros2-learning`.

## What It Builds

- MuJoCo from CMake `FetchContent`.
- MuJoCo Menagerie from CMake `FetchContent`, pinned to a known commit, for the
  official `universal_robots_ur5e` MJCF model and mesh assets.
- OMPL from CMake `FetchContent`.
- Eigen and the required Boost components from CMake `FetchContent`, because
  OMPL needs them.
- A C++23 executable, `ur5_clutter_plan`, that:
  - loads an official Menagerie UR5e scene wrapped with local shelf/ring clutter,
  - checks home and goal validity,
  - plans a joint-space path with OMPL `RRTConnect`,
  - simplifies/shortcuts the geometric path,
  - tries OMPL B-spline smoothing and keeps it only if the 2.5 cm planning
    clearance cushion is preserved,
  - executes the interpolated path with Menagerie position actuators,
  - writes `planned_path.csv` and `executed_trace.csv`.
- A C++23 executable, `ur5_path_replay`, that:
  - loads the same MuJoCo scene,
  - replays `planned_path.csv` or `executed_trace.csv`,
  - shows the tool path and current tool position in a native MuJoCo/GLFW window.
- A C++23 executable, `ur5_path_diagnose`, that:
  - checks robot collision geometry against the red ring along a saved CSV path,
  - samples between adjacent CSV rows to catch interpolation/grazing issues.

## Build

```bash
cd /Users/faizanali/Documents/robotics/projects/ur5-mujoco-ompl
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
./build/ur5_clutter_plan
```

Optional scene argument:

```bash
./build/ur5_clutter_plan /path/to/scene.xml
```

The planner rejects actual obstacle contacts and uses a 2.5 cm planning
clearance between robot collision geometry and shelf/ring obstacles. Final
planned-path and executed-trace validation enforce a 1 cm hard clearance floor.

## Visualize

First generate a path:

```bash
./build/ur5_clutter_plan
```

Then replay it:

```bash
./build/ur5_path_replay
```

Optional explicit scene and CSV arguments:

```bash
./build/ur5_path_replay build/scenes/ur5e_clutter.xml planned_path.csv
./build/ur5_path_replay build/scenes/ur5e_clutter.xml executed_trace.csv
```

Non-interactive loader check:

```bash
./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv
```

Collision diagnostic:

```bash
./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128
./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml executed_trace.csv 4
```

Viewer controls:

- `Space`: play/pause
- `Left` / `Right`: single-step while paused
- `Up` / `Down`: playback speed
- `C`: toggle robot mesh view vs robot collision-geometry view
- `R`: restart
- `Esc`: close
- Mouse drag/scroll: rotate, pan, and zoom camera

Successful validation includes:

```text
Robot model: MuJoCo Menagerie UR5e
Control mode: position servo
Selected goal tool position: [-0.704, -0.232, 0.153]
OMPL using B-spline-smoothed path: true
Planned path obstacle-contact states: 0
Planning obstacle clearance: 0.025 m
Planned path clearance-violation states: 0
Ring-opening path samples: <nonzero>
PID mean waypoint tracking error: <small>
PID max waypoint tracking error: <small>
PID obstacle-contact steps: 0
PID clearance-violation steps: 0
```

## Scope

This is still a learning SIL setup, not a vendor-certified UR controller. The
robot model is the official MuJoCo Menagerie UR5e MJCF, while the shelf/ring
clutter is local simple box geometry intended for planning and controller
practice. The current scene uses a tighter ring, a farther shelf-side target,
and extra shelf/ring-prefixed clutter geoms so the existing planner collision
filter treats them as obstacles.
