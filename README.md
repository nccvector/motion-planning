# motion-planning

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
- A C++23 executable, `motion_planning_clutter_plan`, that:
  - loads an official Menagerie UR5e scene wrapped with local shelf and tight
    red-ring window clutter,
  - checks home and goal validity,
  - plans a joint-space path with OMPL `RRTConnect`,
  - shortcut-simplifies the geometric path into hard knots,
  - tries to fit a natural cubic joint-space spline through those knots, giving
    C2 continuity at internal knots,
  - repairs failed splines by adding violating samples as extra knots while the
    3000 ms planning budget still has room,
  - falls back to a linear interpolation of the OMPL path if spline repair runs
    out of budget,
  - executes the sampled path with Menagerie position actuators,
  - writes `planned_path.csv` and `executed_trace.csv`.
- A C++23 executable, `motion_planning_path_replay`, that:
  - loads the same MuJoCo scene,
  - replays `planned_path.csv` or `executed_trace.csv`,
  - shows the tool path and current tool position in a native MuJoCo/GLFW window.
- A C++23 executable, `motion_planning_goal_loop_demo`, that:
  - cycles through 24 valid joint-space entries,
  - returns to one robust red-ring staging pose between 12 Monte Carlo-selected
    shelf-side gap goals cycling through three spaces between four shelf-entry
    poles,
  - plans each next segment on a separate planning scene/thread while the viewer
    stays responsive,
  - retries failed planning segments up to 5 times,
  - shows the current target posture as a translucent green robot ghost,
  - executes each accepted segment with the same PID controller without writing
    long-running CSV traces.
- A C++23 executable, `motion_planning_goal_monte_carlo`, that:
  - runs deterministic Monte Carlo exploration of valid states in the current
    scene,
  - ranks primary-ring approach-side goals and shelf-gap finish-line goals,
  - prints a C++ initializer for an alternating 24-goal loop list.
- A C++23 executable, `motion_planning_path_diagnose`, that:
  - checks robot collision geometry against the red ring along a saved CSV path,
  - samples between adjacent CSV rows to catch interpolation/grazing issues.

## Build

```bash
cd /Users/faizanali/Documents/robotics/projects/motion-planning
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Run

```bash
./build/motion_planning_clutter_plan
```

This plans the path, opens a live MuJoCo window for the PID execution preview,
and still writes `planned_path.csv` and `executed_trace.csv`. The PID controller
runs as a 60 Hz realtime control loop: each tick computes the desired point
along the time-parametrized path, applies one control update, advances MuJoCo
once with a `1/60 s` timestep, and sleeps any remaining wall time in realtime
mode.

Optional scene argument:

```bash
./build/motion_planning_clutter_plan /path/to/scene.xml
```

Non-interactive run without the live PID window:

```bash
./build/motion_planning_clutter_plan --no-live
```

The default live viewer is paced to MuJoCo simulation time. For a fast preview
that does not sleep to realtime:

```bash
./build/motion_planning_clutter_plan --live-fast
```

Endless goal-loop demo:

```bash
./build/motion_planning_goal_loop_demo
```

This opens the viewer immediately at the first loop goal, plans to the next
goal on a separate planning scene/thread, and then executes the result with the
PID controller on the visible scene. It does not write CSV files. The overlay
shows planning progress, retry attempt, selected path kind (`C2 spline smoothed
path` or linear fallback), planning time, OMPL solve attempts, spline repair
iterations, spline-fit success/fallback status, and 60 Hz PID execution
progress. The translucent green robot shows the current goal posture. If all 5
planning retries fail for a segment, the program stops and prints the exact
start/goal joint vectors for reproducing that pair in a smaller experiment.

Goal-list checks:

```bash
./build/motion_planning_goal_loop_demo --check-goals
./build/motion_planning_goal_loop_demo --check-transitions
```

Regenerate Monte Carlo candidates:

```bash
./build/motion_planning_goal_monte_carlo
```

The current scene uses a tightened primary red ring plus four vertical red
shelf-entry poles. Those poles create three empty spaces. The logical finish
line is at the pole plane (`x = -0.72 m`), and the generated shelf poses place
the end effector at least 1 cm deeper toward the shelf. The loop goal list is
checked to keep 12 approach-side entries and 12 shelf-side goals, split as
4 goals through each pole gap. The approach-side entries intentionally reuse one
robust staging pose so the long-running loop keeps replanning reliably.

The planner rejects actual obstacle contacts. OMPL state validity and final path
acceptance use a 1.5 cm clearance margin between robot collision geometry and
shelf/ring obstacles. The planner has a 3000 ms wall-time budget for OMPL,
shortcutting, spline fitting, spline repair, and linear fallback selection.

OMPL approximate solutions are rejected because they may be partial paths that
do not actually reach the shelf-side goal. The planner retries bounded OMPL
attempts inside the same 3000 ms budget and only sends exact start-to-goal paths
into spline fitting or linear fallback.

The planned path is the hard gate. The MuJoCo position-servo execution preview
still reports obstacle contacts and clearance-margin dips, but only actual
execution contacts fail the run. Clearance dips in `executed_trace.csv` are
controller-following diagnostics, not evidence that the planned path itself was
invalid. Execution trace clearance diagnostics are sampled rather than computed
at every internal physics step to keep the realtime control loop lightweight.

## Visualize

The planner executable already shows live PID execution after planning. In that
live window:

- `C`: toggle robot mesh view vs robot collision-geometry view
- `Esc`: close the live viewer while the CSV write continues

For later replay from CSV, first generate a path:

```bash
./build/motion_planning_clutter_plan --no-live
```

Then replay it:

```bash
./build/motion_planning_path_replay
```

Optional explicit scene and CSV arguments:

```bash
./build/motion_planning_path_replay build/scenes/ur5e_clutter.xml planned_path.csv
./build/motion_planning_path_replay build/scenes/ur5e_clutter.xml executed_trace.csv
```

Non-interactive loader check:

```bash
./build/motion_planning_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv
```

Collision diagnostic:

```bash
./build/motion_planning_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128
./build/motion_planning_path_diagnose build/scenes/ur5e_clutter.xml executed_trace.csv 4
```

Viewer controls:

- `Space`: play/pause
- `Left` / `Right`: single-step while paused
- `Up` / `Down`: playback speed
- `C`: toggle robot mesh view vs robot collision-geometry view
- `R`: restart
- `Esc`: close
- Mouse drag/scroll: rotate, pan, and zoom camera

The live PID viewer and CSV replayer share the same camera defaults and mouse
drag/scroll controls.

Successful validation includes:

```text
Robot model: MuJoCo Menagerie UR5e
Control mode: position servo
Selected goal tool position: [-0.704, -0.232, 0.153]
C2 spline max knot position error: <near zero>
C2 spline max C1 discontinuity: <near zero>
C2 spline max C2 discontinuity: <near zero>
Planner used C2 spline: true|false
Planner fallback: raw OMPL linear|shortcut linear
OMPL solve attempts: <small integer>
OMPL approximate attempts rejected: <count>
Planning wall time: < 3000 ms
Planned path obstacle-contact states: 0
Planning obstacle clearance: 0.015 m
Planned path clearance-violation states: 0
Ring-opening path samples: <nonzero>
PID mean waypoint tracking error: <small>
PID max waypoint tracking error: <small>
PID obstacle-contact steps: 0
PID timing clearance/diagnostics: <diagnostic cost>
PID trajectory duration: <scheduled motion duration>
PID final hold duration: <final target hold>
PID simulated time: <execution duration in MuJoCo seconds>
PID clearance-violation trace rows: <sampled diagnostic count>
```

## Scope

This is still a learning SIL setup, not a vendor-certified UR controller. The
robot model is the official MuJoCo Menagerie UR5e MJCF, while the shelf/ring
clutter is local simple box geometry intended for planning and controller
practice. The current scene uses a tighter ring, a farther shelf-side target,
and extra shelf/ring-prefixed clutter geoms so the existing planner collision
filter treats them as obstacles.
