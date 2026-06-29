# Progress

## 2026-06-29

- Added a second endless goal loop executable for `InformedRRTstar`:
  `ur5_goal_loop_informed_rrtstar_demo`.
- Refactored the endless goal loop implementation into
  `src/ur5_goal_loop_demo.hpp`, leaving `ur5_goal_loop_demo.cpp` and
  `ur5_goal_loop_informed_rrtstar_demo.cpp` as small planner-specific entry
  points.
- Added `PlannerKind` selection to `PlanPath`, keeping `RRTConnect` as the
  default for existing callers and using `InformedRRTstar` with the same range,
  `0.05` goal bias, and delayed collision checking for the new loop demo.
- Added a shared `0.25` exact-goal state-sampling bias in `PlanPath` so both
  loop demos nudge OMPL samples toward the active segment goal; the
  `InformedRRTstar` demo also uses OMPL's native `setGoalBias(0.25)`.
- Reverted the OMPL planner/tree reuse experiment after it caused segfaults;
  loop planning is back to fresh per-call planner construction and
  `planner->clear()` between solve slices.
- Verified:
  - `cmake --build projects/ur5-mujoco-ompl/build --target ur5_goal_loop_demo -j`
  - `cmake --build projects/ur5-mujoco-ompl/build --target ur5_goal_loop_informed_rrtstar_demo -j`
  - `cmake --build projects/ur5-mujoco-ompl/build --target ur5_goal_loop_demo ur5_goal_loop_informed_rrtstar_demo -j`
  - `./projects/ur5-mujoco-ompl/build/ur5_goal_loop_demo --check-goals`
  - `./projects/ur5-mujoco-ompl/build/ur5_goal_loop_informed_rrtstar_demo --check-goals`

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
- Tried explicit OMPL B-spline smoothing after shortcut simplification:
  - initial unconditional B-spline smoothing produced a planned path that passed
    1 cm validation but caused execution obstacle contacts, showing it consumed
    too much controller safety cushion,
  - updated `ur5_clutter_plan` to smooth a copy, check the smoothed copy against
    the 2.5 cm planning clearance, and fall back to the shortcut-simplified path
    if the B-spline path loses that cushion,
  - planner now reports B-spline state count, B-spline planning-clearance
    violation count, and whether the B-spline path was used.
- Verified 6 successful `./build/ur5_clutter_plan` runs after guarded B-spline:
  - all used B-spline in the observed runs,
  - all had 0 planned clearance violations and 0 PID clearance-violation steps,
  - observed max waypoint tracking error range was `0.020-0.040 rad`.
- Latest dense red-ring diagnostics after guarded B-spline:
  - planned path: 0 contacts, 0 negative distances, closest red-ring distance
    `0.024841 m`,
  - executed trace: 0 contacts, 0 negative distances, closest red-ring distance
    `0.020901 m`.
- Replaced final B-spline smoothing with an interpolating natural cubic
  joint-space spline:
  - RRTConnect still finds the geometric path,
  - OMPL shortcut simplification produces hard knots,
  - the natural cubic spline passes through those knots exactly,
  - the spline is C2 continuous at internal knots by construction,
  - the sampled spline must preserve the 2.5 cm planning clearance or the
    planner rejects it and replans.
- Added C2 verification output:
  - max knot position error,
  - max C1 discontinuity,
  - max C2 discontinuity.
- Verified 5 end-to-end `./build/ur5_clutter_plan` runs with C2 spline output:
  - all accepted final paths were sampled C2 natural cubic splines,
  - rejected candidate splines were replanned when they lost planning clearance,
  - accepted C2 residuals were around machine precision (`~1e-15`),
  - all accepted runs had 0 planned clearance violations and 0 PID
    clearance-violation steps.
- Latest accepted C2 run:
  - hard knots: 12,
  - samples: 240,
  - max knot position error: `1.110e-16 rad`,
  - max C1 discontinuity: `7.216e-16 rad/path-unit`,
  - max C2 discontinuity: `5.329e-15 rad/path-unit^2`,
  - PID obstacle-contact steps: 0,
  - PID clearance-violation steps: 0.
- Latest C2 dense diagnostics:
  - planned path: 0 contacts, 0 negative red-ring distances, closest red-ring
    distance `0.057827 m`,
  - executed trace: 0 contacts, 0 negative red-ring distances, closest red-ring
    distance `0.063210 m`,
  - CSV summaries showed 0 contact rows and 0 clearance-violation rows for both
    planned and executed traces.
- Reworked the planner into a single 500 ms budgeted pipeline instead of
  replanning when smoothing fails:
  - OMPL `RRTConnect` gets most of the budget and may return exact or approximate
    solutions,
  - shortcut simplification is treated as optional; failed shortcut repair falls
    back to the raw OMPL path,
  - natural cubic C2 fitting is attempted on the simplified knots,
  - failed spline samples are added back as extra hard knots while enough time
    and knot budget remain,
  - if spline fitting cannot satisfy clearance inside the budget, the planner
    falls back to a linear interpolation of the OMPL path, preferring the raw
    OMPL path over shortcut-linear fallback.
- Current timing and margin choices:
  - planning budget: `0.500 s`,
  - OMPL obstacle clearance: `0.015 m`,
  - final planned-path clearance: `0.015 m`,
  - hard clearance floor: `0.010 m`,
  - sampled output path states: `480`.
- Added execution command substeps so the MuJoCo position-servo preview receives
  smaller setpoint jumps.
- Changed execution clearance-margin dips from a hard failure into diagnostics:
  - actual execution obstacle contacts still fail,
  - planned-path contact/clearance validation remains the hard gate,
  - PID clearance-violation steps now indicate controller-following risk rather
    than invalidating the geometric plan.
- Latest accepted run after the 500 ms pipeline:
  - OMPL exact solution: `true`,
  - planner used C2 spline: `true`,
  - spline repair iterations: `0`,
  - planning wall time: `341.691 ms`,
  - planned path states: `480`,
  - final planned path planning-clearance violation states: `0`,
  - final planned path hard-clearance violation states: `0`,
  - PID obstacle-contact steps: `0`,
  - PID clearance-violation steps: `0`.
- Latest dense diagnostics after the 500 ms pipeline:
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, closest red-ring distance
    `0.015504 m`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv`
    loaded 480 frames successfully,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`
    loaded the generated 459657-frame execution trace successfully.
- Changed the interactive replay default speed from `0.45x` to realtime `1.0x`.
- Root-caused the intermittent
  `No fallback linear path preserved ring passage and hard clearance` failure:
  - the failing run was an OMPL `RRTConnect` approximate solution at the edge of
    the 500 ms budget,
  - approximate OMPL solutions can be partial paths that do not reach the
    shelf-side goal,
  - sending that partial path into spline/fallback validation made the fallback
    check fail with a misleading ring/hard-clearance message.
- Updated `ur5_clutter_plan` so approximate OMPL solutions are rejected before
  smoothing/fallback:
  - OMPL solves in bounded attempts inside the same 500 ms budget,
  - approximate attempts clear planner state and retry while budget remains,
  - only exact start-to-goal solutions enter shortcutting, spline repair, or
    linear fallback,
  - the fallback exception now reports raw/shortcut ring and hard-clearance
    details if it ever occurs on an exact solution.
- Added live PID execution viewing directly to `ur5_clutter_plan`:
  - default `./build/ur5_clutter_plan` opens a MuJoCo window after the final path
    is planned,
  - the window displays the actual PID-driven `mjData` state, not a CSV replay,
  - `C` toggles mesh/collision view and `Esc` closes the live viewer,
  - `--no-live` keeps batch/debug runs non-interactive while still writing
    `planned_path.csv` and `executed_trace.csv`.
- Verified with `cmake --build build --target ur5_clutter_plan`.
- Verified 10 consecutive non-interactive runs with
  `./build/ur5_clutter_plan --no-live`:
  - run 1 reproduced the old approximate-solution condition, rejected it, retried,
    and succeeded on attempt 2,
  - all 10 runs exited successfully,
  - no run produced the old fallback exception,
  - all final planned paths had 0 hard-clearance violations and nonzero
    ring-opening samples.
- Latest final smoke after the live-viewer patch:
  - `./build/ur5_clutter_plan --no-live` exited successfully,
  - planning wall time: `103.737 ms`,
  - planner fallback: `raw OMPL linear`,
  - final planned path hard-clearance violation states: `0`,
  - PID obstacle-contact steps: `0`,
  - PID clearance-violation steps: `0`.
- Latest dense planned-path diagnostic:
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, closest red-ring distance
    `0.019222 m`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml planned_path.csv`
    loaded 480 frames successfully.
- Investigated the long delay after the live PID window launches:
  - a headless timing run showed the post-plan PID preview took `15456.3 ms`,
  - it simulated `919.792 s` of MuJoCo time at a `0.002 s` timestep,
  - it executed `459896` internal PID physics steps and hit the per-setpoint
    settle cap `1916` times,
  - the largest CPU cost was per-step clearance diagnostics:
    `12754.9 ms` of the `15456.3 ms` total,
  - live realtime pacing would intentionally stretch that same simulation to
    roughly 15 minutes, which explains the apparent hang after the GUI appears.
- Updated live PID execution behavior:
  - root correction: the PID controller should run as a fixed-rate realtime
    loop, but the old executor was a settle-per-waypoint loop,
  - replaced the nested "hold each micro-target until settled" executor with a
    time-parametrized trajectory follower,
  - every MuJoCo timestep now computes the desired path state for that time,
    applies one PID/position-servo update, and advances simulation once,
  - default live viewing is paced to MuJoCo simulation time again because the
    simulated trajectory is now seconds long, not hundreds of seconds,
  - `--live-fast` opts into unpaced fast preview,
  - `executed_trace.csv` samples execution diagnostics every 16 internal steps,
    while obstacle contacts are still checked every internal step.
- Verified the post-plan timing after realtime controller correction:
  - `./build/ur5_clutter_plan --no-live` exited successfully,
  - planning wall time: `258.053 ms`,
  - PID trajectory duration: `5.455 s`,
  - final hold duration: `1.000 s`,
  - simulated time: `6.458 s`,
  - PID execution steps: `3229`,
  - PID timing total was `23.479 ms` headless,
  - `executed_trace.csv` contained `203` replayable frames,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`
    loaded the generated trace successfully,
  - dense planned-path diagnostic found 0 contacts, 0 negative robot-ring
    distances, and closest red-ring distance `0.019226 m`.
- Fixed live-viewer camera interaction:
  - root cause: `ur5_path_replay` installed MuJoCo mouse/scroll camera callbacks,
    but the live PID viewer only polled keyboard keys,
  - added `src/ur5_visualization.hpp` for shared camera defaults, mesh/collision
    group toggling, tool-path markers, and mouse drag/scroll camera callbacks,
  - both `ur5_clutter_plan` and `ur5_path_replay` now use the same viewer helper.
- Added an endless goal-loop demo:
  - split the reusable UR5 scene/planner/PID helpers into
    `src/ur5_clutter_core.hpp`,
  - kept `ur5_clutter_plan.cpp` as the one-shot CLI entrypoint,
  - added `src/ur5_goal_loop_demo.cpp` and the `ur5_goal_loop_demo` target,
  - the loop demo starts at the first joint-space goal, opens the viewer
    immediately, plans the next segment on a separate planning scene/thread, and
    executes the accepted path on the visible scene,
  - each segment retries planning up to 4 times and stops with exact start/goal
    joint vectors if retries are exhausted,
  - the loop demo does not write CSV traces for long-running sessions,
  - the overlay reports planning progress, retry attempt, selected path kind,
    planning time, OMPL solve attempts, spline repair iterations, and PID
    execution progress.
- Verified after the loop-demo split:
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo`,
  - `./build/ur5_goal_loop_demo --help`,
  - `./build/ur5_clutter_plan --no-live`.
- Pinned execution to an explicit 60 Hz control/simulation loop:
  - both `ur5_clutter_plan` and `ur5_goal_loop_demo` set MuJoCo execution
    timestep to `1/60 s`,
  - each control tick computes the desired path state, applies one PID/servo
    update, advances MuJoCo once, and sleeps the remaining wall time in
    realtime mode,
  - `--live-fast` still skips the sleep for quick preview/debug runs.
- Verified after the 60 Hz control/simulation change:
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo`,
  - `./build/ur5_clutter_plan --no-live`,
  - `MuJoCo timestep: 0.0166667 s`,
  - PID execution steps: `365`,
  - PID final max joint error: `0.0207284 rad`,
  - PID obstacle-contact steps: `0`,
  - PID clearance-violation trace rows: `0`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, and closest red-ring
    distance `0.020968 m`,
  - `./build/ur5_goal_loop_demo --help`.
- Added Monte Carlo loop-goal search and a larger verified loop:
  - added `src/ur5_goal_monte_carlo.cpp` and the `ur5_goal_monte_carlo` target,
  - the search samples valid states in the current MuJoCo scene, ranks
    red-ring approach-side goals and shelf-side goals separately, and prints a
    C++ initializer for an alternating 20-goal loop list,
  - the final loop list contains 20 goals: 10 needle-side and 10 shelf-side,
    alternating,
  - all 10 shelf-side goals have tool positions beyond the ring on the shelf
    side, with representative x positions from about `-0.61 m` to `-0.69 m`,
  - added `--check-goals` and `--check-transitions` to `ur5_goal_loop_demo`,
  - `--check-transitions` verified all 20 loop edges, and every checked edge
    planned as `C2 spline smoothed path`.
- Updated loop visualization/status:
  - added a translucent green target robot ghost for the goal currently being
    planned/executed,
  - execution status now explicitly shows `Spline fit: success` or
    `Spline fit: failed; using linear fallback`,
  - max loop planning attempts increased from 4 to 5.
- Changed the shared realtime execution loop from 60 Hz to 90 Hz:
  - execution timestep is now `1/90 s`,
  - each tick still computes the current trajectory target, applies one
    PID/servo update, advances MuJoCo once, and sleeps remaining wall time in
    realtime mode.
- Verified after the Monte Carlo/loop/90 Hz change:
  - `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release`,
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo ur5_goal_monte_carlo`,
  - `./build/ur5_goal_monte_carlo`,
  - `./build/ur5_goal_loop_demo --check-goals`,
  - `./build/ur5_goal_loop_demo --check-transitions`,
  - `./build/ur5_clutter_plan --no-live`,
  - `MuJoCo timestep: 0.0111111 s`,
  - PID execution steps: `530`,
  - PID obstacle-contact steps: `0`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, and closest red-ring
    distance `0.025195 m`.
- Tightened the red-ring scene and regenerated/validated the loop goals:
  - reduced the primary ring opening to about half its previous width/height,
  - added two copied red-ring shelf windows at the low/high shelf approach
    locations,
  - promoted the window geometry into shared `WindowSpec` data so planning
    validation, loop checks, and Monte Carlo candidate scoring use the same
    window definitions,
  - replaced stale one-shot home/goal candidates with Monte Carlo window poses,
  - the loop list remains 20 valid alternating goals: 10 primary-window
    approach goals and 10 shelf-window goals split 5 low / 5 high,
  - the deepest checked shelf-side loop goal reaches tool x about `-0.779 m`.
- Verified after tightening the red-ring windows:
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo ur5_goal_monte_carlo ur5_path_diagnose`,
  - `./build/ur5_goal_monte_carlo`,
  - `./build/ur5_goal_loop_demo --check-goals`,
  - `./build/ur5_goal_loop_demo --check-transitions`,
  - transition coverage hit all three windows: primary, shelf-low, and
    shelf-high,
  - `./build/ur5_clutter_plan --no-live`,
  - one-shot path hit the primary ring and both shelf windows with 0 planned
    obstacle-contact states and 0 planned clearance-violation states,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, and closest red-ring
    distance `0.015159 m`.
- Changed the shared realtime execution loop back to 60 Hz:
  - execution timestep is `1/60 s`,
  - each control tick computes the trajectory target, applies one PID/servo
    update, advances MuJoCo once, and sleeps remaining wall time in realtime
    mode.
- Verified after restoring 60 Hz execution:
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo`,
  - `./build/ur5_goal_loop_demo --check-goals`,
  - `./build/ur5_clutter_plan --no-live`,
  - `MuJoCo timestep: 0.0166667 s`,
  - PID execution steps: `301`,
  - PID simulated time: `5.01667 s`,
  - PID final max joint error: `0.0187608 rad`,
  - PID obstacle-contact steps: `0`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, and closest red-ring
    distance `0.015159 m`.
- Moved the shelf windows into side-by-side A/B/C compartments:
  - kept the primary red ring as the approach-side threading/staging window,
  - replaced the stacked low/high shelf windows with `shelf_window_a`,
    `shelf_window_b`, and `shelf_window_c` at the shelf face,
  - widened the shelf window openings enough for the UR5e collision geometry
    while keeping the A/B/C centers separated across the shelf,
  - generalized window validation and Monte Carlo selection to three shelf
    buckets.
- Regenerated the endless-loop goals for A/B/C cycling:
  - `./build/ur5_goal_monte_carlo --samples 800000` produced 12 approach goals
    and 4 goals for each shelf window,
  - `./build/ur5_goal_loop_demo --check-goals` verified 24 goals: 12
    approach-side and 12 shelf-side, balanced 4/4/4 across A/B/C,
  - `./build/ur5_goal_loop_demo --check-transitions` verified all 24 loop
    transitions and nonzero transition coverage for primary, A, B, and C
    windows,
  - default `./build/ur5_goal_monte_carlo` also succeeds after the refreshed
    one-shot candidate anchors.
- Verified after the A/B/C window change:
  - `cmake --build build --target ur5_clutter_plan ur5_goal_loop_demo ur5_goal_monte_carlo ur5_path_diagnose`,
  - `./build/ur5_clutter_plan --no-live`,
  - `MuJoCo timestep: 0.0166667 s`,
  - PID execution steps: `301`,
  - PID obstacle-contact steps: `0`,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 contacts, 0 negative robot-ring distances, and closest red-ring
    distance `0.018087 m`.
- Replaced the shelf A/B/C windows with a four-column pole scene:
  - removed the three red shelf-window bodies,
  - added four thin vertical red shelf-entry poles at the shelf entry plane,
  - removed the old shelf-entry guard posts so the three spaces between poles
    are the actual shelf-entry constraints,
  - kept the primary red ring as the approach-side threading/staging window,
  - changed the shelf-side logical targets to `shelf_gap_a`, `shelf_gap_b`,
    and `shelf_gap_c`,
  - set the finish line at the pole plane and the shelf goal target to 1 cm
    deeper toward the shelf.
- Regenerated robot poses for all three pole gaps:
  - `./build/ur5_goal_monte_carlo --samples 500000` produced 12 approach goals
    and 4 goals through each shelf pole gap,
  - refreshed one-shot home/goal candidates from those generated poses,
  - refreshed the 24-goal endless-loop initializer to use one robust
    approach-side staging pose between shelf-gap A, B, C poses because the
    fully diverse approach list produced brittle 500 ms transition pairs.
- Verified after the pole-gap scene change:
  - `cmake --build build --target ur5_goal_loop_demo ur5_clutter_plan ur5_goal_monte_carlo`,
  - default `./build/ur5_goal_monte_carlo` now uses 500k samples and produces
    at least 4 goals for each shelf gap,
  - `./build/ur5_goal_loop_demo --check-goals` verifies 24 loop entries, 12
    approach-side entries, 12 shelf-side entries, and balanced 4/4/4 coverage
    across `shelf_gap_a`, `shelf_gap_b`, and `shelf_gap_c`,
  - `./build/ur5_goal_loop_demo --check-transitions` verifies all 24 loop
    transitions and nonzero transition coverage for the primary ring and all
    three shelf gaps,
  - `./build/ur5_clutter_plan --no-live` selected a `shelf_gap_a` goal and
    planned/executed with 0 planned contacts, 0 planned clearance violations,
    0 PID obstacle-contact steps, and 0 PID clearance-violation trace rows,
  - `./build/ur5_path_replay --check build/scenes/ur5e_clutter.xml executed_trace.csv`,
  - `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
    found 0 robot-ring contacts, 0 negative robot-ring distances, and closest
    robot-ring distance `0.015483 m`.
- Increased the motion-planning wall-time budget from 500 ms to 3000 ms:
  - `kPlanningTimeBudgetSeconds` is now `3.000 s`,
  - README budget text and the approximate-path rejection message now report
    the 3000 ms budget,
  - verified with `cmake --build build --target ur5_goal_loop_demo
    ur5_clutter_plan`, `./build/ur5_goal_loop_demo --check-goals`, and
    `./build/ur5_clutter_plan --no-live`, which reported
    `Planning time budget: 3.000 s`.
- Added RRTConnect loop path reuse without keeping old planner trees alive:
  - fresh RRTConnect loop plans now store reusable geometric path knots per
    loop transition key,
  - revisiting the same transition rehydrates the cached path into an OMPL
    `PathGeometric`, validates/repairs it, then spends the normal postprocess
    budget on `reduceVertices`, `ropeShortcutPath`, `partialShortcutPath`,
    clearance checks, and the existing C2/fallback pipeline,
  - cached paths are discarded and replanned if they fail validation,
  - the live overlay now reports whether a segment came from a fresh solve or a
    cached path,
  - added `--check-cache` to smoke-test a fresh RRTConnect transition followed
    by an immediate cached reuse of the same transition,
  - kept Informed RRT* on the existing fresh-solve behavior.
- Verified the cached-path change with:
  - `cmake --build build --target ur5_goal_loop_demo
    ur5_goal_loop_informed_rrtstar_demo -j`,
  - `./build/ur5_goal_loop_demo --check-cache`, which reported
    `first_used_cache=false`, `second_used_cache=true`, and one cached entry,
  - `./build/ur5_goal_loop_informed_rrtstar_demo --check-goals`.

## Next

- Run `./build/ur5_clutter_plan` normally to see live PID execution immediately
  after planning.
- Run `./build/ur5_goal_loop_demo` to watch the endless goal-to-goal realtime
  planning/execution loop.
- Use `./build/ur5_goal_loop_demo --check-transitions` after changing the loop
  goal list.
- Use `./build/ur5_goal_monte_carlo` to regenerate candidate loop goals.
- Use `./build/ur5_clutter_plan --no-live` for repeated planner/debug batches.
- Use `./build/ur5_clutter_plan --live-fast` when you want an unpaced fast
  preview instead of realtime playback.
- Use `./build/ur5_path_diagnose build/scenes/ur5e_clutter.xml planned_path.csv 128`
  when a visual pass looks suspicious; trust this over eyeballing perspective.
