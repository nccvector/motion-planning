#include "motion_planning_goal_loop_demo.hpp"

int main(int argc, char** argv) {
  return RunGoalLoopDemo(
      argc, argv, PlannerKind::kRrtConnect, "UR5e endless goal loop - RRTConnect");
}
