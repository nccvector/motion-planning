#include "ur5_goal_loop_demo.hpp"

int main(int argc, char** argv) {
  return RunGoalLoopDemo(argc,
                         argv,
                         PlannerKind::kSpars2,
                         "UR5e endless goal loop - SPARS2");
}
