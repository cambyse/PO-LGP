#include <Actions/actionMachine.h>
#include <Actions/actions.h>


void testActionMachine() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  // auto a1 =
  //     new MoveEffTo("endeffR", ARR(.6, -.5, 1.2));
  // activity.machine->waitForActionCompletion(a1);
  
  // auto a2 =
  //     new AlignEffTo("endeffR", ARR(1., 0., 0.), ARR(1., 0., 0.));
  // activity.machine->waitForActionCompletion(a2);
  // auto a3 = new MoveEffTo("endeffL", ARR(.8, .4, 1.2));
  
  // auto action =
  //     new MoveEffTo("endeffR", ARR(.6, -.4, 1.0))
  // );
  // activity.machine->waitForActionCompletion(action);
  
  // action =
  //     new AlignEffTo("endeffR", ARR(1, 0, 0), ARR(1, 0, 0));
  GroundedAction* action = new PushForce(*activity.machine, "endeffR", ARR(1, 2, 3)/*, ARR(1, 4, 8)*/);
  activity.machine->waitForActionCompletion(action);
  
  // MT::wait(2);
  
  // activity.machine->waitForActionCompletion(a3);
  
  // auto a4 = new PushForce("endeffR", ors::Vector(15, 0, 0), ARR(2, 0, 0)))
  // activity.machine->waitForActionCompletion(a4);
  // }
  engine().close(activity);
}

void do_the_dance() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  for(int i = 0; i < 5; ++i) {
    GroundedAction* a_right = new MoveEffTo(*activity.machine, "endeffR", {.6, -.5, 1.2});
    GroundedAction* a_left = new MoveEffTo(*activity.machine, "endeffL", {.6, .6, 1.2});
    activity.machine->waitForActionCompletion(a_left);
    activity.machine->waitForActionCompletion(a_right);
    
    GroundedAction* a_right2 = new MoveEffTo(*activity.machine, "endeffR", {.3, -.7, 1.0});
    GroundedAction* a_left2 = new MoveEffTo(*activity.machine, "endeffL", {.3, .7, 1.0});
    activity.machine->waitForActionCompletion(a_left2);
    activity.machine->waitForActionCompletion(a_right2);
  }
  
  engine().close(activity);
}

void test_push() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  GroundedAction* a_right = new MoveEffTo(*activity.machine, "endeffR", {.6, -.3, 1});
  activity.machine->waitForActionCompletion(a_right);
  cout << "waiting" << endl;
  MT::wait(3);
  GroundedAction* push = new PushForce(*activity.machine, "endeffR", {.0, -.05, 0}/*, {0., 1., 0.}*/);
  activity.machine->waitForActionCompletion(push);
  
  engine().close(activity);
}

void idle() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  GroundedAction* right = new MoveEffTo(*activity.machine, "endeffR", {.95, -.2, .9});
  GroundedAction* left = new MoveEffTo(*activity.machine, "endeffL", {.95, .2, .9});
  activity.machine->waitForActionCompletion(right);
  activity.machine->waitForActionCompletion(left);
  
  // GroundedAction* align_right = new AlignEffTo(*activity.machine, "endeffR", {.95, 0, 0.}, {.95, 0, 0});
  // GroundedAction* align_left = new AlignEffTo(*activity.machine, "endeffL", {.95, 0, 0.}, {.95, 0, 0});
  // activity.machine->waitForActionCompletion(align_right);
  // activity.machine->waitForActionCompletion(align_left);
  
  engine().close(activity);
}

void test_collision() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  new MoveEffTo(*activity.machine, "endeffR", {.8, .1, .9});
  new MoveEffTo(*activity.machine, "endeffL", {.8, -.1, .9});
  activity.machine->waitForActionCompletion();
  cout << "actions done" << endl;
  MT::wait(5);
  engine().close(activity);
}

void idle2() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  auto t = new OrientationQuat(*activity.machine, "endeffR", {1, 1, 0, 0});
  activity.machine->waitForActionCompletion(t);
  cout << "Done waiting" << endl;
  
  new MoveEffTo(*activity.machine, "endeffR", {.8, -.2, .9});
//  new OrientationQuat("endeffR", {1, 1, 0, 0});

  activity.machine->waitForActionCompletion();
  MT::wait(5);
  
  // new MoveEffTo(*activity.machine, "endeffR", {.6, -.2, .9});
  // new AlignEffTo(*activity.machine, "endeffR", {1, 0, 0.}, {1, 0, 0});
  // activity.machine->waitForActionCompletion();
  
  engine().close(activity);
}


// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
  // test_push();
  // idle();
  // idle2();
  // return 0;
  // test_collision();
  do_the_dance();
  // testActionMachine();
  
  return 0;
}
