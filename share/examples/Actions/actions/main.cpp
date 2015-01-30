#include <Actions/actionMachine.h>
#include <Actions/actionMachine_internal.h>
#include <Actions/actions.h>
#include <Motion/motionHeuristics.h>


//===========================================================================
//very basic - just to play around

void TEST(ActionMachine){
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

//===========================================================================
// do some sequential hand movements

void TEST(Dance) {
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

//===========================================================================
// do some sequential hand movements
void TEST(FollowTrajectory) {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);



  // first construct the trajectory
  arr q = interpolate_trajectory({.6, -.5, 1.2}, {.6, .6, 1.2}, 100);

  // then construct a space in which to execute
  TaskMap *t = new DefaultTaskMap(posTMT, activity.machine->s->world, "endeffR"); //that the constructure requires a 'world' is ugly!

  // then the action
  GroundedAction *a = new FollowReferenceInTaskSpace(*activity.machine, "my_follow_task", t, q, 5.);

  cout <<"I'm here...waiting" <<endl;
  MT::wait(7.);

  engine().close(activity);
}

//===========================================================================
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

//===========================================================================
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

//===========================================================================
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

//===========================================================================
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

//===========================================================================
void test_record() {
  ActionSystem activity;
  GroundedAction *core = new CoreTasks(*activity.machine);
  engine().open(activity);

  GroundedAction *t = new Relax(*activity.machine, "relax");
  core->actionState = inactive;
  activity.machine->s->feedbackController.useSwift=false;
  activity.ctrl_obs.waitForNextRevision();
  cout << "\nStart relax " << endl;

  arr trajQ, trajX;
  for (uint i =0;i<1000;i++){
    arr q = activity.ctrl_obs.get()->q;
    arr qdot = activity.ctrl_obs.get()->qdot;
    arr x = ARRAY(activity.machine->s->world.getShapeByName("endeffR")->X.pos);
    activity.machine->s->feedbackController.setState(q,qdot);
    activity.machine->s->q = q;
    activity.machine->s->qdot = qdot;

    trajQ.append(~q);
    trajX.append(~x);
    MT::wait(0.01);
  }
  cout << "End recording" << endl;

  activity.machine->removeGroundedAction(t);
  core->actionState = active;

  write(LIST<arr>(trajX),"trajX.data");
  write(LIST<arr>(trajQ),"trajQ.data");

  engine().close(activity);
}

//===========================================================================
void test_replay() {
  ActionSystem activity;
  GroundedAction *core = new CoreTasks(*activity.machine);
  engine().open(activity);

  arr trajX;
  trajX << FILE("trajX.data");

  // 1. put robot in initial position
  arr x0 = trajX[0];
  cout << x0 << endl;
  cout <<"Goto init position" <<endl;
  GroundedAction* right = new MoveEffTo(*activity.machine, "endeffR", x0);
  activity.machine->waitForActionCompletion(right);
  MT::wait(3.);

  // 2. execute trajectory
  TaskMap *t = new DefaultTaskMap(posTMT, activity.machine->s->world, "endeffR"); //that the constructure requires a 'world' is ugly!
  GroundedAction *a = new FollowReferenceInTaskSpace(*activity.machine, "replay task", t, trajX, 10.);
  cout <<"Execute trajectory" <<endl;
  MT::wait(10.);

  engine().close(activity);
}


// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
  // test_push();
//   idle();
  // idle2();
  // return 0;
  // test_collision();
//  testDance();
//  testFollowTrajectory();
//   testActionMachine();

//  test_record();
  test_replay();

  
  return 0;
}
