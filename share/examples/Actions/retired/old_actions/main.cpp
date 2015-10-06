#include <Actions/actionMachine.h>
#include <Actions/actionMachine_internal.h>
#include <Actions/actions.h>
#include <Motion/motionHeuristics.h>


//==============================================================================
void gripper() {
  ActionSystem activity;
  auto& m = *activity.machine;

  new CoreTasks(*activity.machine);
  engine().open(activity);

  auto target = m.s->world.getShapeByName("mymarker")->X;
  cout << target << endl;
  Action* open_left = new OpenGripper(*activity.machine, Side::LEFT);
  Action* open_right = new OpenGripper(*activity.machine, Side::RIGHT);
  activity.machine->waitForActionCompletion(open_left);
  activity.machine->waitForActionCompletion(open_right);

  engine().close(activity);
}

//==============================================================================
// grab the marker
void grap_the_marker() {
  ActionSystem activity;
  auto& m = *activity.machine;

  new CoreTasks(*activity.machine);
  engine().open(activity);

  Action* open_right = new OpenGripper(m, Side::RIGHT);
  // m.waitForActionCompletion(open_right);

  // align along marker
  auto target = m.s->world.getShapeByName("mymarker")->X;
  cout << "target:" << target << endl;
  arr target_position = conv_vec2arr(target.pos);
  arr target_orientation = conv_quat2arr(target.rot);
  Action* pose_to = new PoseTo(m, "endeffR", target_position, target_orientation);

  // TODO
  // cage marker

  // Action* close_right = new CloseGripper(*activity.machine, Side::RIGHT);
  // activity.machine->waitForActionCompletion(open_right);

  m.waitForActionCompletion(open_right);
  m.waitForActionCompletion(pose_to);
  engine().close(activity);
}

//==============================================================================
void TEST(Dance) {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);

  for(int i = 0; i < 2; ++i) {
    Action* a_right = new MoveEffTo(*activity.machine, "endeffR", {.6, -.5, 1.2});
    Action* a_left = new MoveEffTo(*activity.machine, "endeffL", {.6, .6, 1.2});
    activity.machine->waitForActionCompletion(a_left);
    activity.machine->waitForActionCompletion(a_right);

    Action* a_right2 = new MoveEffTo(*activity.machine, "endeffR", {.3, -.7, 1.0});
    Action* a_left2 = new MoveEffTo(*activity.machine, "endeffL", {.3, .7, 1.0});
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
  TaskMap *t = new DefaultTaskMap(posTMT, activity.machine->s->world, "endeffR"); //that the constructor requires a 'world' is ugly!

  // then the action
  Action *a = new FollowReferenceInTaskSpace(*activity.machine, "my_follow_task", t, q, 5.);

  cout <<"I'm here...waiting" <<endl;
  MT::wait(7.);

  engine().close(activity);
}

//===========================================================================
void TEST(Push) {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);

  Action *a, *b;

  b = new FollowReference(*activity.machine, "moving", new DefaultTaskMap(vecTMT, *activity.machine->world, "endeffL", Vector_x),
                          {1./MT_SQRT2, 0, -1./MT_SQRT2}, {}, -1., .5, .9, .1, 10., 100., -1.);

  a = new FollowReference(*activity.machine, "moving", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .7}, {}, -1., .5, .9, .1, 10.);

  activity.machine->waitForActionCompletion(a);

  a = new FollowReference(*activity.machine, "orientation", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .5}, {}, -1., .5, .9, .05, 10.);

  activity.machine->waitForActionCompletion(a);

  activity.machine->removeAction(b);

  a = new Homing(*activity.machine, "homing");
  activity.machine->waitForActionCompletion(a);

#if 0
  a = new MoveEffTo(*activity.machine, "endeffR", {.7, -.5, .6});

  b = new AlignEffTo(*activity.machine, "endeffR", {1, 0, 0.}, {0, 0, -1.});
  activity.machine->waitForActionCompletion(a);
  activity.machine->waitForActionCompletion(b);

  a = new MoveEffTo(*activity.machine, "endeffR", {.7, -.5, .6});
  b = new AlignEffTo(*activity.machine, "endeffR", {1, 0, 0.}, {0, 0, -1.});
  activity.machine->waitForActionCompletion(a);
  activity.machine->waitForActionCompletion(b);

#endif

//  cout << "waiting" << endl;
//  MT::wait(3);
//  cout << "pushing" << endl;
//  Action* push = new PushForce(*activity.machine, "endeffR", {.0, -.05, 0}/*, {0., 1., 0.}*/);
//  activity.machine->waitForActionCompletion(push);

  MT::wait(1.);
  engine().close(activity);
}

//===========================================================================
void idle() {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);

  Action* right = new MoveEffTo(*activity.machine, "endeffR", {.95, -.2, .9});
  Action* left = new MoveEffTo(*activity.machine, "endeffL", {.95, .2, .9});
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

  auto t = new OrientationQuat(*activity.machine, "endeffR", {1., 1., 0., 0.});
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
  Action *core = new CoreTasks(*activity.machine);
  engine().open(activity);

  Action *t = new Relax(*activity.machine, "relax");
  core->active = false;
  activity.machine->s->feedbackController.useSwift=false;
  activity.ctrl_obs.waitForNextRevision();
  cout << "\nStart relax " << endl;

  arr trajQ, trajX;
  for (uint i =0;i<1000;i++){
    arr q = activity.ctrl_obs.get()->q;
    arr qdot = activity.ctrl_obs.get()->qdot;
    arr x = conv_vec2arr(activity.machine->s->world.getShapeByName("endeffR")->X.pos);
    activity.machine->s->feedbackController.setState(q,qdot);
    activity.machine->s->q = q;
    activity.machine->s->qdot = qdot;

    trajQ.append(~q);
    trajX.append(~x);
    MT::wait(0.01);
  }
  cout << "End recording" << endl;

  activity.machine->removeAction(t);
  core->active = true;

  write(LIST<arr>(trajX),"trajX.data");
  write(LIST<arr>(trajQ),"trajQ.data");

  engine().close(activity);
}

//===========================================================================
void test_replay() {
  ActionSystem activity;
  Action *core = new CoreTasks(*activity.machine);
  engine().open(activity);

  arr trajX;
  trajX << FILE("trajX.data");

  // 1. put robot in initial position
  arr x0 = trajX[0];
  cout << x0 << endl;
  cout <<"Goto init position" <<endl;
  Action* right = new MoveEffTo(*activity.machine, "endeffR", x0);
  activity.machine->waitForActionCompletion(right);
  MT::wait(3.);

  // 2. execute trajectory
  TaskMap *t = new DefaultTaskMap(posTMT, activity.machine->s->world, "endeffR"); //that the constructure requires a 'world' is ugly!
  Action *a = new FollowReferenceInTaskSpace(*activity.machine, "replay task", t, trajX, 10.);
  cout <<"Execute trajectory" <<endl;
  MT::wait(10.);

  engine().close(activity);
}


// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
  testPush();
//  idle();
//  idle2();
//  return 0;
//  test_collision();
//  testDance();
//  testFollowTrajectory();

//  test_record();
//  test_replay();

  
  return 0;
}
