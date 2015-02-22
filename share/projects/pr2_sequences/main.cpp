#include <Actions/actionMachine.h>
#include <Actions/actionMachine_internal.h>
#include <Actions/actions.h>
#include <Motion/motionHeuristics.h>

#include <FOL/fol.h>


//===========================================================================
void TEST(Push) {
  ActionSystem activity;
  new CoreTasks(*activity.machine);
  engine().open(activity);
  
  Action *a, *b, *c;

  c = new FollowReference(*activity.machine, "moving",
                          new DefaultTaskMap(gazeAtTMT, *activity.machine->world, "endeffHead", NoVector, "endeffL"),
                          {}, {}, -1., .5, .9, .1, 10., 1., -1.);

  b = new FollowReference(*activity.machine, "moving", new DefaultTaskMap(vecTMT, *activity.machine->world, "endeffL", Vector_x),
                          {1./MT_SQRT2, 0, -1./MT_SQRT2}, {}, -1., .5, .9, .1, 10., 100., -1.);

  a = new FollowReference(*activity.machine, "moving", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .7}, {}, -1., .5, .9, .1, 10.);

  activity.machine->waitForActionCompletion(a);

  a = new FollowReference(*activity.machine, "orientation", new DefaultTaskMap(posTMT, *activity.machine->world, "endeffL"),
                          {.7, .3, .5}, {}, -1., .5, .9, .05, 10.);

  activity.machine->waitForActionCompletion(a);

  activity.machine->removeAction(b);
  activity.machine->removeAction(c);

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
  
  engine().close(activity);
}

// ============================================================================

void testFol(){
  KeyValueGraph G;
  FILE("machine.fol") >>G;
  G.checkConsistency();
  cout <<G <<endl;

  Item *terminal=G["Terminal"]->kvg()(0);
  forwardChaining_FOL(G, terminal, true);

}

// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);
  
//  testPush();
  testFol();
  
  return 0;
}
