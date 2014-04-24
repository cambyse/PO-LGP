#include <pr2/actionMachine.h>
#include <pr2/actions.h>

void testActionMachine()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  // auto a1 = activity.machine->add(
  //     new MoveEffTo("endeffR", ARR(.6, -.5, 1.2)));
  // activity.machine->waitForActionCompletion(a1);

  // auto a2 = activity.machine->add(
  //     new AlignEffTo("endeffR", ARR(1., 0., 0.), ARR(1., 0., 0.)));
  // activity.machine->waitForActionCompletion(a2);
  // auto a3 = activity.machine->add(new MoveEffTo("endeffL", ARR(.8, .4, 1.2)));

  // auto action = activity.machine->add(
  //     new MoveEffTo("endeffR", ARR(.6, -.4, 1.0))
  // );
  // activity.machine->waitForActionCompletion(action);

  // action = activity.machine->add(
  //     new AlignEffTo("endeffR", ARR(1, 0, 0), ARR(1, 0, 0)));
  GroundedAction* action = activity.machine->add(
      new PushForce("endeffR", ARR(1, 2, 3), ARR(1, 4, 8))
  );
  activity.machine->waitForActionCompletion(action);

  // MT::wait(2);

    // activity.machine->waitForActionCompletion(a3);

    // auto a4 = activity.machine->add(new PushForce("endeffR", ors::Vector(15, 0, 0), ARR(2, 0, 0)))
    // activity.machine->waitForActionCompletion(a4);
  // }
  engine().close(activity);
}


void do_the_dance()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  for (int i = 0; i < 5; ++i) {
    GroundedAction* a_right = activity.machine->add(new MoveEffTo("endeffR", {.6, -.5, 1.2}));
    GroundedAction* a_left = activity.machine->add(new MoveEffTo("endeffL", {.6, .6, 1.2}));
    activity.machine->waitForActionCompletion(a_left);
    activity.machine->waitForActionCompletion(a_right);

    GroundedAction* a_right2 = activity.machine->add(new MoveEffTo("endeffR", {.3, -.7, 1.0}));
    GroundedAction* a_left2 = activity.machine->add(new MoveEffTo("endeffL", {.3, .7, 1.0}));
    activity.machine->waitForActionCompletion(a_left2);
    activity.machine->waitForActionCompletion(a_right2);
  }

  engine().close(activity);
}


void test_push()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  GroundedAction* a_right = activity.machine->add(new MoveEffTo("endeffR", {.6, -.3, 1}));
  activity.machine->waitForActionCompletion(a_right);
  cout << "waiting" << endl;
  MT::wait(3);
  GroundedAction* push = activity.machine->add(new PushForce("endeffR", {.0, -.05, 0}, {0., 1., 0.}));
  activity.machine->waitForActionCompletion(push);

  engine().close(activity);
}


void idle()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  GroundedAction* right = activity.machine->add(new MoveEffTo("endeffR", {.95, -.2, .9}));
  GroundedAction* left = activity.machine->add(new MoveEffTo("endeffL", {.95, .2, .9}));
  activity.machine->waitForActionCompletion(right);
  activity.machine->waitForActionCompletion(left);

  // GroundedAction* align_right = activity.machine->add(new AlignEffTo("endeffR", {.95, 0, 0.}, {.95, 0, 0}));
  // GroundedAction* align_left = activity.machine->add(new AlignEffTo("endeffL", {.95, 0, 0.}, {.95, 0, 0}));
  // activity.machine->waitForActionCompletion(align_right);
  // activity.machine->waitForActionCompletion(align_left);

  engine().close(activity);
}

void test_collision()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);
  activity.machine->add(new MoveEffTo("endeffR", {.8, .1, .9}));
  activity.machine->add(new MoveEffTo("endeffL", {.8, -.1, .9}));
  activity.machine->waitForActionCompletion();
  cout << "actions done" << endl;
  MT::wait(5);
  engine().close(activity);
}

void idle2()
{
  ActionSystem activity;
  activity.machine->add(new CoreTasks());
  engine().open(activity);

  activity.machine->add_sequence(
      new MoveEffTo("endeffR", {.95, -.2, .9}),
      new AlignEffTo("endeffR", {.95, 0, 0.}, {.95, 0, 0})
  );
  activity.machine->add_sequence(
      new MoveEffTo("endeffL", {.95, .2, .9}),
      new AlignEffTo("endeffL", {.95, 0, 0.}, {.95, 0, 0})
  );

  activity.machine->waitForActionCompletion();
  engine().close(activity);
}


// ============================================================================
int main(int argc, char** argv)
{
  MT::initCmdLine(argc, argv);
  // test_push();
  // idle();
  // idle2();
  test_collision();
  // do_the_dance();
  // testActionMachine();
  return 0;
}
