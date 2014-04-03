#include <pr2/actionMachine.h>
#include <pr2/actions.h>

void testActionMachine(){
  ActionSystem activity;
  activity.machine->add(new CoreTasks());

  engine().open(activity);
  // for (int i = 0; i < 20; ++i) {
    auto a1 = activity.machine->add(new MoveEffTo("endeffR", ARR(.8, -.4, 1.2)));
    // activity.machine->waitForActionCompletion(a1);
 
    auto a2 = activity.machine->add(new AlignEffTo("endeffR", ARR(1., 0., 0.), ARR(1., 0., 0.)),
                                    ActionState::queued);
    // auto a3 = activity.machine->add(new MoveEffTo("endeffL", ARR(.8, .4, 1.2)));

    activity.machine->waitForActionCompletion(a2);
    // activity.machine->waitForActionCompletion(a3);

    // auto a4 = activity.machine->add(new PushForce("endeffR", ors::Vector(15, 0, 0), ARR(2, 0, 0)))
    // activity.machine->waitForActionCompletion(a4);
  // }
  engine().close(activity);
}

void testActionMachineOrig(){
  // ActionSystem S;

  // S.machine->addGroundedAction(coreTasks, NULL, NULL, NoArr, NoArr);
  // GroundedAction *a1 = S.machine->addGroundedAction(moveEffTo, "endeffR", NULL, ARR(.5,-.4,1.2), NoArr);
  // GroundedAction *a2 = S.machine->addGroundedAction(alignEffTo, "endeffR", NULL, ARR(1., 0., 0.), ARR(1., 0., 0.));
// //  GroundedAction *a2 = S.machine->addGroundedAction(moveEffTo, "endeffR", NULL, ARR(.5,-.4,.8), NoArr);
  // a2->actionState=GroundedAction::queued;
  // a2->dependsOnCompletion.append(a1);

  // engine().open(S);

  // S.machine->waitForActionCompletion(a2);
  // a1 = S.machine->addGroundedAction(moveEffTo, "endeffL", NULL, ARR(.5,-.2,.8), NoArr);
  // a2 = S.machine->addGroundedAction(alignEffTo, "endeffR", NULL, ARR(1., 0., 0.), ARR(0., 0., 1.));
  // S.machine->waitForActionCompletion(a1);
  // S.machine->waitForActionCompletion(a2);


// //  GroundedAction *a = S.machine->addGroundedAction(pushForce, "endeffR", NULL, ARR(10., 0., 0.), NoArr);

  // engine().close(S);
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testActionMachine();
  return 0;
}
