#include <pr2/actionMachine.h>

void testActionMachine(){
  ActionSystem S;

  S.machine->addGroundedAction(coreTasks, NULL, NULL, NoArr, NoArr);
  GroundedAction *a1 = S.machine->addGroundedAction(moveEffTo, "endeffR", NULL, ARR(.5,-.4,1.2), NoArr);
  GroundedAction *a2 = S.machine->addGroundedAction(alignEffTo, "endeffR", NULL, ARR(1., 0., 0.), ARR(1., 0., 0.));
//  GroundedAction *a2 = S.machine->addGroundedAction(moveEffTo, "endeffR", NULL, ARR(.5,-.4,.8), NoArr);
  a2->value=GroundedAction::queued;
  a2->dependsOnCompletion.append(a1);

  engine().open(S);

  S.machine->waitForActionCompletion(a2);
  a1 = S.machine->addGroundedAction(moveEffTo, "endeffL", NULL, ARR(.5,-.2,.8), NoArr);
  a2 = S.machine->addGroundedAction(alignEffTo, "endeffR", NULL, ARR(1., 0., 0.), ARR(0., 0., 1.));
  S.machine->waitForActionCompletion(a1);
  S.machine->waitForActionCompletion(a2);


//  GroundedAction *a = S.machine->addGroundedAction(pushForce, "endeffR", NULL, ARR(10., 0., 0.), NoArr);

  engine().close(S);
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testActionMachine();
  return 0;
}
