#include <Actions/TaskControllerModule.h>
#include <Actions/swig.h>
#include <Actions/activities.h>
#include <Hardware/gamepad/gamepad.h>
#include <FOL/relationalMachineModule.h>

#include <Core/util.h>
#include <System/engine.h>

// ============================================================================

struct MyTask : TaskCtrlActivity {

  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt);
  virtual bool isConv();
};

void MyTask::configure2(const char *name, Graph& specs, ors::KinematicWorld& world) {
  map = new DefaultTaskMap(specs, taskController->modelWorld);
  task = new CtrlTask(name, *map, specs);
  stopTolerance=1e-2; //TODO: overwrite from specs
}

void MyTask::step2(double dt){
}

bool MyTask::isConv(){
  return ((task->y_ref.nd==1 && task->y.N==task->y_ref.N
           && maxDiff(task->y, task->y_ref)<stopTolerance
           && maxDiff(task->v, task->v_ref)<stopTolerance));
}

// ============================================================================
int main(int argc, char** argv) {
  registerActivity<MyTask>("MyTask");

  ActionSwigInterface S(false);

  S.createNewSymbol("MyTask");
  S.createNewSymbol("endeffR");
  S.createNewSymbol("endeffL");

  S.setFact("(MyTask endeffR){ type=pos, ref1=endeffR, target=[.2, -.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffL){ type=pos, ref1=endeffL, target=[.2, +.5, 1.3], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(conv MyTask endeffL)");
  S.waitForCondition("(conv MyTask endeffR)");
  S.setFact("(MyTask endeffL)!, (MyTask endeffR)!, (conv MyTask endeffL)!, (conv MyTask endeffR)!");
  S.setFact("(MyTask endeffR){ type=pos, ref1=endeffR, target=[.7, -.2, .7], PD=[.5, .9, .5, 10.]}");
  S.setFact("(MyTask endeffL){ type=pos, ref1=endeffL, target=[.7, +.2, .7], PD=[.5, .9, .5, 10.]}");
  S.waitForCondition("(quit)");

  return 0;
}


