#include <Actions/swig.h>
#include <Actions/TaskControllerModule.h>
#include <Actions/activities.h>
#include <Hardware/gamepad/gamepad.h>
#include <FOL/relationalMachineModule.h>

#include <Core/util.h>
#include <System/engine.h>

// ============================================================================
struct MySystem : System{
  ACCESS(bool, quitSignal)
  ACCESS(RelationalMachine, RM)
  ACCESS(MT::String, effects)
  MySystem(){
    addModule<TaskControllerModule>(NULL, Module::loopWithBeat, .01);
    addModule<RelationalMachineModule>(NULL, Module::loopWithBeat, .01);
    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
//      addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
    }
    connect();
  }
};

// ============================================================================
int main(int argc, char** argv) {
  registerActivity<FollowReferenceActivity>("FollowReferenceActivity");

  MySystem S;
  engine().open(S);

  for(;;){
    S.quitSignal.waitForNextRevision();
    if(S.quitSignal.get()==true) break;
//    S.RM.waitForNextRevision();
//    if(S.RM.set()->queryCondition("(quit)")) break;
  }

  engine().close(S);

  return 0;
}


