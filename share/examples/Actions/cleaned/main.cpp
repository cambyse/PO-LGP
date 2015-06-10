#include <Actions/swig.h>
#include <Actions/TaskControllerModule.h>
#include <Actions/taskCtrlActivities.h>
#include <Actions/RelationalMachineModule.h>
#include <Hardware/gamepad/gamepad.h>

#include <Core/util.h>
#include <System/engine.h>

// ============================================================================
struct MySystem : System{
  ACCESS(bool, quitSignal)
  ACCESS(RelationalMachine, RM)
  ACCESS(MT::String, effects)
  ACCESS(MT::String, state)
  TaskControllerModule *tcm;

  MySystem(){
    tcm = addModule<TaskControllerModule>(NULL, Module::loopWithBeat, .01);
    addModule<ActivitySpinnerModule>(NULL, Module::loopWithBeat, .01);
    addModule<RelationalMachineModule>(NULL, Module::listenFirst, .01);

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

  MySystem S;
  S.tcm->verbose=false;
  engine().open(S, true);

  for(;;){
    //    S.quitSignal.waitForNextRevision();
    //    if(S.quitSignal.get()==true) break;
    S.state.waitForNextRevision();
    cout <<"new state: " <<S.state.get()() <<endl;
    if(S.RM.set()->queryCondition("(quit)")) break;
  }

  engine().close(S);

  return 0;
}


