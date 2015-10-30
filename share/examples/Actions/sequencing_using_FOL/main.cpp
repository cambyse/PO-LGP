#include <Actions/TaskControllerModule.h>
//#include <Actions/taskCtrlActivities.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/sys.h>

#include <Hardware/gamepad/gamepad.h>

#include <Core/util.h>

// ============================================================================
struct MySystem {
  ACCESS(bool, quitSignal)
  ACCESS(RelationalMachine, RM)
  ACCESS(mlr::String, effects)
  ACCESS(mlr::String, state)
  ACCESS(ors::KinematicWorld, modelWorld)

  TaskControllerModule *tcm;

  MySystem(){
    tcm = addModule<TaskControllerModule>(NULL, Module::loopWithBeat, .01);
    addModule<ActivitySpinnerModule>(NULL, Module::loopWithBeat, .01);
    addModule<RelationalMachineModule>(NULL, Module::listenFirst);

    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
    if(mlr::getParameter<bool>("useRos",false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
//      addModule<RosCom_ForceSensorSync>(NULL, Module::loopWithBeat, 1.);
    }
    //    connect();
    createSymbolsForShapes(RM.set(), modelWorld.get());
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
    //    cout <<"new state: " <<S.state.get()() <<endl;
    if(S.RM.get()->queryCondition("(quit)")) break;
  }

  engine().close(S);

  return 0;
}


