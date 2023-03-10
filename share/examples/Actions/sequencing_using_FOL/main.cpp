#include <Control/TaskControlThread.h>
//#include <Actions/taskCtrlActivities.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/sys.h>

#include <Hardware/gamepad/gamepad.h>

#include <Core/util.h>

// ============================================================================
struct MySystem {
  ACCESSname(bool, quitSignal)
  ACCESSname(RelationalMachine, RM)
  ACCESSname(mlr::String, effects)
  ACCESSname(mlr::String, state)
  ACCESSname(mlr::KinematicWorld, modelWorld)

  TaskControlThread *tcm;

  MySystem(){
    tcm = addModule<TaskControlThread>(NULL, .01);
    addModule<ActivitySpinnerModule>(NULL, .01);
    addModule<RelationalMachineModule>(NULL );

    new GamepadInterface;
    if(mlr::getParameter<bool>("useRos",false)){
      new RosCom_Spinner();
      new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
      new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);

//      addModule<RosCom_ForceSensorSync>(NULL, /*Module::loopWithBeat,*/ 1.);
    }
    //    connect();
    createSymbolsForShapes(RM.set(), modelWorld.get());
  }
};

// ============================================================================
int main(int argc, char** argv) {

  MySystem S;
  S.tcm->verbose=false;
  threadOpenModules(true);


  for(;;){
    //    S.quitSignal.waitForNextRevision();
    //    if(S.quitSignal.get()==true) break;
    S.state.waitForNextRevision();
    //    cout <<"new state: " <<S.state.get()() <<endl;
    if(S.RM.get()->queryCondition("(quit)")) break;
  }

  threadCloseModules();

  return 0;
}


