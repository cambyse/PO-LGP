#include <RosCom/roscom.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>
#include <Actions/RelationalMachineModule.h>
#include <Actions/ActivitySpinnerModule.h>
#include <RosCom/serviceRAP.h>
#include <RosCom/baxter.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("minimalPositionControl");

  {
    Access_typed<sensor_msgs::JointState> jointState(NULL, "jointState");

    //-- setup a more complex 'system', mainly composed of the TaskController and the RelationalMachine
    TaskControllerModule tcm("baxter");
    RelationalMachineModule rm;
    ActivitySpinnerModule aspin;

    GamepadInterface gamepad;
    OrsPoseViewer ctrlView({"ctrl_q_real", "ctrl_q_ref"}, tcm.realWorld);

    SendPositionCommandsToBaxter spctb;
    if(mlr::getParameter<bool>("useRos")){
      new Subscriber<sensor_msgs::JointState> ("/robot/joint_states", jointState);
    }

    ServiceRAP rapservice;
    RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

    //-- ugly...
    for(Node *n:registry().getNodes("Activity")) rm.newSymbol(n->keys.last().p);
    for(ors::Shape *sh:tcm.realWorld.shapes) rm.newSymbol(sh->name.p);

    //-- run script
    threadOpenModules(true);
    rm.runScript("script.g");
//    moduleShutdown().waitForValueGreaterThan(0);
    threadCloseModules();
  }

  cout <<"bye bye" <<endl;
  return 0;
}
