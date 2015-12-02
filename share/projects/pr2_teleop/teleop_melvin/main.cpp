//#include <pr2/actionMachine.h>
//#include <pr2/actions.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Publisher.h>
#include <Hardware/G4/module_G4Recorder.h>
#include <Hardware/G4/module_G4Display.h>
// #include <Hardware/G4/module_G4Debugger.h>

#include "g4mapper/calibrator_module.h"
#include "g4taprecon/taprecon.h"
#include "pd_executor/pd_executor_module.h"

// ============================================================================

struct PR2G4Control {
  GamepadInterface gpi;
  G4Poller g4poller;
  G4HutoRoMap hutoro;
  G4MoveRecon g4moverecon;
  PDExecutor exec;

  PR2G4Control() {
//    addModule<GamepadInterface>(NULL, .05);
//    addModule<G4Poller>(NULL, .05);
  //  addModule<G4Display>(NULL,Module::loopWithBeat,0.05);
//    addModule<G4HutoRoMap>("G4HuToRoMap", .05);
//    addModule<G4MoveRecon>("G4MoveRecon", .05);
    // auto g4debug = addModule<G4Debugger>(NULL, Module::listenFirst);
    // g4debug->id().load("g4mapping.kvg");
    //auto pd_executor = addModule<PDExecutor>(NULL, .05);
//   addModule<PDExecutor>("PDExecutor", .01);


    // ROS
    if(mlr::getParameter<bool>("useRos", false)) {
      //new RosSpinner;
      //addModule<RosCom_Spinner>(NULL, .001);
      //addModule<RosCom_ControllerSync>(NULL, .001);
    }
    //connect();
  };
};


// ============================================================================
int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);

  PR2G4Control system;
  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  return 0;
}

