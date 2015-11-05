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
struct PR2G4Control:System {
  PR2G4Control() {
    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .05);
    addModule<G4Poller>(NULL, Module::loopWithBeat, .05);
  //  addModule<G4Display>(NULL,Module::loopWithBeat,0.05);
    addModule<G4HutoRoMap>("G4HuToRoMap", Module::loopWithBeat , .05);
    addModule<G4MoveRecon>("G4MoveRecon", Module::loopWithBeat , .05);
    // auto g4debug = addModule<G4Debugger>(NULL, Module::listenFirst);
    // g4debug->id().load("g4mapping.kvg");
    //auto pd_executor = addModule<PDExecutor>(NULL, Module::loopWithBeat, .05);
   addModule<PDExecutor>("PDExecutor", Module::loopWithBeat, .01);


    // ROS
    if(MT::getParameter<bool>("useRos", false)) {
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::loopWithBeat, .001);
    }
    connect();
  };
};


// ============================================================================
int main(int argc, char** argv) {
  MT::initCmdLine(argc, argv);

  PR2G4Control system;
  engine().open(system);
  engine().shutdown.waitForSignal();
  engine().close(system);

  return 0;
}

