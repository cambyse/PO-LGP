// #include <RosCom/actionMachine.h>
// #include <RosCom/actions.h>
#include <Control/taskController.h>
#include <Hardware/gamepad/gamepad.h>
#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Publisher.h>
#include <Hardware/G4/module_G4Recorder.h>
// #include <Hardware/G4/module_G4Debugger.h>

#include "calibrator_module.h"
#include "pd_executor_module.h"

// ============================================================================
struct PR2G4Control:System {
  PR2G4Control() {
    addModule<G4Poller>(NULL, Module::loopWithBeat, .05);
    // auto g4debug = addModule<G4Debugger>(NULL, Module::listenFirst);
    // g4debug->id().load("g4mapping.kvg");
    
    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .05);
    addModule<Calibrator>("Calibrator", Module::loopWithBeat, .05);
    addModule<PDExecutor>("PDExecutor", Module::loopWithBeat, .05);

    // ROS
    if(mlr::getParameter<bool>("useRos", false)) {
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::loopWithBeat, .001);
    }
    connect();
  };
};

// ============================================================================
int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);

  PR2G4Control system;
  engine().open(system);
  engine().shutdown.waitForSignal();
  engine().close(system);

  return 0;
}

