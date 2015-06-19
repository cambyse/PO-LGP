// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <Hardware/G4/G4.h>
// #include <Hardware/G4/module_G4Debugger.h>

#include "pd_executor_module.h"

// ============================================================================
struct PR2G4Control:System {
  PR2G4Control() {
    addModule<G4Poller>(NULL, Module::loopWithBeat, .01);
    addModule<GamepadInterface>(NULL, Module::loopWithBeat, .01);
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

