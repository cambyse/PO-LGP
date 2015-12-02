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

#include <Actions/teleopControl.h>
#include <Actions/TaskControllerModule.h>

#include <pr2/roscom.h>
#include <Perception/perception.h>

// ============================================================================

struct PR2G4Control {
  GamepadInterface gpi;
  G4Poller g4poller;
  G4HutoRoMap hutoro;
//  PDExecutor exec;

//  RosCom_Spinner spinner;
  TaskControllerModule tcm;
  TeleopControlActivity teleop;
  OrsViewer orsviewer;

//  GamepadInterface gamepad;
//  GamepadControlActivity gpc;
//  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
//  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub_ctrl_ref("/marc_rt_controller/jointReference", ctrl_ref);
//  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub_pr2_odom("/robot_pose_ekf/odom_combined", pr2_odom);


  PR2G4Control() {

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

