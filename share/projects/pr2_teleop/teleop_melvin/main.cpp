#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Display.h>
#include <Actions/teleopControl.h>
#include <Actions/TaskControllerModule.h>
#include <pr2/roscom.h>
#include <Perception/perception.h>
#include <Hardware/gamepad/gamepad.h>
#include "calibrator_module.h"

// ============================================================================

int main(int argc, char** argv) {
  mlr::initCmdLine(argc, argv);

  rosCheckInit("teleopNode");

  GamepadInterface gpi;
  G4Poller g4poller;
  G4HutoRoMap hutoro;

  TaskControllerModule tcm;
  TeleopControlActivity teleop;
  OrsViewer orsviewer;
  G4Display g4disp;

  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, pr2_odom)
  if(mlr::getParameter<bool>("useRos", false)){
    new RosCom_Spinner;
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> ("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          ("/marc_rt_controller/jointReference", ctrl_ref);
    new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       ("/robot_pose_ekf/odom_combined", pr2_odom);
  }

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  return 0;
}

