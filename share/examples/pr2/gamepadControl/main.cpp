#include <Core/module.h>
#include <pr2/roscom.h>
#include <Actions/gamepadControl.h>
#include <Actions/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
//#include <Perception/perception.h>


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("gamepadControl");

  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, pr2_odom)

  RosCom_Spinner spinner;
  TaskControllerModule tcm;
  GamepadInterface gamepad;
  GamepadControlActivity gpc;
//  OrsViewer
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub_ctrl_ref("/marc_rt_controller/jointReference", ctrl_ref);
  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub_pr2_odom("/robot_pose_ekf/odom_combined", pr2_odom);

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
