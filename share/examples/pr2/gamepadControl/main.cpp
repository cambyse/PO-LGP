#include <Core/module.h>
#include <pr2/roscom.h>
#include <Actions/gamepadControl.h>
#include <Actions/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Ors/orsviewer.h>


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("gamepadControl");

  Access_typed<CtrlMsg> ctrl_ref(NULL, "ctrl_ref");
  Access_typed<CtrlMsg> ctrl_obs(NULL, "ctrl_obs");
  Access_typed<arr>     pr2_odom(NULL, "pr2_odom");

  TaskControllerModule tcm;
  GamepadInterface gamepad;
  GamepadControlActivity gpc;
  OrsViewer view;
  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages
  if(mlr::getParameter<bool>("useRos")){
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> ("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          ("/marc_rt_controller/jointReference", ctrl_ref);
    new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       ("/robot_pose_ekf/odom_combined", pr2_odom);
  }

  threadOpenModules(true);

  moduleShutdown().waitForValueGreaterThan(0);

  threadCloseModules();

  //NodeL subs = registry().getNodesOfType<SubscriberType*>();
  //for(Node *n:subs){ delete n->get<SubscriberType*>(); delete n; }
  cout <<"bye bye" <<endl;
  return 0;
}
