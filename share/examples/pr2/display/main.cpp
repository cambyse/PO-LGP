#include <Core/thread.h>
#include <RosCom/roscom.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Perception/perception.h>
#include <Ors/orsviewer.h>

// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);


  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(arr, pr2_odom)
  ACCESSname(byteA, modelCameraView)

  TaskControllerModule tcm;
  GamepadInterface gamepad;
  GamepadControlActivity gpc;
  OrsViewer orsviewer("modelWorld", true);
  ImageViewer camview("modelCameraView");
//  ImageViewer depthview("modelDepthView");

  if(mlr::getParameter<bool>("useRos", false)){
    rosCheckInit("display");
    new RosCom_Spinner;
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> ("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          ("/marc_rt_controller/jointReference", ctrl_ref);
    new SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       ("/robot_pose_ekf/odom_combined", pr2_odom);
  }

  threadOpenModules(true);

  mlr::wait(10.);
//  write_ppm( modelCameraView.get()(), "z.ppm", false);

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
