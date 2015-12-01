#include <Core/module.h>
#include <pr2/roscom.h>
#include <Actions/gamepadControl.h>
#include <pr2/TaskControllerModule.h>
#include <Hardware/gamepad/gamepad.h>
#include <Actions/swig.h>


// =================================================================================================
//int main(int argc, char** argv){
//  mlr::initCmdLine(argc, argv);

//  rosCheckInit("calibrateFTsensors");

//  ACCESSname(CtrlMsg, ctrl_ref)
//  ACCESSname(CtrlMsg, ctrl_obs)
//  ACCESSname(arr, Fl);
//  ACCESSname(arr, Fr);
//  ACCESSname(arr, pr2_odom)

//  RosCom_Spinner spinner;
//  TaskControllerModule tcm;
//  GamepadInterface gamepad;
//  GamepadControlActivity gpc;
//  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
//  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub_ctrl_ref("/marc_rt_controller/jointReference", ctrl_ref);
//  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub_pr2_odom("/robot_pose_ekf/odom_combined", pr2_odom);
//  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subL("/ft_sensor/l_ft_compensated", Fl);
//  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subR("/ft_sensor/r_ft_compensated", Fr);


//  threadOpenModules(true);

//  moduleShutdown().waitForValueGreaterThan(0);

//  threadCloseModules();
//  cout <<"bye bye" <<endl;
//  return 0;
//}

// =================================================================================================

int main(int argc, char** argv) {
  ActionSwigInterface S;

  ACCESSname(RelationalMachine, RM)

      cout <<RM.get()->KB <<endl;

//  cout <<S.getSymbols() <<endl;
  S.setFact("(Control vec endeffR){ vec1=[1 0 0] target=[0 0 1] }");
  S.setFact("(Control vec endeffL){ vec1=[1 0 0] target=[0 0 1] }");
  S.waitForCondition("(conv Control vecDiff endeffR)");


  threadCloseModules();
  registry().clear();

  cout <<"bye bye" <<endl;
  return 0;
}
