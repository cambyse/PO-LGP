#include <Core/thread.h>
#include <RosCom/roscom.h>
#include <Actions/gamepadControl.h>
#include <Control/TaskControlThread.h>
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
//  TaskControlThread tcm;
//  GamepadInterface gamepad;
//  GamepadControlActivity gpc;
//  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
//  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>          pub_ctrl_ref("/marc_rt_controller/jointReference", ctrl_ref);
//  SubscriberConv<geometry_msgs::PoseWithCovarianceStamped, arr, &conv_pose2transXYPhi>       sub_pr2_odom("/robot_pose_ekf/odom_combined", pr2_odom);
//  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subL("/ft_sensor/l_ft_compensated", Fl);
//  SubscriberConvNoHeader<geometry_msgs::WrenchStamped, arr, &conv_wrench2arr> subR("/ft_sensor/r_ft_compensated", Fr);


//  threadOpenModules(true);

//  moduleShutdown().waitForStatusGreaterThan(0);

//  threadCloseModules();
//  cout <<"bye bye" <<endl;
//  return 0;
//}

// =================================================================================================

struct RecordFTs : Thread {
  Access_typed<arr> wrenchL;
  ofstream fil;

  RecordFTs() : Thread("RecordeFTs"), wrenchL(this, "wrenchL", true){
  }

  void open(){ mlr::open(fil, "z.wrenchL"); }
  void close(){ fil.close(); }

  void step(){
    fil <<wrenchL.get()() <<endl;
  }

};

// =================================================================================================

int main(int argc, char** argv) {
  ActionSwigInterface S;
//  S.setVerbose(true);

  RecordFTs recorder;

  S.setFact("(Control vec endeffR){ vec1=[1 0 0] target=[0 0 1] }");
  S.setFact("(Control vec endeffL){ vec1=[1 0 0] target=[0 0 1] }");
  S.waitForCondition("(conv Control vec endeffR) (conv Control vec endeffL)");
  S.setFact("(Control vec endeffR)! (Control vec endeffL)! (conv Control vec endeffR)! (conv Control vec endeffL)!");
  mlr::wait(1.);

  S.setFact("(Control vec endeffR){ vec1=[1 0 0] target=[1 0 0] }");
  S.setFact("(Control vec endeffL){ vec1=[1 0 0] target=[1 0 0] }");
  S.waitForCondition("(conv Control vec endeffR) (conv Control vec endeffL)");
  S.setFact("(Control vec endeffR)! (Control vec endeffL)! (conv Control vec endeffR)! (conv Control vec endeffL)!");
  mlr::wait(1.);

//  S.setFact("(Control vec endeffR){ vec1=[1 0 0] target=[0 0 -1] }");
//  S.setFact("(Control vec endeffL){ vec1=[1 0 0] target=[0 0 -1] }");
//  S.waitForCondition("(conv Control vec endeffR)");
//  S.setFact("(Control vec endeffR)! (Control vec endeffL)! (conv Control vec endeffR)! (conv Control vec endeffL)!");
//  mlr::wait(1.);

//  S.setFact("(Control vec endeffR){ vec1=[1 0 0] target=[1 0 0] }");
//  S.setFact("(Control vec endeffL){ vec1=[1 0 0] target=[1 0 0] }");
//  S.waitForCondition("(conv Control vec endeffR)");
//  S.setFact("(Control vec endeffR)! (Control vec endeffL)! (conv Control vec endeffR)! (conv Control vec endeffL)!");
//  mlr::wait(1.);

  threadCloseModules();
  registry().clear();

  cout <<"bye bye" <<endl;
  return 0;
}
