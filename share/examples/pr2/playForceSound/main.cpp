#include <pr2/roscom.h>
#include <Actions/PlayForceSound.h>


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("playForceSound");

  ACCESSname(CtrlMsg, ctrl_obs)
  RosCom_Spinner spinner;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);
  PlayForceSoundActivity fs;

  threadOpenModules(true);

  mlr::wait(3.);

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
