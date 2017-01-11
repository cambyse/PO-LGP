//#include <System/engine.h>
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include <RosCom/roscom.h>


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("ros_sync");

  ACCESSname(CtrlMsg, ctrl_obs)
  RosCom_Spinner spinner;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub_ctrl_obs("/marc_rt_controller/jointState", ctrl_obs);

  threadOpenModules(true);

  mlr::KinematicWorld world("model.kvg");

  bool useRos = true;
  initialSyncJointStateWithROS(world, ctrl_obs, useRos);

  for (int i = 0; i<200; i++) {
    syncJointStateWitROS(world, ctrl_obs, useRos);
    world.gl().update(STRING("frame " << i), false, false, false);
    mlr::wait(0.01);
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
