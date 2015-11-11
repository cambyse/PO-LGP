//#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

// =================================================================================================
struct MySystem {
  ACCESS(CtrlMsg, ctrl_obs)

  MySystem() {
    new RosCom_Spinner();
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
    //new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
  }
};


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  MySystem system;
  threadOpenModules(true);

  ors::KinematicWorld world("model.kvg");

  bool useRos = true;
  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);

  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);
    world.gl().update(STRING("frame " << i), false, false, false);
    mlr::wait(0.01);
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
