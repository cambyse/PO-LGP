//#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/subscribeAlvarMarkers.h>

// =================================================================================================
struct MySystem {
  // Access Variables
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(AlvarMarkers, ar_pose_markers)

  MySystem() {
    new RosCom_Spinner();
    new SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg>("/marc_rt_controller/jointState", ctrl_obs);
    new PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState>("/marc_rt_controller/jointReference", ctrl_ref);
    //addModule<ROSSUB_ar_pose_marker>(NULL /*,Module::listenFirst*/ );
    new Subscriber<AlvarMarkers>("/ar_pose_marker", (Access_typed<AlvarMarkers>&)ar_pose_markers);

    //connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  rosCheckInit("nodeName");
  bool useRos = true;

  ors::KinematicWorld world("model.kvg");
  MySystem system;
  threadOpenModules(true);

  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);
  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);

    AlvarMarkers markers = system.ar_pose_markers.get();
    syncMarkers(world, markers);

    // world.calc_fwdPropagateFrames();
    // world.calc_fwdPropagateShapeFrames();

    world.gl().update(STRING("frame " << i), false, false, false);
    mlr::wait(0.01);
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
  return 0;
}
