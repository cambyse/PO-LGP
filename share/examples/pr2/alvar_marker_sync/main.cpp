#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/rosalvar.h>

// =================================================================================================
struct MySystem:System {
  // Access Variables
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(ar_track_alvar_msgs::AlvarMarkers, ar_pose_marker)

  MySystem() {
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
    addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    addModule<ROSSUB_ar_pose_marker>(NULL, Module::listenFirst);
    connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  bool useRos = true;

  ors::KinematicWorld world("model.kvg");
  MySystem system;
  engine().open(system);

  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);
  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);

    ar_track_alvar_msgs::AlvarMarkers markers = system.ar_pose_marker.get();
    syncMarkers(world, markers);

    // world.calc_fwdPropagateFrames();
    // world.calc_fwdPropagateShapeFrames();

    world.gl().update(STRING("frame " << i), false, false, false);
    MT::wait(0.01);
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
