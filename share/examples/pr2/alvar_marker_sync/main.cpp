//#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/rosalvar.h>

// =================================================================================================
struct MySystem {
  // Access Variables
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(AlvarMarkers, ar_pose_marker)

  MySystem() {
    addModule<RosCom_Spinner>(NULL, /*Module::loopWithBeat,*/ .001);
    addModule<RosCom_ControllerSync>(NULL /*,Module::listenFirst*/ );
    addModule<ROSSUB_ar_pose_marker>(NULL /*,Module::listenFirst*/ );
    //connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  bool useRos = true;

  ors::KinematicWorld world("model.kvg");
  MySystem system;
  threadOpenModules(true);

  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);
  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);

    AlvarMarkers markers = system.ar_pose_marker.get();
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
