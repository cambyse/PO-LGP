#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

// =================================================================================================
struct MySystem:System {
  ACCESS(CtrlMsg, ctrl_obs)

  MySystem() {
    addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
    addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    connect();
  }
};


// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);

  ors::KinematicWorld world("model.kvg");

  bool useRos = true;
  initialSyncJointStateWithROS(world, system.ctrl_obs, useRos);

  for (int i = 0; true; i++) {
    syncJointStateWitROS(world, system.ctrl_obs, useRos);
    world.gl().update(STRING("frame " << i), false, false, false);
    MT::wait(0.01);
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
