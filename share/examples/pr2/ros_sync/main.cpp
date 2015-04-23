#include <System/engine.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <pr2/roscom.h>

// =================================================================================================
struct MySystem:System {
  // Access Variables
  // ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs)

  MySystem() {
    if (MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
    }
    connect();
  }
};

// =================================================================================================
/**
 * This starts the initial sync of the world with ctrl_obs from the robot.
 *
 * This is verbose (helps debugging) and retries to connect to the robot multiple times.
 *
 * If useRos==false then nothing happens.
 */
void initial_sync_PR2_with_world(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  //-- wait for first q observation!
  cout << "** Waiting for ROS message of joints for initial configuration.." << endl
       << "   If nothing is happening: is the controller running?" << endl;

  for (uint trials = 0; trials < 20; trials++) {
    ctrl_obs.var->waitForNextRevision();
    cout << "REMOTE joint dimension=" << ctrl_obs.get()->q.N << endl;
    cout << "LOCAL  joint dimension=" << world.q.N << endl;

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      cout << "** Updating world state" << endl;
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
    cout << "retrying..." << endl;
  }
  HALT("sync'ing real PR2 with simulated failed");
}

/**
 * Sync the world with ctrl_obs from the robot.
 *
 * If useRos==false then nothing happens.
 */
void sync_PR2_with_world(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  for (uint trials = 0; trials < 2; trials++) {
    ctrl_obs.var->waitForNextRevision();

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
  }
  HALT("sync'ing real PR2 with simulated failed");
}

// =================================================================================================
int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  MySystem system;
  engine().open(system);

  ors::KinematicWorld world("model.kvg");
  // makeConvexHulls(world.shapes);

  bool useRos = MT::getParameter<bool>("useRos", false);
  if (useRos) {
    cout << "Using ROS" << endl;
  } else {
    cout << "NOT using ROS" << endl;
  }

  initial_sync_PR2_with_world(world, system.ctrl_obs, useRos);

  for (int i = 0; true; i++) {
    sync_PR2_with_world(world, system.ctrl_obs, useRos);
    world.gl().update(STRING("frame " << i), false, false, false);
    MT::wait(0.01);
  }

  engine().close(system);
  cout <<"bye bye" <<endl;
  return 0;
}
