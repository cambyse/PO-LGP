#include <Kin/kin.h>
#include <Control/taskControl.h>
#include <Hardware/gamepad/gamepad.h>
//#include <System/engine.h>
#include <Gui/opengl.h>


#include "simulator.h"
#include <Control/gamepad2tasks.h>

void TEST(Simulator){
  struct MySystem{
    ACCESS(arr, q_ref);
    ACCESS(arr, qdot_ref);
    ACCESS(arr, q_obs);
    ACCESS(arr, qdot_obs);
    ACCESS(arr, gamepadState);
    MySystem(){
      new PR2Simulator;
      new GamepadInterface;
      //connect();
    }
  } S;

  mlr::KinematicWorld world("model.kvg");
  arr q, qdot;
  world.getJointState(q, qdot);

  TaskControlMethods MP(world, false);
  MP.qitselfPD.y_ref = q;
  MP.qitselfPD.active=false;
  MP.H_rate_diag = world.getHmetric();
  Gamepad2Tasks j2t(MP);

  //engine().enableAccessLog();
  threadOpenModules(true);

  for(;;){
    S.qdot_obs.data->waitForNextRevision();
    arr gamepad = S.gamepadState.get();
    MP.setState(S.q_obs.get(), S.qdot_obs.get());
    MP.world.gl().update("operational space sim");
    bool shutdw = j2t.updateTasks(gamepad);
    if(shutdw) moduleShutdown().incrementValue();

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }
    S.q_ref.set() = q;
    S.qdot_ref.set() = qdot;
    if(moduleShutdown().getValue()) break; //waitForSignal();
  }

  threadCloseModules();
  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  //reach();
  testSimulator();

  return 0;
}
