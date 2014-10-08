#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>

#include "simulator.h"
#include <Motion/gamepad2tasks.h>

void testSimulator(){
  struct MySystem:System{
    ACCESS(arr, q_ref);
    ACCESS(arr, qdot_ref);
    ACCESS(arr, q_obs);
    ACCESS(arr, qdot_obs);
    ACCESS(arr, gamepadState);
    MySystem(){
      addModule<PR2Simulator>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
      connect();
    }
  } S;

  ors::KinematicWorld world("model.kvg");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  MP.qitselfPD.y_ref = q;
  MP.qitselfPD.active=false;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  engine().enableAccessLog();
  engine().open(S);

  for(;;){
    S.qdot_obs.var->waitForNextRevision();
    arr gamepad = S.gamepadState.get();
    MP.setState(S.q_obs.get(), S.qdot_obs.get());
    MP.world.gl().update("operational space sim");
    bool shutdown = j2t.updateTasks(gamepad);
    if(shutdown) engine().shutdown.incrementValue();

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }
    S.q_ref.set() = q;
    S.qdot_ref.set() = qdot;
    if(engine().shutdown.getValue()) break; //waitForSignal();
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  //reach();
  testSimulator();

  return 0;
}
