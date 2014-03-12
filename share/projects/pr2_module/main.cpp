#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>

#include "roscom.h"

struct MySystem:System{
  ACCESS(arr, q_ref);
  ACCESS(arr, qdot_ref);
  ACCESS(arr, q_obs);
  ACCESS(arr, qdot_obs);
  ACCESS(arr, fL_obs);
  ACCESS(arr, joystickState);
  RosCom *ros;
  MySystem(){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
    connect();
  }
};

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  ors::KinematicWorld world("model.kvg");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  MP.nullSpacePD.y_ref = q;
  MP.nullSpacePD.active=false;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  MySystem S;
  engine().open(S);
  S.joystickState.var->waitForNextRevision();

  //-- wait for first q observation!
  cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  for(;;){
    S.q_obs.var->waitForNextRevision();
    if(S.q_obs.get()->N==MP.world.q.N
       && S.qdot_obs.get()->N==MP.world.q.N
       && S.fL_obs.get()->N==3)
      break;
  }

  //-- set current state
  cout <<"** GO!" <<endl;
  q = S.q_obs.get();
  qdot = S.qdot_obs.get();
  MP.setState(q, qdot);
  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  arr fL_base = S.fL_obs.get();

  for(uint t=0;;t++){
    S.joystickState.var->waitForNextRevision();
    arr joy = S.joystickState.get();

//    q    = S.q_obs.get();
//    qdot = S.qdot_obs.get();
//    MP.setState(q,qdot);

    cout <<S.fL_obs.get()() <<endl;

    bool shutdown = j2t.updateTasks(joy);
    if(shutdown) engine().shutdown.incrementValue();

    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }
    MP.setState(q, qdot);
    if(!(t%10))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    S.q_ref.set() = q;
    S.qdot_ref.set() = zero_qdot;
//    S.ros->publishJointReference();

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);

  return 0;
}
