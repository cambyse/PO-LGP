#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>

#include "roscom.h"

struct MySystem:System{
  ACCESS(arr, q);
  ACCESS(arr, qdot);
  ACCESS(arr, q_ref);
  ACCESS(arr, qdot_ref);
  ACCESS(arr, joystickState);
  RosCom *ros;
  MySystem(){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    ros = addModule<RosCom>(NULL, Module_Thread::loopFull);
    connect();
  }
};

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  ors::KinematicWorld world("scene");

  FeedbackMotionControl MP(world, false);
  MP.nullSpacePD.active=false;
  MP.H_rate_diag = .001* pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  MySystem S;
  engine().open(S);

  for(;;){
    S.joystickState.var->waitForNextRevision();
    arr joy = S.joystickState.get();
    arr q = S.q.get();
    arr qdot = S.qdot.get();

    if(q.N==7 && qdot.N==7){
      MP.setState(q, qdot);
      MP.world.gl().update();
    }

    //cout <<S.q.get()() <<endl;
    bool shutdown = j2t.updateTasks(joy, 0.01);
    if(shutdown) engine().shutdown.incrementValue();

    if(q.N && qdot.N==q.N){
      for(uint tt=0;tt<10;tt++){
        arr a = MP.operationalSpaceControl();
        q += .001*qdot;
        qdot += .001*a;
      }
      S.q_ref.set() = q;
      S.qdot_ref.set() = qdot;
      S.ros->publishJointReference();
    }

    if(engine().shutdown.getValue() || !rosOk()) break;
  }

  engine().close(S);

  return 0;
}
