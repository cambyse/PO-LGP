#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>

#include "simulator.h"
#include "joystick2tasks.h"

void testSimulator(){
  struct MySystem:System{
    ACCESS(arr, q_ref);
    ACCESS(arr, qdot_ref);
    ACCESS(arr, q_obs);
    ACCESS(arr, joystickState);
    MySystem(){
      addModule<PR2Simulator>(NULL, ModuleThread::loopWithBeat, .001);
      addModule<JoystickInterface>(NULL, ModuleThread::loopWithBeat, .01);
      connect();
    }
  } S;

  ors::KinematicWorld world("model.kvg");
  FeedbackMotionControl MP(world, false);
  Joystick2Tasks j2t(MP);
  arr q, qdot;
  world.getJointState(q, qdot);
  MP.nullSpacePD.y_ref = q;
  MP.nullSpacePD.active=false;

  engine().enableAccessLog();
  engine().open(S);

  uint idx = world.getJointByName("l_shoulder_pan_joint")->qIndex;

  for(;;){
    S.joystickState.var->waitForNextWriteAccess();
    arr joy = S.joystickState.get();
#if 0
    arr q = S.q_obs.get();
    if(q.N && joy.N){
      q(idx) += 10.*joy(4);
      S.q_ref.set() = q;
    }
#else
    MP.setState(q, qdot);
//    MP.world.gl().update("operational space sim");
    j2t.updateTasks(joy);
    for(uint tt=0;tt<10;tt++){
      arr a = MP.operationalSpaceControl();
      q += .001*qdot;
      qdot += .001*a;
    }
    S.q_ref.set() = q;
    S.qdot_ref.set() = qdot;
#endif
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
