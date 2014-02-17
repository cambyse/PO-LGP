#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>

#include "simulator.h"
#include "joystick2tasks.h"

void testSimulator(){
  struct MySystem:System{
    ACCESS(arr, q_ref);
    ACCESS(arr, qdot_ref);
    ACCESS(arr, q_obs);
    ACCESS(arr, qdot_obs);
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
  MP.H_rate_diag = pr2_reasonable_W(world);

  engine().enableAccessLog();
  engine().open(S);

  for(;;){
    S.qdot_obs.var->waitForNextWriteAccess();
    arr joy = S.joystickState.get();
    MP.setState(S.q_obs.get(), S.qdot_obs.get());
//    MP.world.gl().update("operational space sim");
    j2t.updateTasks(joy,0.01);
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
