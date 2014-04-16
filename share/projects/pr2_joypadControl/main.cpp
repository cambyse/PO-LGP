#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/joystick/joystick.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>


struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, joystickState);
  RosCom *ros;
  MySystem():ros(NULL){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", false))
      ros = addModule<RosCom>(NULL, Module_Thread::loopWithBeat, .001);
    connect();
  }
};

void testJoypad(){
  ors::KinematicWorld world("model.kvg");
  arr q, qdot;
  world.getJointState(q, qdot);

  FeedbackMotionControl MP(world, false);
  MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  MySystem S;
  engine().open(S);

  if(S.ros){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==MP.world.q.N
         && S.ctrl_obs.get()->qdot.N==MP.world.q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;
    //arr fL_base = S.fL_obs.get();
    MP.setState(q, qdot);
  }
  arr zero_qdot(qdot.N);
  zero_qdot.setZero();
  CtrlMsg refs;

  for(uint t=0;;t++){
    S.joystickState.var->waitForNextRevision();
    arr joypadState = S.joystickState.get();

//    q    = S.q_obs.get();
//    qdot = S.qdot_obs.get();
//    MP.setState(q,qdot);

//    cout <<S.ctrl_obs.get()->fL <<endl;

    bool shutdown = j2t.updateTasks(joypadState);
    if(shutdown) engine().shutdown.incrementValue();

    arr a = MP.operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
    MP.reportCurrentState();
    MP.setState(q, qdot);
    if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(joypadState.N) mode = uint(joypadState(0));
    if(mode==2){
      cout <<"FORCE TASK" <<endl;
      refs.fL = ARR(10., 0., 0.);
      refs.fL_gainFactor = 1.;
      refs.Kp_gainFactor = .3;
    }else{
      refs.fL = ARR(0., 0., 0.);
      refs.fL_gainFactor = 0.;
      refs.Kp_gainFactor = 1.;
    }

    refs.q=q;
    refs.qdot=zero_qdot;
    S.ctrl_ref.set() = refs;
    if(S.ros) S.ros->publishJointReference();

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);

}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testJoypad();
  return 0;
}
