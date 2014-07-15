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
  MySystem(){
    addModule<JoystickInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    connect();
  }
};

void testJoypad(){
  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  ors::Joint *trans=world.getJointByName("worldTranslationRotation");
  ors::Shape *ftL_shape=world.getShapeByName("endeffForceL");

  ors::KinematicWorld worldCopy = world;

  FeedbackMotionControl MP(world, true);
  MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  MySystem S;
  engine().open(S);

  if(MT::getParameter<bool>("useRos", false)){
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
    bool shutdown = j2t.updateTasks(joypadState);
    if(t>10 && shutdown) engine().shutdown.incrementValue();

    // joint state
    worldCopy.setJointState(S.ctrl_obs.get()->q, S.ctrl_obs.get()->qdot);
    if(!(t%4))
      worldCopy.gl().update(STRING("real robot state"));

    arr a = MP.operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
    MP.reportCurrentState();
    MP.setState(q, qdot);
    //MP.world.reportProxies();
    if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(joypadState.N) mode = uint(joypadState(0));
    if(mode==2){
      arr y_fL, J_fL;
      MP.world.kinematicsPos(y_fL, J_fL, ftL_shape->body, &ftL_shape->rel.pos);
      cout <<"FORCE TASK" <<endl;
      refs.fL = ARR(10., 0., 0.);
      refs.fL_gainFactor = 1.;
      refs.Kp_gainFactor = .3;
      refs.u_bias = refs.fL_gainFactor*(~J_fL * refs.fL);
    }else{
      refs.fL = ARR(0., 0., 0.);
      refs.fL_gainFactor = 0.;
      refs.Kp_gainFactor = 1.;
      refs.u_bias = zeros(q.N);
    }

    refs.q=q;
    refs.qdot=zero_qdot;
//    refs.u_bias=zero_qdot;
    if(trans && trans->qDim()==3){
      refs.qdot(trans->qIndex+0) = qdot(trans->qIndex+0);
      refs.qdot(trans->qIndex+1) = qdot(trans->qIndex+1);
      refs.qdot(trans->qIndex+2) = qdot(trans->qIndex+2);
      if(true){ //no translations!
        refs.qdot(trans->qIndex+0) = 0.;
        refs.qdot(trans->qIndex+1) = 0.;
        refs.qdot(trans->qIndex+2) = 0.;
      }
    }
    S.ctrl_ref.set() = refs;

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);

}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testJoypad();
  return 0;
}
