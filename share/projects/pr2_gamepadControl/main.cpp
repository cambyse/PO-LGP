#include <Motion/gamepad2tasks.h>
#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Core/array-vector.h>

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, gamepadState);
  MySystem(){
    addModule<GamepadInterface>(NULL, Module_Thread::loopWithBeat, .01);
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ForceSensorSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    connect();
  }
};

void changeColor(void*){  orsDrawAlpha = .7; }
void changeColor2(void*){  orsDrawAlpha = 1.; }

void TEST(Gamepad){
  MySystem S;
  engine().open(S);

  ors::KinematicWorld world("model.kvg");
  makeConvexHulls(world.shapes);
  world >>FILE("z.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  ors::Joint *trans=world.getJointByName("worldTranslationRotation");

  ors::KinematicWorld world_pr2 = world;
  world.gl().add(changeColor);
//  world.gl().add(ors::glDrawGraph, &world_pr2);
  world.gl().add(changeColor2);

  FeedbackMotionControl MP(world, true);
  MP.qitselfPD.y_ref = q;
  MP.H_rate_diag = pr2_reasonable_W(world);
  Gamepad2Tasks j2t(MP);

  bool useRos = MT::getParameter<bool>("useRos", false);
  if(useRos){
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

  arr fLobs,uobs;
  ofstream fil("z.forces");
  MT::arrayBrackets="  ";

  for(uint t=0;;t++){
//    S.gamepadState.var->waitForNextRevision();
    arr gamepadState = S.gamepadState.get();
    bool shutdown = j2t.updateTasks(gamepadState);
    if(t>10 && shutdown) engine().shutdown.incrementValue();

    // joint state
    if(useRos){
      world_pr2.setJointState(S.ctrl_obs.get()->q, S.ctrl_obs.get()->qdot);
      fLobs = S.ctrl_obs.get()->fL;
      uobs = S.ctrl_obs.get()->u_bias;
    }

    //compute control
    arr a = MP.operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
    cout <<t <<endl;
//    MP.reportCurrentState();
    MP.setState(q, qdot);
    //MP.world.reportProxies();
    if(!(t%4))
      MP.world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(gamepadState.N) mode = uint(gamepadState(0));
    if(mode==2){
//      arr y_fL, J_fL;
//      MP.world.kinematicsPos(y_fL, J_fL, ftL_shape->body, &ftL_shape->rel.pos);
      cout <<"FORCE TASK" <<endl;
#if 0 // set a feew fwd force task
      refs.fL = ARR(10., 0., 0.);
      refs.u_bias = 1.*(~J_fL * refs.fL);
      refs.Kq_gainFactor = ARR(.3);
#else // apply force in direction fe
      arr fe = ARR(0.,0.,-5.);
      double alpha = .01;
      ors::Shape *ftL_shape = world.getShapeByName("endeffForceL");
      arr JeFT,Jeq;
      MP.world.kinematicsPos(NoArr,Jeq,ftL_shape->body,&ftL_shape->rel.pos);
      MP.world.kinematicsPos_wrtFrame(NoArr,JeFT,ftL_shape->body,&ftL_shape->rel.pos,MP.world.getShapeByName("l_ft_sensor"));

      JeFT = inverse_SymPosDef(JeFT*~JeFT)*JeFT;

      // compute u_bias
      refs.u_bias = ~Jeq*fe;
//      refs.u_bias = zeros(q.N);

      // compute force feedback
      refs.fL = fe;
      refs.KfL_gainFactor = alpha*~Jeq;
      refs.EfL = JeFT;

      Jeq = inverse_SymPosDef(Jeq*~Jeq)*Jeq;

      fil <<t <<' ' <<fe <<' ' << JeFT*fLobs << " " << Jeq*uobs << endl;

      // compute position gains that are 0 along force direction
//      arr yVec_fL, JVec_fL;
//      ors::Vector rel = ftL_shape->rel.rot*ors::Vector(fe/length(fe));
//      MP.world.kinematicsVec(yVec_fL, JVec_fL, ftL_shape->body,&rel);

//      ors::Quaternion quat;
//      quat.setDiff(ors::Vector(1.,0.,0.),yVec_fL);
//      arr R = ~quat.getArr();
//      arr J_fL0 = R*Jeq;
//      J_fL0[0]=0.;

//      refs.Kq_gainFactor = ~Jeq*inverse(Jeq * ~Jeq)*J_fL0;
//      refs.Kq_gainFactor = 10.*refs.Kq_gainFactor + 0.1*eye(q.N);

//       test gains
//      arr dq;
//      MP.world.getJointState(dq);
//      dq = dq*0.+0.1;
//      cout << refs.Kq_gainFactor*dq << endl;
//      cout << ~R*J_fL0*dq << endl;
//      arr dy = ~R*Jeq*refs.Kq_gainFactor*dq;
//      cout << R*dy << endl;

#endif
    }else{
      refs.fL = ARR(0., 0., 0.,0.,0.,0.);
      refs.KfL_gainFactor.clear();
      refs.EfL.clear();
      refs.u_bias = zeros(q.N);
    }
    refs.Kq_gainFactor = 0.;
    refs.Kd_gainFactor = 1.;
    refs.gamma = 1.;

    refs.q=q;
    refs.qdot=zero_qdot;
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
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    cout <<"ratios:" <<refs.velLimitRatio <<' ' <<refs.effLimitRatio <<endl;
    S.ctrl_ref.set() = refs;

    if(engine().shutdown.getValue()/* || !rosOk()*/) break;
  }

  engine().close(S);
  cout <<"bye bye" <<endl;
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testGamepad();
  return 0;
}
