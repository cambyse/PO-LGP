#pragma once
#include <Core/array.h>

#if 0
struct Pr2GamepadController : Thread {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(arr, gamepadState)

  mlr::KinematicWorld world;
  arr q, qdot, zero_qdot;
  TaskControlMethods *MP;
  mlr::Shape *ftL_shape;
  mlr::Joint *trans;
  Gamepad2Tasks *j2t;
  CtrlMsg refs;

  Pr2GamepadController() : Thread("Pr2GamepadController"){
  }
  void open(){
    world.init("model.kvg");
    makeConvexHulls(world.shapes);
    world >>FILE("z.ors");
    world.getJointState(q, qdot);
    trans=world.getJointByName("worldTranslationRotation");
    ftL_shape=world.getShapeByName("endeffL");

    MP = new TaskControlMethods(world, true);
    MP->qitselfPD.y_ref = q;
    MP->H_rate_diag = world.getHmetric();
    j2t = new Gamepad2Tasks(*MP);


    if(mlr::getParameter<bool>("useRos", false)){
      //-- wait for first q observation!
      cout <<"** Waiting for ROS message on initial configuration.." <<endl;
      for(;;){
        ctrl_obs.data->waitForNextRevision();
        if(ctrl_obs.get()->q.N==MP->world.q.N
           && ctrl_obs.get()->qdot.N==MP->world.q.N)
          break;
      }

      //-- set current state
      cout <<"** GO!" <<endl;
      q = ctrl_obs.get()->q;
      qdot = ctrl_obs.get()->qdot;
      //arr fL_base = S.fL_obs.get();
      MP->setState(q, qdot);
    }
    zero_qdot.resize(qdot.N).setZero();
  }
  void step(){
    arr gamepad = this->gamepadState.get();
    j2t->updateTasks(gamepad);

    //compute control
    arr a = MP->operationalSpaceControl();
    q += .01*qdot;
    qdot += .01*a;
//    MP->reportCurrentState();  MP->world.reportProxies();
    MP->setState(q, qdot);
//    if(!(t%4))
//      MP->world.gl().update(STRING("local operational space controller state t="<<(double)t/100.), false, false, false);

    //-- force task
    uint mode = 0;
    if(gamepad.N) mode = uint(gamepad(0));
    if(mode==2){
      arr y_fL, J_fL;
      MP->world.kinematicsPos(y_fL, J_fL, ftL_shape->body, ftL_shape->rel.pos);
      cout <<"FORCE TASK" <<endl;
      refs.fL = ARR(10., 0., 0.);
      J_fL = J_fL.sub(0,1,0,-1);
      arr gain = 10.*(~J_fL*J_fL) + .3*eye(q.N);
      cout <<J_fL <<gain <<endl;
//      refs.u_bias = 1.*(~J_fL * refs.fL);
      refs.Kp = gain;
//      refs.Kp = ARR(.3);
      refs.u_bias = zeros(q.N);
    }else{
      refs.fL = ARR(0., 0., 0.);
      refs.Kp = ARR(1.);
      refs.u_bias = zeros(q.N);
    }

    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.);
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
    ctrl_ref.set() = refs;

  }
  void close(){
  }
};
#endif
