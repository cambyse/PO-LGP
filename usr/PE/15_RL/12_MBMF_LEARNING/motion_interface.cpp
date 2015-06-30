#include "motion_interface.h"
#include <Algo/spline.h>
#include <Motion/pr2_heuristics.h>


Motion_Interface::Motion_Interface(ors::KinematicWorld &world_)
{
  world = new ors::KinematicWorld(world_);
  world->q = world_.q;
  engine().open(S);

  //-- wait for first q observation!
  cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  for(;;){
    S.ctrl_obs.var->waitForNextRevision();
    cout <<"REMOTE joint dimension=" <<S.ctrl_obs.get()->q.N <<endl;
    cout <<"LOCAL  joint dimension=" <<world->q.N <<endl;

    if(S.ctrl_obs.get()->q.N==world->q.N
       && S.ctrl_obs.get()->qdot.N==world->q.N)
      break;
  }

  //-- set current state
  cout <<"** GO!" <<endl;
  q = S.ctrl_obs.get()->q;
  qdot = S.ctrl_obs.get()->qdot;

  world->setJointState(q, qdot);
}


void Motion_Interface::executeTrajectory(arr &X, double T)
{
  arr Xdot;
  getVel(Xdot,X,T/X.d0);

  MT::Spline XS(X.d0,X);
  MT::Spline XdotS(Xdot.d0,Xdot);

  arr Xdes,Vdes,Vact,Xact;

  MT::timerStart(true);
  CtrlMsg refs;
  double t = 0.;
  while(t<T){
    double s = t/T;
    refs.q = XS.eval(s);
    refs.qdot=XdotS.eval(s)*0.;

    refs.fL = zeros(6);
    refs.Ki.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 1.0;
    refs.Kd = 1.0;
    refs.Kint = .01;
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = .1;
    S.ctrl_ref.set() = refs;

    t = t + MT::timerRead(true);

    Xdes.append(~refs.q);
    Xact.append(~S.ctrl_obs.get()->q);
    Vdes.append(~refs.qdot);
    Vact.append(~S.ctrl_obs.get()->qdot);
  }
  write(LIST<arr>(Xdes),"plots/Xdes.dat");
  write(LIST<arr>(Xact),"plots/Xact.dat");
  write(LIST<arr>(Vdes),"plots/Vdes.dat");
  write(LIST<arr>(Vact),"plots/Vact.dat");

  /*
  arr Xdot;
  getVel(Xdot,X,T/X.d0);

  MT::Spline XS(X.d0,X);
  MT::Spline XdotS(Xdot.d0,Xdot);

  arr Xdes,Vdes,Vact,Xact;

  MT::timerStart(true);
  CtrlMsg refs;
  double t = 0.;
  double gamma = 0.98;
  arr err = XS.eval(0.)*0.;
  while(t<T){
    double s = t/T;
    refs.q = XS.eval(s);
    refs.qdot=XdotS.eval(s)*0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+1) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+2) = 0.;

    refs.fL = zeros(6);
    refs.Ki.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
//    refs.u_bias = err;
    refs.Kp = 1.0;
    refs.Kd = 1.0;
    refs.Kint = .01;
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = .1;
    S.ctrl_ref.set() = refs;

    Xdes.append(~refs.q);
    Xact.append(~S.ctrl_obs.get()->q);
    Vdes.append(~refs.qdot);
    Vact.append(~S.ctrl_obs.get()->qdot);

    t = t + MT::timerRead(true);
//    uint qI = world->getJointByName("l_gripper_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 200.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_wrist_flex_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 1.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_wrist_roll_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 1.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_forearm_roll_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 1.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_elbow_flex_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 7.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_upper_arm_roll_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 8.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_shoulder_lift_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 10.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    qI = world->getJointByName("l_shoulder_pan_joint")->qIndex;
//    err(qI) = gamma*err(qI) + 20.*(refs.q(qI) - S.ctrl_obs.get()->q(qI));
//    err(world->getJointByName("laser_tilt_mount_joint")->qIndex) = 0.;
  }
  write(LIST<arr>(Xdes),"plots/Xdes.dat");
  write(LIST<arr>(Xact),"plots/Xact.dat");
  write(LIST<arr>(Vdes),"plots/Vdes.dat");
  write(LIST<arr>(Vact),"plots/Vact.dat");*/
}

void Motion_Interface::gotoPosition(arr &x)
{
  MotionProblem MP(*world,false);
  MP.T = 100;
  MP.tau = 0.05;
  MP.x0 = S.ctrl_obs.get()->q;

  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e0);

  t =MP.addTask("posT", new TaskMap_qItself());
  t->setCostSpecs(MP.T-2,MP.T, x, 1e2);

  MotionProblemFunction MPF(MP);
  arr X = MP.getInitialization();
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrainedMix(X, NoArr, Convert(MPF), o);
  executeTrajectory(X,MP.T*MP.tau);
}

void Motion_Interface::recordDemonstration(arr &X,double T)
{
  /// send zero gains
  CtrlMsg refs;
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.fL = zeros(6);
  refs.Ki.clear();
  refs.J_ft_inv.clear();
  refs.u_bias = zeros(q.N);
  refs.Kp = 0.;
  refs.Kd = 0.;
  refs.gamma = 1.;
  refs.velLimitRatio = .1;
  refs.effLimitRatio = 1.;
  S.ctrl_ref.set() = refs;

  MT::wait(3.);

  /// record demonstrations
  double t = 0.;
  MT::timerStart(true);
  while(t<T) {
    X.append(~S.ctrl_obs.get()->q);
    MT::wait(0.1);
    t = t + MT::timerRead(true);
  }

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp = 2.;
  refs.Kd = 1.;
  S.ctrl_ref.set() = refs;
}

void Motion_Interface::getJointState(arr &x)
{
  x = S.ctrl_obs.get()->q;
}
