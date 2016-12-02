#include "motion_interface.h"
#include <Algo/spline.h>



Motion_Interface::Motion_Interface(mlr::KinematicWorld &world_)
{
  world = new mlr::KinematicWorld(world_);
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


void Motion_Interface::executeTrajectory(arr &X, double duration) {
  arr Xdot;
  getVel(Xdot,X,duration/X.d0);

  mlr::Spline XS(X.d0,X);
  mlr::Spline XdotS(Xdot.d0,Xdot);

  Xdes.clear(); Xact.clear(); FLact.clear(); Tact.clear();
  mlr::timerStart(true);
  CtrlMsg refs;
  double t = 0.;
  double dt = duration/X.d0;
  double t_last = -dt;
  while(t<duration){
    double s = t/duration;
    refs.q = XS.eval(s);
    refs.qdot=XdotS.eval(s);
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+1) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+2) = 0.;
    refs.fL = zeros(6);
    refs.Ki.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 3.5;
    refs.Kd = 1.0;
    refs.Kint = .01;
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 1.0;
    S.ctrl_ref.set() = refs;

    t = t + mlr::timerRead(true);

    if (t-t_last >= dt){
      t_last = t;
      Tact.append(ARR(t));
      Xdes.append(~refs.q);
      Xact.append(~S.ctrl_obs.get()->q);
      FLact.append(~S.ctrl_obs.get()->fL);
    }
  }

  write(LIST<arr>(Tact),"plots/Tact.dat");
  write(LIST<arr>(Xdes),"plots/Xdes.dat");
  write(LIST<arr>(Xact),"plots/Xact.dat");
//  write(LIST<arr>(Vdes),"plots/Vdes.dat");
//  write(LIST<arr>(Vact),"plots/Vact.dat");
  if (FLact.N>0) write(LIST<arr>(FLact),"plots/FLact.dat");

}

void Motion_Interface::gotoPosition(arr &x)
{
  MotionProblem MP(*world,false);
  MP.T = 100;
  MP.tau = 0.05;
  MP.x0 = S.ctrl_obs.get()->q;

  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = world->getHmetric();
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

void Motion_Interface::recordDemonstration(arr &X,double duration)
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
  refs.Kint.clear();
  refs.gamma = 1.;
  refs.velLimitRatio = .1;
  refs.effLimitRatio = 1.;
  refs.intLimitRatio = .1;
  S.ctrl_ref.set() = refs;

  mlr::wait(5.);

  /// record demonstrations
  double t = 0.;
  mlr::timerStart(true);
  while(t<duration) {
    X.append(~S.ctrl_obs.get()->q);
    mlr::wait(0.1);
    t = t + mlr::timerRead(true);
  }

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp = 1.;
  refs.Kd = 1.;
  S.ctrl_ref.set() = refs;
}

