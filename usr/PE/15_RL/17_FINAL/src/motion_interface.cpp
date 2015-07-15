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

  markers = S.ar_pose_marker.get();
  syncMarkers(*world, markers);

  world->setJointState(q, qdot);
}


void Motion_Interface::executeTrajectory(arr &X, double T)
{
  double dt = T/X.d0;
  Xref = X;
  arr Xdot;
  getVel(Xdot,X,dt);

  MT::Spline XS(X.d0,X);
  MT::Spline XdotS(Xdot.d0,Xdot);

  Tact.clear(); Xdes.clear(); Xact.clear(); FLact.clear(); Uact.clear(); Mact.clear();

  MT::timerStart(true);
  CtrlMsg refs;
  double t = 0.;
  double t_last = -dt;
  while(t<T){
    double s = t/T;
    refs.q = XS.eval(s);
    refs.qdot=XdotS.eval(s)*1.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+1) = 0.;
    refs.qdot(world->getJointByName("worldTranslationRotation")->qIndex+2) = 0.;

    refs.fL = zeros(6);
    refs.Ki.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 1.5;
    refs.Kd = 1.2;
    refs.Kint = .003;
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 1.5;
    S.ctrl_ref.set() = refs;

    t = t + MT::timerRead(true);

    if (t-t_last >= dt){
      t_last = t;
      Tact.append(ARR(t));
      Xdes.append(~refs.q);
      Xact.append(~S.ctrl_obs.get()->q);
      FLact.append(~S.ctrl_obs.get()->fL);
      Uact.append(~S.ctrl_obs.get()->u_bias);

      markers = S.ar_pose_marker.get();
      syncMarkers(*world, markers);
      Mact.append(~ARRAY(world->getBodyByName("marker4")->X.pos));
    }
  }

}

void Motion_Interface::gotoPosition(arr x)
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
  //  displayTrajectory(X,-1,*world,"demo");
  executeTrajectory(X,MP.T*MP.tau);
}

void Motion_Interface::recordDemonstration(arr &X,double T)
{
  MT::wait(5.);

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
  refs.intLimitRatio = 1.5;
  S.ctrl_ref.set() = refs;

  MT::wait(2.);
  cout << "start recording" << endl;

  /// record demonstrations
  double t = 0.;
  MT::timerStart(true);
  while(t<T) {
    X.append(~S.ctrl_obs.get()->q);
    MT::wait(0.1);
    t = t + MT::timerRead(true);
  }
  cout << "stop recording" << endl;

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp = 1.5;
  refs.Kd = 1.2;
  refs.Kint = ARR(.003);
  S.ctrl_ref.set() = refs;
}

void Motion_Interface::sendZeroGains(double T)
{
  cout << "sending zero gains" << endl;

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

  //  MT::wait(T);
  world->watch(true);

  cout << "resetting gains" << endl;
  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp = 1.;
  refs.Kd = 1.;
  S.ctrl_ref.set() = refs;
}

/// save last run
void Motion_Interface::logging(MT::String folder, uint id) {
  write(LIST<arr>(Xref),STRING(folder<<"X"<<id<<".dat"));
  write(LIST<arr>(Xref),STRING(folder<<"X.dat"));

  write(LIST<arr>(Xact),STRING(folder<<"Xact"<<id<<".dat"));
  write(LIST<arr>(Xact),STRING(folder<<"Xact.dat"));

  write(LIST<arr>(Xdes),STRING(folder<<"Xdes"<<id<<".dat"));
  write(LIST<arr>(Xdes),STRING(folder<<"Xdes.dat"));

  write(LIST<arr>(Tact),STRING(folder<<"Tdes"<<id<<".dat"));
  write(LIST<arr>(Tact),STRING(folder<<"Tdes.dat"));

  if (FLact.N>0) write(LIST<arr>(FLact),STRING(folder<<"FLact"<<id<<".dat"));
  if (FLact.N>0) write(LIST<arr>(FLact),STRING(folder<<"FLact.dat"));

  if (Mact.N>0) write(LIST<arr>(Mact),STRING(folder<<"Mact"<<id<<".dat"));
  if (Mact.N>0) write(LIST<arr>(Mact),STRING(folder<<"Mact.dat"));

  if (Uact.N>0) write(LIST<arr>(Uact),STRING(folder<<"Uact"<<id<<".dat"));
  if (Uact.N>0) write(LIST<arr>(Uact),STRING(folder<<"Uact.dat"));
}
