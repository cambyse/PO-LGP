#include "trajectoryInterface.h"
#include <Algo/spline.h>
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>

TrajectoryInterface::TrajectoryInterface(ors::KinematicWorld &world_) {
  world = new ors::KinematicWorld(world_);
  world->q = world_.q;

  threadOpenModules(true); //engine().open(S);

  useRos = mlr::getParameter<bool>("useRos");
  fixBase = mlr::getParameter<bool>("fixBase",true);
  fixTorso = mlr::getParameter<bool>("fixTorso",true);

  if (useRos) {
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

    //-- define controller msg (TODO: read from MT.cfg)
    refs.fL = zeros(6);
    refs.KiFT.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = ARR(2.);
    refs.Kd = ARR(1.);
    refs.Ki = ARR(0.5);
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.8;
  }
}


void TrajectoryInterface::executeTrajectory(arr &X, double T, bool recordData)
{
  world->watch(true,"Press Enter to display trajectory");
  displayTrajectory(X,100,*world,"X");
  world->watch(true,"Press Enter to execute trajectory");

  /// compute spline for trajectory execution
  double dt = T/double(X.d0);
  cout <<"dt: " << dt << endl;
  cout <<"T: " << T << endl;
  arr Xdot;
  getVel(Xdot,X,dt);
  mlr::Spline XS(X.d0,X);
  mlr::Spline XdotS(Xdot.d0,Xdot);

  /// clear logging variables
  if (recordData) {logX = X; logT.clear(); logXdes.clear(); logXact.clear(); logFL.clear(); logU.clear(); logM.clear();}

  ors::Joint *trans = world->getJointByName("worldTranslationRotation");
  ors::Joint *torso = world->getJointByName("torso_lift_joint");

  arr q0;
  if (useRos) {
    q0 = S.ctrl_obs.get()->q;
  } else {
    q0 = world->getJointState();
  }

  mlr::timerStart(true);
  double t = 0.;

  while(t<T) {
    double s = t/T;
    if (s>1. || s<0.) { break;}
    /// set next target
    refs.q = XS.eval(s);
    refs.qdot = XdotS.eval(s);

    /// fix base
    if (fixBase) {
      refs.qdot(trans->qIndex) = 0.;
      refs.qdot(trans->qIndex+1) = 0.;
      refs.qdot(trans->qIndex+2) = 0.;
    }
    /// fix torso
    if (fixTorso) {
      refs.q(torso->qIndex) = q0(torso->qIndex);
      refs.qdot(torso->qIndex) = 0.;
    }

    /// set controller parameter
    if (useRos) { S.ctrl_ref.set() = refs;}

    mlr::wait(0.01);
    t = t + mlr::timerRead(true);

    world->setJointState(refs.q);
    world->gl().update();

    /// logging
    if (recordData) {
        logT.append(ARR(t));
        logXdes.append(~refs.q);
        logXact.append(~S.ctrl_obs.get()->q);
        logFL.append(~S.ctrl_obs.get()->fL);
        logU.append(~S.ctrl_obs.get()->u_bias);
    }
  }
}


void TrajectoryInterface::gotoPosition(arr x, double T) {
  MotionProblem MP(*world,false);
  MP.T = 100;
  MP.tau = 0.05;
  if (useRos) {
    MP.x0 = S.ctrl_obs.get()->q;
  }else{
    MP.x0 = world->getJointState();
  }

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

  executeTrajectory(X,T);
}


void TrajectoryInterface::recordDemonstration(arr &X,double T,double dt,double T_start) {
  mlr::wait(T_start);

  /// send zero gains
  CtrlMsg refs_zero;
  refs_zero.q = S.ctrl_obs.get()->q;
  refs_zero.qdot=S.ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.KiFT.clear();
  refs_zero.J_ft_inv.clear();
  refs_zero.u_bias = zeros(q.N);
  refs_zero.Kp = zeros(q.N,q.N);
  refs_zero.Kd = ARR(0.);
  refs_zero.Ki = ARR(0.);
  refs_zero.gamma = 1.;
  refs_zero.velLimitRatio = .1;
  refs_zero.effLimitRatio = 1.;
  refs_zero.intLimitRatio = 1.;

  /// fix head joint during recording
  uint idx = world->getJointByName("head_pan_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;
  idx = world->getJointByName("head_tilt_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;

  S.ctrl_ref.set() = refs_zero;

  mlr::wait(3.);
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "START RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// record demonstrations
  double t = 0.;
  X.clear();
  mlr::timerStart(true);
  while(t<T) {
    arr q = S.ctrl_obs.get()->q;
    X.append(~q);
    mlr::wait(dt);
    t = t + mlr::timerRead(true);
  }
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "STOP RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  S.ctrl_ref.set() = refs;
}


void TrajectoryInterface::pauseMotion(bool sendZeroGains) {
  cout << "stopping motion" << endl;

  /// send zero gains
  CtrlMsg refs_zero = refs;
  refs_zero.q = S.ctrl_obs.get()->q;
  refs_zero.qdot=S.ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.KiFT.clear();
  refs_zero.J_ft_inv.clear();
  refs_zero.u_bias = zeros(q.N);
  if (sendZeroGains) {
    cout << "sending zero gains" << endl;
    refs_zero.Kp = ARR(0.);
    refs_zero.Kd = ARR(0.);
    refs_zero.Ki = ARR(0.);
  }
  refs_zero.gamma = 1.;
  S.ctrl_ref.set() = refs_zero;

  world->watch(true,"press button to continue");
  cout << "continuing motion" << endl;

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  S.ctrl_ref.set() = refs;
}


void TrajectoryInterface::logging(mlr::String folder, uint id) {
  write(LIST<arr>(logXact),STRING(folder<<"Xact"<<id<<".dat"));
  write(LIST<arr>(logXdes),STRING(folder<<"Xdes"<<id<<".dat"));

  write(LIST<arr>(logT),STRING(folder<<"T"<<id<<".dat"));
  write(LIST<arr>(logX),STRING(folder<<"X"<<id<<".dat"));
  write(LIST<arr>(logX),STRING(folder<<"X.dat"));

  if (logFL.N>0) write(LIST<arr>(logFL),STRING(folder<<"FL"<<id<<".dat"));
  if (logM.N>0) write(LIST<arr>(logM),STRING(folder<<"M"<<id<<".dat"));
  if (logU.N>0) write(LIST<arr>(logU),STRING(folder<<"U"<<id<<".dat"));
}
