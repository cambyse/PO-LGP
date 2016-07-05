#include "trajectoryInterface.h"
#include <Algo/spline.h>
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>

#ifdef MLR_ROS

#include "roscom.h"
#include "spinner.h"

struct sTrajectoryInterface{
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState> pub;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub;
  RosCom_Spinner spinner;

  sTrajectoryInterface():
    pub("/marc_rt_controller/jointReference", ctrl_ref),
    sub("/marc_rt_controller/jointState", ctrl_obs) {
  }
};
#else
struct sTrajectoryInterface{
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
};
#endif

TrajectoryInterface::TrajectoryInterface(ors::KinematicWorld &world_plan_,ors::KinematicWorld &world_pr2_)
  : S(NULL){
  world_plan = &world_plan_;
  world_pr2 = &world_pr2_;

  S = new sTrajectoryInterface();
  threadOpenModules(true);

  useRos = mlr::getParameter<bool>("useRos");
  fixBase = mlr::getParameter<bool>("fixBase",true);
  fixTorso = mlr::getParameter<bool>("fixTorso",true);

  if (useRos) {
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for (;;) {
      S->ctrl_obs.var->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<S->ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<world_pr2->q.N <<endl;

      if (S->ctrl_obs.get()->q.N==world_pr2->q.N
         && S->ctrl_obs.get()->qdot.N==world_pr2->q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S->ctrl_obs.get()->q;
    qdot = S->ctrl_obs.get()->qdot;

    world_pr2->setJointState(q, qdot);
    arr q_plan;
    getStatePlan(q_plan);
    world_plan->setJointState(q_plan,q_plan*0.);

    //-- define controller msg
    refs.fL = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = ARR(mlr::getParameter<double>("controller/Kp",1.5));
    refs.Kd = ARR(mlr::getParameter<double>("controller/Kd",2.5));
    refs.Ki = ARR(mlr::getParameter<double>("controller/Ki",0.));
    refs.fL_gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.9;
  }
}

void TrajectoryInterface::executeTrajectoryPlan(arr &X_plan, double T, bool recordData, bool displayTraj) {
  /// convert trajectory into pr2 kinematics world
  arr X_pr2;
  transferQbetweenTwoWorlds(X_pr2,X_plan,*world_pr2,*world_plan);
  executeTrajectory(X_pr2, T, recordData,displayTraj);
}

void TrajectoryInterface::executeTrajectory(arr &X_pr2, double T, bool recordData, bool displayTraj) {
  if (displayTraj) {
    world_pr2->watch(true,"Press Enter to visualize motion");
    displayTrajectory(X_pr2,-1,*world_pr2,"X_pr2");
    world_pr2->watch(true,"Press Enter to execute motion");
  }
  /// compute spline for trajectory execution
  double dt = T/double(X_pr2.d0);
  cout <<"dt: " << dt << endl;
  cout <<"T: " << T << endl;
  arr Xdot;
  getVel(Xdot,X_pr2,dt);
  mlr::Spline XS(X_pr2.d0,X_pr2);
  mlr::Spline XdotS(Xdot.d0,Xdot);

  /// clear logging variables
  if (recordData) {logT.clear(); logXdes.clear(); logX.clear(); logFL.clear(); logU.clear(); logM.clear(); logXref = X_pr2;}

  ors::Joint *trans = world_pr2->getJointByName("worldTranslationRotation");
  ors::Joint *torso = world_pr2->getJointByName("torso_lift_joint");

  arr q0;
  if (useRos) {
    q0 = S->ctrl_obs.get()->q;
  } else {
    q0 = world_pr2->getJointState();
  }

  mlr::timerStart(true);
  double t = 0.;

  double dtLog = 0.05;
  double tPrev = -dtLog;
  while(t<T*1.5) {
    double s = t/T;
    if (s>1.) { s=1.;}
    if (s<0.) { break;}
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
    if (useRos) { S->ctrl_ref.set() = refs;}

    t = t + mlr::timerRead(true);

    world_pr2->setJointState(refs.q);
//    world_pr2->gl().update();

    /// logging
    if (recordData && (s<1.) && ((t-tPrev)>=dtLog)) {
      tPrev = t;
      logT.append(ARR(t));
      logXdes.append(~refs.q);
      logX.append(~S->ctrl_obs.get()->q);
      logFL.append(~S->ctrl_obs.get()->fL);
      logFR.append(~S->ctrl_obs.get()->fR);
      logU.append(~S->ctrl_obs.get()->u_bias);
    }
  }
}

void TrajectoryInterface::getStatePlan(arr &q_plan) {
  arr q_pr2 = S->ctrl_obs.get()->q;
  transferQbetweenTwoWorlds(q_plan,q_pr2,*world_plan,*world_pr2);
}

void TrajectoryInterface::getState(arr &q_pr2) {
  cout << "1" << endl;
//  S->ctrl_obs.var->waitForNextRevision();
  cout << "2" << endl;
  q_pr2 = S->ctrl_obs.get()->q;
}


void TrajectoryInterface::gotoPositionPlan(arr x_plan, double T, bool recordData, bool displayTraj) {
  arr x_pr2;
  transferQbetweenTwoWorlds(x_pr2,x_plan,*world_pr2,*world_plan);
  gotoPosition(x_pr2, T, recordData,displayTraj);
}

void TrajectoryInterface::gotoPosition(arr x_pr2, double T, bool recordData, bool displayTraj) {
  MotionProblem MP(*world_pr2,false);
  MP.T = 100;
  MP.tau = 0.05;
  if (useRos) {
    MP.world.setJointState(S->ctrl_obs.get()->q);
  }else{
    MP.world.setJointState(world_pr2->getJointState());
  }

  Task *t;
  t = MP.addTask("tra", new TaskMap_Transition(*world_pr2), sumOfSqrTT);
  ((TaskMap_Transition*)&t->map)->H_rate_diag = pr2_reasonable_W(*world_pr2);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e0);

  t =MP.addTask("posT", new TaskMap_qItself(), sumOfSqrTT);
  t->setCostSpecs(MP.T-2,MP.T, x_pr2, 1e2);

  arr X_pr2 = MP.getInitialization();
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrained(X_pr2, NoArr, Convert(MP), o);

  executeTrajectory(X_pr2, T, recordData, displayTraj);
}


void TrajectoryInterface::recordDemonstration(arr &X_pr2,double T,double dt,double T_start) {
  mlr::wait(T_start);

  /// send zero gains
  CtrlMsg refs_zero;
  refs_zero.q = S->ctrl_obs.get()->q;
  refs_zero.qdot=S->ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.KiFTL.clear();
  refs_zero.J_ft_invL.clear();
  refs_zero.u_bias = zeros(q.N);
  refs_zero.Kp = zeros(q.N,q.N);
  refs_zero.Kd = ARR(0.);
  refs_zero.Ki = ARR(0.);
  refs_zero.fL_gamma = 1.;
  refs_zero.velLimitRatio = .1;
  refs_zero.effLimitRatio = 1.;
  refs_zero.intLimitRatio = 1.;

  /// fix head joint during recording
  uint idx = world_pr2->getJointByName("head_pan_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;
  idx = world_pr2->getJointByName("head_tilt_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;

  S->ctrl_ref.set() = refs_zero;

  mlr::wait(3.);
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "START RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// record demonstrations
  double t = 0.;
  X_pr2.clear();
  mlr::timerStart(true);
  while(t<T) {
    arr q = S->ctrl_obs.get()->q;
    X_pr2.append(~q);
    mlr::wait(dt);
    t = t + mlr::timerRead(true);
  }
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "STOP RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// reset gains
  refs.q = S->ctrl_obs.get()->q;
  refs.qdot=S->ctrl_obs.get()->qdot*0.;
  S->ctrl_ref.set() = refs;
}

void TrajectoryInterface::moveLeftGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_pr2->getJointByName("l_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2,5.,false,true);
}

void TrajectoryInterface::moveRightGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_pr2->getJointByName("r_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2);
}

void TrajectoryInterface::pauseMotion(bool sendZeroGains) {
  cout << "stopping motion" << endl;

  /// send zero gains
  CtrlMsg refs_zero = refs;
  refs_zero.q = S->ctrl_obs.get()->q;
  refs_zero.qdot=S->ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.KiFTL.clear();
  refs_zero.J_ft_invL.clear();
  refs_zero.u_bias = zeros(q.N);
  if (sendZeroGains) {
    cout << "sending zero gains" << endl;
    refs_zero.Kp = ARR(0.);
    refs_zero.Kd = ARR(0.);
    refs_zero.Ki = ARR(0.);
  }
  refs_zero.fL_gamma = 1.;
  S->ctrl_ref.set() = refs_zero;

  world_pr2->watch(true,"press button to continue");
  cout << "continuing motion" << endl;

  /// reset gains
  refs.q = S->ctrl_obs.get()->q;
  refs.qdot=S->ctrl_obs.get()->qdot*0.;
  S->ctrl_ref.set() = refs;
}


void TrajectoryInterface::logging(mlr::String folder, uint id) {
  write(LIST<arr>(logX),STRING(folder<<"X"<<id<<".dat"));
  write(LIST<arr>(logXdes),STRING(folder<<"Xdes"<<id<<".dat"));
  write(LIST<arr>(logXref),STRING(folder<<"Xref"<<id<<".dat"));
  write(LIST<arr>(logT),STRING(folder<<"T"<<id<<".dat"));

  if (logFL.N>0) write(LIST<arr>(logFL),STRING(folder<<"FL"<<id<<".dat"));
  if (logFR.N>0) write(LIST<arr>(logFR),STRING(folder<<"FR"<<id<<".dat"));
  if (logM.N>0) write(LIST<arr>(logM),STRING(folder<<"M"<<id<<".dat"));
  if (logU.N>0) write(LIST<arr>(logU),STRING(folder<<"U"<<id<<".dat"));
}
