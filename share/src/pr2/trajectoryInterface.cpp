#include "trajectoryInterface.h"
#include <Algo/spline.h>
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>


TrajectoryInterface::TrajectoryInterface(ors::KinematicWorld &world_plan_,ors::KinematicWorld &world_pr2_) {
  world_plan = &world_plan_;
  world_pr2 = &world_pr2_;

  threadOpenModules(true);

  useRos = mlr::getParameter<bool>("useRos",false);
  fixBase = mlr::getParameter<bool>("fixBase",true);
  fixTorso = mlr::getParameter<bool>("fixTorso",true);
  useMarker = mlr::getParameter<bool>("useMarker",false);

  if (useRos) {
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for (;;) {
      S.ctrl_obs.var->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<S.ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<world_pr2->q.N <<endl;

      if (S.ctrl_obs.get()->q.N==world_pr2->q.N
          && S.ctrl_obs.get()->qdot.N==world_pr2->q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S.ctrl_obs.get()->q;
    qdot = S.ctrl_obs.get()->qdot;


    syncState();
    syncMarker();



    //-- define controller msg
    refs.fL = zeros(6);
    refs.KiFT.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = ARR(mlr::getParameter<double>("controller/Kp",1.5));
    refs.Kd = ARR(mlr::getParameter<double>("controller/Kd",1.5));
    refs.Ki = ARR(mlr::getParameter<double>("controller/Ki",0.));
    refs.gamma = 1.;
    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.9;
  }

  syncState();

  world_plan->gl().title = STRING("world_plan");
  world_pr2->gl().title = STRING("world_pr2");
  world_plan->watch(false);
  world_pr2->watch(false);
  world_plan->gl().resize(800,800);
  world_pr2->gl().resize(800,800);
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
  if (recordData) {logT.clear(); logXdes.clear(); logX.clear(); logFL.clear(); logU.clear(); logM.clear(); logM.resize(22); logXref = X_pr2;}

  ors::Joint *trans = world_pr2->getJointByName("worldTranslationRotation");
  ors::Joint *torso = world_pr2->getJointByName("torso_lift_joint");

  arr q0;
  if (useRos) {
    q0 = S.ctrl_obs.get()->q;
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
    if (useRos) { S.ctrl_ref.set() = refs;}

    t = t + mlr::timerRead(true);

    world_pr2->setJointState(refs.q);
    //    world_pr2->gl().update();

    syncMarker();

    /// logging
    if (recordData && (s<1.) && ((t-tPrev)>=dtLog)) {
      tPrev = t;
      logT.append(ARR(t));
      logXdes.append(~refs.q);
      if (useRos) {
        logX.append(~S.ctrl_obs.get()->q);
        logFL.append(~S.ctrl_obs.get()->fL);
        logFR.append(~S.ctrl_obs.get()->fR);
        logU.append(~S.ctrl_obs.get()->u_bias);

        if (useMarker) {
          for (uint i=0;i<21;i++) {
            ors::Body *body = world_plan->getBodyByName(STRING("marker"<<i));
            if (body) {
              logM(i).append(~cat(conv_vec2arr(body->X.pos),conv_quat2arr(body->X.rot)));
            }
          }
        }
      }
    }
  }
}

void TrajectoryInterface::syncState() {
  arr q_pr2,q_plan;
  if (useRos) {
    q_pr2 = S.ctrl_obs.get()->q;
  } else {
    q_pr2 = world_pr2->getJointState();
  }
  transferQbetweenTwoWorlds(q_plan,q_pr2,*world_plan,*world_pr2);
  world_pr2->setJointState(q_pr2);
  world_plan->setJointState(q_plan);

  /// sync torso
  if (world_plan->getJointByName("torso_lift_joint")->type==ors::JT_fixed) {
    world_plan->getJointByName("torso_lift_joint")->A = world_pr2->getJointByName("torso_lift_joint")->A;
    world_plan->getJointByName("torso_lift_joint")->Q.pos = world_pr2->getJointByName("torso_lift_joint")->Q.pos;

    world_plan->calc_q_from_Q();
    world_plan->calc_fwdPropagateFrames();
  }
}

void TrajectoryInterface::syncMarker() {
  if (useRos && useMarker) {
    markers = S.ar_pose_markers.get();
    syncMarkers(*world_pr2,markers);
    syncMarkers(*world_pr2,markers);
    syncMarkers(*world_plan,markers);
    syncMarkers(*world_plan,markers);
  }
}

void TrajectoryInterface::saveState(mlr::String filename) {
  arr q_pr2;
  getState(q_pr2);
  write(LIST<arr>(q_pr2),filename);
}

void TrajectoryInterface::getStatePlan(arr &q_plan) {
  syncState();
  world_plan->getJointState(q_plan);
}

void TrajectoryInterface::getState(arr &q_pr2) {
  syncState();
  world_pr2->getJointState(q_pr2);
}

void TrajectoryInterface::gotoPosition(mlr::String filename, double T, bool recordData, bool displayTraj) {
  arr q;
  q << FILE(filename); q.flatten();
  CHECK_EQ(q.N,world_pr2->getJointStateDimension(),STRING("gotoPosition: wrong joint state dimension"));
  fixTorso=false;
  gotoPosition(q,T,recordData,displayTraj);
  fixTorso=true;
}

void TrajectoryInterface::gotoPositionPlan(arr x_plan, double T, bool recordData, bool displayTraj) {
  CHECK_EQ(x_plan.N,world_plan->getJointStateDimension(),STRING("wrong joint state dimension"));
  arr x_pr2;
  transferQbetweenTwoWorlds(x_pr2,x_plan,*world_pr2,*world_plan);
  gotoPosition(x_pr2, T, recordData,displayTraj);
}

void TrajectoryInterface::gotoPosition(arr x_pr2, double T, bool recordData, bool displayTraj) {
  CHECK_EQ(x_pr2.N,world_pr2->getJointStateDimension(),STRING("wrong joint state dimension"));
  MotionProblem MP(*world_pr2,false);
  MP.T = 100;
  MP.tau = 0.05;
  if (useRos) {
    MP.x0 = S.ctrl_obs.get()->q;
  } else {
    MP.x0 = world_pr2->getJointState();
  }

  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world_pr2));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world_pr2);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e0);

  t =MP.addTask("posT", new TaskMap_qItself());
  t->setCostSpecs(MP.T-2,MP.T, x_pr2, 1e2);

  MotionProblemFunction MPF(MP);
  arr X_pr2 = MP.getInitialization();
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrained(X_pr2, NoArr, Convert(MPF), o);

  executeTrajectory(X_pr2,T,recordData,displayTraj);
}


void TrajectoryInterface::recordDemonstration(arr &X_pr2,double T,double dt,double T_start) {
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
  uint idx = world_pr2->getJointByName("head_pan_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;
  idx = world_pr2->getJointByName("head_tilt_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;

  S.ctrl_ref.set() = refs_zero;

  mlr::wait(3.);
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "START RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// record demonstrations
  double t = 0.;
  X_pr2.clear();
  mlr::timerStart(true);
  while(t<T) {
    arr q = S.ctrl_obs.get()->q;
    X_pr2.append(~q);
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

void TrajectoryInterface::moveLeftGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_pr2->getJointByName("l_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2,3.);
}

void TrajectoryInterface::moveRightGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_pr2->getJointByName("r_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2,3.);
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

  world_pr2->watch(true,"press button to continue");
  cout << "continuing motion" << endl;

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  S.ctrl_ref.set() = refs;
}


void TrajectoryInterface::logging(mlr::String folder, uint id) {

  write(LIST<arr>(logT),STRING(folder<<"T"<<id<<".dat"));
  write(LIST<arr>(logXdes),STRING(folder<<"Xdes"<<id<<".dat"));
  write(LIST<arr>(logXref),STRING(folder<<"Xref"<<id<<".dat"));

  if (logX.N>0) write(LIST<arr>(logX),STRING(folder<<"X"<<id<<".dat"));
  if (logFL.N>0) write(LIST<arr>(logFL),STRING(folder<<"FL"<<id<<".dat"));
  if (logFR.N>0) write(LIST<arr>(logFR),STRING(folder<<"FR"<<id<<".dat"));
  if (useMarker) {
    for (uint i=0;i<logM.N;i++){
      if (logM(i).N>0) {
        write(LIST<arr>(logM(i)),STRING(folder<<"M"<<i<<id<<".dat"));
      }
    }
  }

  if (logU.N>0) write(LIST<arr>(logU),STRING(folder<<"U"<<id<<".dat"));
}
