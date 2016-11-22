#include "trajectoryInterface.h"
#include <Algo/spline.h>
#include <Motion/pr2_heuristics.h>
#include <Gui/opengl.h>
#include <RosCom/subscribeAlvarMarkers.h>
#include <Optim/convert.h>
#include <Optim/lagrangian.h>

#include "roscom.h"
#include "spinner.h"

struct sTrajectoryInterface{
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSname(ar::AlvarMarkers, ar_pose_markers)
  PublisherConv<marc_controller_pkg::JointState, CtrlMsg, &conv_CtrlMsg2JointState> pub;
  SubscriberConvNoHeader<marc_controller_pkg::JointState, CtrlMsg, &conv_JointState2CtrlMsg> sub;
  Subscriber<ar::AlvarMarkers> markerSub;

  RosCom_Spinner spinner;

  sTrajectoryInterface():
    pub("/marc_rt_controller/jointReference", ctrl_ref),
    sub("/marc_rt_controller/jointState", ctrl_obs),
    markerSub("/ar_pose_marker", (Access_typed<ar::AlvarMarkers>&)ar_pose_markers){
  }
};

TrajectoryInterface::TrajectoryInterface(ors::KinematicWorld &world_plan_,ors::KinematicWorld &world_robot_)
  : S(NULL){
  rosCheckInit("trajectoryInterface");

  S = new sTrajectoryInterface();
  threadOpenModules(true);

  world_plan = &world_plan_;
  world_robot = &world_robot_;

  world_plan->gl("world_plan"); world_robot->gl("world_robot");

  world_plan->watch(false); world_robot->watch(false);
  world_plan->gl().resize(600,600); world_robot->gl().resize(600,600);

  useRos = mlr::getParameter<bool>("useRos",false);
  useMarker = mlr::getParameter<bool>("useMarker",false);
  fixBase = mlr::getParameter<bool>("fixBase",true);
  fixTorso = mlr::getParameter<bool>("fixTorso",true);

  if (useRos) {
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
    for (;;) {
      S->ctrl_obs.var->waitForNextRevision();
      cout <<"REMOTE joint dimension=" <<S->ctrl_obs.get()->q.N <<endl;
      cout <<"LOCAL  joint dimension=" <<world_robot->q.N <<endl;

      if (S->ctrl_obs.get()->q.N==world_robot->q.N
         && S->ctrl_obs.get()->qdot.N==world_robot->q.N)
        break;
    }

    //-- set current state
    cout <<"** GO!" <<endl;
    q = S->ctrl_obs.get()->q;
    qdot = S->ctrl_obs.get()->qdot;

    syncState();
    syncMarker();

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

void TrajectoryInterface::executeTrajectoryPlan(arr &X_plan, double T, bool recordData, bool displayTraj, bool reverseMotion) {
  /// convert trajectory into pr2 kinematics world
  arr X_pr2;
  transferQbetweenTwoWorlds(X_pr2,X_plan,*world_robot,*world_plan);
  executeTrajectory(X_pr2, T, recordData,displayTraj);
}

void TrajectoryInterface::executeTrajectory(arr &X_robot, double T, bool recordData, bool displayTraj, bool reverseMotion) {
  arr Xref = X_robot;
  if (reverseMotion) {Xref.reverseRows();}
  if (displayTraj) {
    world_robot->watch(true,"Press Enter to visualize motion");
    displayTrajectory(Xref,-1,*world_robot,"Xref");
    world_robot->watch(true,"Press Enter to execute motion");
  }
  /// compute spline for trajectory execution
  double dt = T/double(Xref.d0);
  cout <<"Execute trajectory with dt= " << dt << " and T= "<<T << endl;
  arr Xdot;
  getVel(Xdot,Xref,dt);
  mlr::Spline XS(Xref.d0,Xref);
  mlr::Spline XdotS(Xdot.d0,Xdot);

  /// clear logging variables
  if (recordData) {logT.clear(); logXdes.clear(); logX.clear(); logFL.clear(); logU.clear(); logM.clear(); logM.resize(22); logXref = Xref;}

  ors::Joint *trans = world_robot->getJointByName("worldTranslationRotation");
  ors::Joint *torso = world_robot->getJointByName("torso_lift_joint");

  arr q0;
  if (useRos) {
    q0 = S->ctrl_obs.get()->q;
  } else {
    q0 = world_robot->getJointState();
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
    if (useRos) {
      S->ctrl_ref.set() = refs;
      t = t + mlr::timerRead(true);
    } else {
      t = t + 0.1;
    }

    world_robot->setJointState(refs.q);
    syncMarker();

    /// logging
    if (recordData && (s<1.) && ((t-tPrev)>=dtLog)) {
      tPrev = t;
      logT.append(ARR(t));
      logXdes.append(~refs.q);
      if (useRos) {
        logX.append(~S->ctrl_obs.get()->q);
        logFL.append(~S->ctrl_obs.get()->fL);
        logFR.append(~S->ctrl_obs.get()->fR);
        logU.append(~S->ctrl_obs.get()->u_bias);

        if (useMarker) {
          for (uint i=0;i<21;i++) {
            ors::Body *body = world_plan->getBodyByName(STRING("marker"<<i),false);
            if (body) {
              logM(i).append(~cat(conv_vec2arr(body->X.pos),conv_quat2arr(body->X.rot)));
            }
          }
        }
      }
    }
  }
}

void TrajectoryInterface::getStatePlan(arr &q_plan) {
  syncState();
  world_plan->getJointState(q_plan);
}

void TrajectoryInterface::getState(arr &q_robot) {
  syncState();
  world_robot->getJointState(q_robot);
}

void TrajectoryInterface::gotoPosition(mlr::String filename, double T, bool recordData, bool displayTraj) {
  arr q;
  q << FILE(filename);
  CHECK_EQ(q.N,world_robot->getJointStateDimension(),STRING("gotoPosition: wrong joint state dimension"));
  fixTorso=false;
  gotoPosition(q,T,recordData,displayTraj);
  fixTorso=true;
}

void TrajectoryInterface::gotoPositionPlan(arr x_plan, double T, bool recordData, bool displayTraj) {
  CHECK_EQ(x_plan.N,world_plan->getJointStateDimension(),STRING("wrong joint state dimension"));
  arr x_pr2;
  transferQbetweenTwoWorlds(x_pr2,x_plan,*world_robot,*world_plan);
  gotoPosition(x_pr2, T, recordData,displayTraj);
}

void TrajectoryInterface::gotoPosition(arr x_robot, double T, bool recordData, bool displayTraj) {
  CHECK_EQ(x_robot.N,world_robot->getJointStateDimension(),STRING("wrong joint state dimension"));
  MotionProblem MP(*world_robot,false);
  MP.T = 100;
  MP.tau = 0.05;
  if (useRos) {
    MP.world.setJointState(S->ctrl_obs.get()->q);
  }else{
    MP.world.setJointState(world_robot->getJointState());
  }

  Task *t;
  t = MP.addTask("tra", new TaskMap_Transition(*world_robot), sumOfSqrTT);
  ((TaskMap_Transition*)&t->map)->H_rate_diag = pr2_reasonable_W(*world_robot);
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e0);

  t =MP.addTask("posT", new TaskMap_qItself(), eqTT);
  t->setCostSpecs(MP.T-2,MP.T, x_robot, 1e0);

  arr X_robot = MP.getInitialization();
  X_robot.reshape(MP.T,world_robot->getJointStateDimension());
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrained(X_robot, NoArr, Convert(MP.komo_problem), o);

  executeTrajectory(X_robot, T, recordData, displayTraj);
}


void TrajectoryInterface::recordDemonstration(arr &X_robot,double T,double dt,double T_start) {
  mlr::wait(T_start);

  /// send zero gains
  CtrlMsg refs_zero;
  refs_zero.q = S->ctrl_obs.get()->q;
  refs_zero.qdot=S->ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.fR = zeros(6);
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
  uint idx = world_robot->getJointByName("head_pan_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;
  idx = world_robot->getJointByName("head_tilt_joint")->qIndex;
  refs_zero.Kp(idx,idx) = 2.0;

  S->ctrl_ref.set() = refs_zero;

  mlr::wait(3.);
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "START RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// record demonstrations
  double t = 0.;
  X_robot.clear();
  mlr::timerStart(true);
  while(t<T) {
    arr q = S->ctrl_obs.get()->q;
    X_robot.append(~q);
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

void TrajectoryInterface::syncMarker() {
  if (useRos && useMarker) {
    markers = S->ar_pose_markers.get();
    syncMarkers(*world_robot,markers);
    syncMarkers(*world_plan,markers);
  }
}

void TrajectoryInterface::syncState() {
  arr q_robot,q_plan;
  if (useRos) {
    q_robot = S->ctrl_obs.get()->q;
  } else {
    q_robot = world_robot->getJointState();
  }
  transferQbetweenTwoWorlds(q_plan,q_robot,*world_plan,*world_robot);
  world_robot->setJointState(q_robot);
  world_plan->setJointState(q_plan);

  /// sync torso
  if (world_plan->getJointByName("torso_lift_joint")->type==ors::JT_rigid) {
    world_plan->getJointByName("torso_lift_joint")->A = world_robot->getJointByName("torso_lift_joint")->A;
    world_plan->getJointByName("torso_lift_joint")->Q.pos = world_robot->getJointByName("torso_lift_joint")->Q.pos;

    world_plan->calc_q_from_Q();
    world_plan->calc_fwdPropagateFrames();
  }
}

void TrajectoryInterface::moveLeftGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_robot->getJointByName("l_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2,3.);
}

void TrajectoryInterface::moveRightGripper(double d) {
  arr q_pr2;
  getState(q_pr2);
  q_pr2(world_robot->getJointByName("r_gripper_joint")->qIndex) = d;
  gotoPosition(q_pr2,3.);
}

void TrajectoryInterface::saveState(mlr::String filename) {
  arr q_robot;
  getState(q_robot);
  write(LIST<arr>(q_robot),filename);
}

void TrajectoryInterface::pauseMotion(bool sendZeroGains) {
  cout << "stopping motion" << endl;

  /// send zero gains
  CtrlMsg refs_zero = refs;
  refs_zero.q = S->ctrl_obs.get()->q;
  refs_zero.qdot=S->ctrl_obs.get()->qdot*0.;
  refs_zero.fL = zeros(6);
  refs_zero.fR = zeros(6);
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
  refs_zero.fR_gamma = 1.;
  S->ctrl_ref.set() = refs_zero;

  world_robot->watch(true,"press button to continue");
  cout << "continuing motion" << endl;

  /// reset gains
  refs.q = S->ctrl_obs.get()->q;
  refs.qdot=S->ctrl_obs.get()->qdot*0.;
  S->ctrl_ref.set() = refs;
}



void TrajectoryInterface::logging(mlr::String folder, mlr::String name, uint id) {
  mlr::String filename;
  if (id==0) {
    filename = mlr::String(STRING(folder<<"/"<<name<<"_"));
  }else {
    filename = mlr::String(STRING(folder<<"/"<<id<<"_"<<name<<"_"));
    write(LIST<arr>(ARR(id)),STRING(folder<<name<<"_id.dat"));
  }

  write(LIST<arr>(logT),STRING(filename<<"T.dat"));
  write(LIST<arr>(logXdes),STRING(filename<<"Xdes.dat"));
  write(LIST<arr>(logXref),STRING(filename<<"Xref.dat"));

  if (logX.N>0) write(LIST<arr>(logX),STRING(filename<<"X.dat"));
  if (logXplan.N>0) write(LIST<arr>(logXplan),STRING(filename<<"Xplan.dat"));
  if (logFL.N>0) write(LIST<arr>(logFL),STRING(filename<<"FL.dat"));
  if (logFR.N>0) write(LIST<arr>(logFR),STRING(filename<<"FR.dat"));
  if (useMarker) {
    for (uint i=0;i<logM.N;i++){
      if (logM(i).N>0) {
        write(LIST<arr>(logM(i)),STRING(filename<<"M"<<i<<".dat"));
      }
    }
  }

  if (logU.N>0) write(LIST<arr>(logU),STRING(filename<<"U.dat"));
}
