#include "motion_interface.h"
#include <Algo/spline.h>



Motion_Interface::Motion_Interface(mlr::KinematicWorld &world_)
{
  world = new mlr::KinematicWorld(world_);
  world->q = world_.q;
  threadOpenModules(true);

  useBase = false;
  useTorso = false;

  if (!useBase) {
    world->getJointByName("worldTranslationRotation")->H = 3000000;
    world->getJointByName("torso_lift_joint")->H = 3000000;
  }


  //-- wait for first q observation!
  cout <<"** Waiting for ROS message on initial configuration.." <<endl;
  for(;;){
    S.ctrl_obs.data->waitForNextRevision();
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

void Motion_Interface::setOdom(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose){
  mlr::Quaternion quat(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
  mlr::Vector pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);

  double angle;
  mlr::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = mlr::sign(rotvec(2)) * angle;
}

void Motion_Interface::executeTrajectory(arr &X, double T, bool recordData)
{
  mlr::wait(2.);

  double dt = T/X.d0;
  Xref = X;
  arr Xdot;
  getVel(Xdot,X,dt);
  write(LIST<arr>(Xdot),"Xdot");
  write(LIST<arr>(X),"X");

  mlr::Spline XS(X.d0,X);
  mlr::Spline XdotS(Xdot.d0,Xdot);

  if (recordData) {Tact.clear(); Xdes.clear(); Xact.clear(); FLact.clear(); Uact.clear(); Mact.clear(); }

  mlr::Joint *trans = world->getJointByName("worldTranslationRotation");
  mlr::Joint *torso = world->getJointByName("torso_lift_joint");

  arr q_real;
  q_real = S.ctrl_obs.get()->q;

  mlr::timerStart(true);
  CtrlMsg refs;
  double t = 0.;
  double t_last = -dt;

  while(t<T){
    double s = t/T;
    refs.q = XS.eval(s);
    refs.qdot=XdotS.eval(s);

    /// base
    if (useBase) {
      q_real = S.ctrl_obs.get()->q;
      setOdom(q_real, trans->qIndex, S.pr2_odom.get());
      double gain = 0.75;
      refs.qdot(trans->qIndex) += gain*(refs.q(trans->qIndex+0)-q_real(trans->qIndex+0));
      refs.qdot(trans->qIndex+1) += gain*(refs.q(trans->qIndex+1)-q_real(trans->qIndex+1));
      refs.qdot(trans->qIndex+2) += gain*(refs.q(trans->qIndex+2)-q_real(trans->qIndex+2));
    } else {
      refs.qdot(trans->qIndex) = 0.;
      refs.qdot(trans->qIndex+1) = 0.;
      refs.qdot(trans->qIndex+2) = 0.;
    }

    /// torso
    if (!useTorso) {
      refs.q(torso->qIndex) = q_real(torso->qIndex);
      refs.qdot(torso->qIndex) = 0.;
    }

    refs.fL = zeros(6);
    refs.KiFTL.clear();
    refs.J_ft_invL.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 2.0;
    refs.Kd = 1.2;
    refs.Ki = .5;
    refs.fL_gamma = 1.;
    refs.velLimitRatio = .2;
    refs.effLimitRatio = 1.;
    refs.intLimitRatio = 0.8;
    S.ctrl_ref.set() = refs;

    t = t + mlr::timerRead(true);

    if (t-t_last >= dt){
      t_last = t;

      if (recordData) {
        Tact.append(ARR(t));
        Xdes.append(~refs.q);
        Xact.append(~S.ctrl_obs.get()->q);
        FLact.append(~S.ctrl_obs.get()->fL);
        Uact.append(~S.ctrl_obs.get()->u_bias);

        markers = S.ar_pose_marker.get();
        syncMarkers(*world, markers);
        Mact.append(~conv_vec2arr(world->getBodyByName("marker4")->X.pos));
      }
    }
  }

}

void Motion_Interface::gotoPosition(arr x)
{
  KOMO MP(*world,false);
  MP.T = 100;
  MP.tau = 0.05;
  MP.x0 = S.ctrl_obs.get()->q;

  mlr::Joint *trans = world->getJointByName("worldTranslationRotation");
  if (useBase) { setOdom(MP.x0, trans->qIndex, S.pr2_odom.get()); }


  Task *t;
  t = MP.addTask("tra", new TaskMap_Transition(*world));
  ((TaskMap_Transition*)&t->map)->H_rate_diag = world->getHmetric();
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e0);

  t =MP.addTask("posT", new TaskMap_qItself());
  t->setCostSpecs(MP.T-2,MP.T, x, 1e2);

  MotionProblemFunction MPF(MP);
  arr X = MP.getInitialization();
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrained(X, NoArr, Convert(MPF), o);

  executeTrajectory(X,MP.T*MP.tau);
}

void Motion_Interface::recordDemonstration(arr &X,double T)
{
  mlr::wait(5.);

  /// send zero gains
  CtrlMsg refs;
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.fL = zeros(6);
  refs.KiFTL.clear();
  refs.J_ft_invL.clear();
  refs.u_bias = zeros(q.N);
  refs.Kp = zeros(q.N,q.N); // = 0.;
  refs.Kd = 0.;
  refs.Ki.clear();
  refs.fL_gamma = 1.;
  refs.velLimitRatio = .1;
  refs.effLimitRatio = 1.;
  refs.intLimitRatio = 1.5;

  // fix head joint
  uint idx = world->getJointByName("head_pan_joint")->qIndex;
  refs.Kp(idx,idx) = 2.0;
  idx = world->getJointByName("head_tilt_joint")->qIndex;
  refs.Kp(idx,idx) = 2.0;


  S.ctrl_ref.set() = refs;



  mlr::wait(3.);
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "START RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  mlr::Joint *trans = world->getJointByName("worldTranslationRotation");

  /// record demonstrations
  double t = 0.;
  mlr::timerStart(true);
  while(t<T) {
    arr q = S.ctrl_obs.get()->q;
    if (useBase) {setOdom(q, trans->qIndex, S.pr2_odom.get());}
    X.append(~q);
    mlr::wait(0.1);
    t = t + mlr::timerRead(true);
  }
  cout << "//////////////////////////////////////////////////////////////////" << endl;
  cout << "STOP RECORDING" << endl;
  cout << "//////////////////////////////////////////////////////////////////" << endl;

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp.clear();
  refs.Kp = ARR(2.0);
  refs.Kd = 1.2;
  refs.Ki = ARR(.3);
  S.ctrl_ref.set() = refs;
}

void Motion_Interface::stopMotion(bool sendZeroGains)
{
  cout << "stopping motion" << endl;

  /// send zero gains
  CtrlMsg refs;
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.fL = zeros(6);
  refs.KiFTL.clear();
  refs.J_ft_invL.clear();
  refs.u_bias = zeros(q.N);
  if (sendZeroGains) {
    cout << "sending zero gains" << endl;
    refs.Kp = 0.;
    refs.Kd = 0.;
    refs.Ki.clear();
  } else {
    refs.Kp = ARR(2.0);
    refs.Kd =  ARR(1.2);
    refs.Ki = ARR(.3);
  }
  refs.fL_gamma = 1.;
  refs.velLimitRatio = .1;
  refs.effLimitRatio = 1.;
  refs.intLimitRatio = 1.;
  S.ctrl_ref.set() = refs;

  world->watch(true,"press button to continue");

  /// reset gains
  refs.q = S.ctrl_obs.get()->q;
  refs.qdot=S.ctrl_obs.get()->qdot*0.;
  refs.Kp = ARR(2.0);
  refs.Kd =  ARR(1.2);
  refs.Ki = ARR(.3);
  S.ctrl_ref.set() = refs;
  cout << "continuing motion" << endl;
}

/// save last run
void Motion_Interface::logging(mlr::String folder, uint id) {
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
