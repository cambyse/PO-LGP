#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Core/array-vector.h>
#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Algo/spline.h>

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, marker_pose);
  MySystem(){
    if(MT::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module_Thread::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module_Thread::listenFirst);
      addModule<RosCom_ARMarkerSync>(NULL, Module_Thread::loopWithBeat, 1.);
    }
    connect();
  }
};

void changeColor(void*){  orsDrawAlpha = .7; }
void changeColor2(void*){  orsDrawAlpha = 1.; }

void planTrajHandle(arr &x,ors::KinematicWorld &world) {
  MotionProblem MP(world,false);

  /// load parameter from file
  double costScale = 1e2;

  arr param = ARRAY(.1,1.281488,5.7258,5.7258,3.50189,0.749742);
  param.subRange(1,param.d0-1) = param.subRange(1,param.d0-1)/length(param.subRange(1,param.d0-1))*costScale;

  uint pC = 0;
  // transition costs
  Task *t;
  t =MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0,MP.T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  ((TransitionTaskMap*)&t->map)->H_rate_diag.subRange(0,2)=100.;
  pC++;

  // time points
  uint P = 60;
  uint C = 95;
  uint U = 157;
  uint F = MP.T;

  /// tasks
  // contact with handle
  t =MP.addTask("posC", new DefaultTaskMap(posTMT, world, "endeffL",NoVector));
  t->setCostSpecs(P, P, ARRAY(world.getShapeByName("target1")->X.pos), param(pC));
  pC++;

  t = MP.addTask("vecC", new DefaultTaskMap(vecAlignTMT, world,"endeffL",ors::Vector(1.,0.,0.),"handle",ors::Vector(-1.,0.,0.)));
  t->setCostSpecs(C, U, ARRAY(1.), param(pC));
  pC++;

  t = MP.addTask("vecC2", new DefaultTaskMap(vecAlignTMT, world,"endeffL",ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(P, P, ARRAY(1.), param(pC));
  pC++;

////  ors::Vector vecTarget = ors::Vector(0., 0.9939, -0.1104);
  ors::Vector vecTarget = ors::Vector(0., 1.,0.);
  vecTarget.normalize();
  t =MP.addTask("vecUF", new DefaultTaskMap(vecAlignTMT, world, "endeffL",ors::Vector(0.,1.,0.),"handle",vecTarget));
  t->setCostSpecs(U, F, ARRAY(1.), param(pC));
  pC++;

  t = MP.addTask("door_joint", new TaskMap_qItself(world.getJointByName("frame_door")->qIndex, world.getJointStateDimension()));
  t->setCostSpecs(F-10, F, ARRAY(-.47), param(pC));
//  t->setCostSpecs(F-10, F, ARRAY(-.37), param(pC));
//  t->setCostSpecs(F-10, F, ARRAY(-.27), param(pC));
//  t->setCostSpecs(F-10, F, ARRAY(-.17), param(pC));
//    t->setCostSpecs(F-10, F, ARRAY(-.57), param(pC));
//    t->setCostSpecs(F-10, F, ARRAY(-.67), param(pC));
//    t->setCostSpecs(F-10, F, ARRAY(-.77), param(pC));
  pC++;


  /// constraints
  t =MP.addTask("torso_fixation", new qItselfConstraint(world.getJointByName("torso_lift_joint")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(0,F, {0.}, 1.);
  t->map.order=1;


  t =MP.addTask("door_fixation", new qItselfConstraint(world.getJointByName("frame_door")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(0,U, {0.}, 1.);
  t->map.order=1;

  t = MP.addTask("contact", new PointEqualityConstraint(world, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(C, F, {0.}, 1.);


//  // q limit constraint
//  t = MP.addTask("qLimits", new LimitsConstraint());
//  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt =MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr lambda(T+1,1); lambda.setZero();
  x = repmat(~MP.x0,T+1,1);
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=1, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-3));

  MP.costReport(true);
  MP.world.watch(true);
  displayTrajectory(x,MP.T*10.,MP.world,"world");

  // visualization code
}

void planTrajInit(arr &x,ors::KinematicWorld &world) {
  MotionProblem MP(world,false);

  /// load parameter from file
  double costScale = 1e2;

  uint pC = 0;
  // transition costs
  Task *t;
  t =MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0,MP.T, ARR(0.), 1e0);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  pC++;

  // time points
  uint F =MP.T;

  arr q0 = ARRAY(0.250694, 0., 1.04935, 0.362997, 1.45626, -1.64196, -2.68619, -0.456745, 0.0659161);

  /// tasks
  // contact with handle
  t =MP.addTask("q", new TaskMap_qItself());
  t->setCostSpecs(F-5, F, q0, 1e1);
  pC++;

  /// constraints
  // q limit constraint
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, {0.}, 1.);

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt =MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr lambda(T+1,1); lambda.setZero();
  x = repmat(~MP.x0,T+1,1);
  optConstrainedMix(x, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2.,stopTolerance = 1e-2));

  MP.costReport(true);
  displayTrajectory(x,MP.T,MP.world,"world");
}

void transPlanPR2(MT::Array<MT::String> &active_joints, ors::KinematicWorld &w_plan, ors::KinematicWorld &w_pr2, const arr &q_plan, arr &q_pr2) {
  for (uint i = 0; i<active_joints.d0;i++){
    for (uint k = 0; k<w_plan.getJointByName(active_joints(i))->qDim(); k++){
      uint planIdx = w_plan.getJointByName(active_joints(i))->qIndex+k;
      uint pr2Idx = w_pr2.getJointByName(active_joints(i))->qIndex+k;
      q_pr2(pr2Idx) = q_plan(planIdx);
    }
  }
}

void transPR2Plan(MT::Array<MT::String> &act_joints, ors::KinematicWorld &w_pr2, ors::KinematicWorld &w_plan, const arr &q_pr2, arr &q_plan) {
  for (uint i = 0; i<act_joints.d0;i++){
    uint pr2Idx = w_pr2.getJointByName(act_joints(i))->qIndex;
    uint planIdx = w_plan.getJointByName(act_joints(i))->qIndex;
    q_plan(planIdx) = q_pr2(pr2Idx);
  }
}

void syncMarker(ors::KinematicWorld &world, arr &marker_pose){
  arr refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
  world.getBodyByName("marker4")->X.pos = refFrame + marker_pose[4].subRange(0,2);
  world.getBodyByName("marker11")->X.pos = refFrame + marker_pose[11].subRange(0,2);
  world.getBodyByName("marker15")->X.pos = refFrame + marker_pose[15].subRange(0,2);
  world.getBodyByName("marker17")->X.pos = refFrame + marker_pose[17].subRange(0,2);

  // compute door angle
  arr v1 = ARRAY(world.getBodyByName("marker4")->X.pos - world.getBodyByName("marker15")->X.pos);
  v1(2)=0.;
  v1 = v1/sqrt(sumOfSqr(v1));
  arr v2 = ARRAY(1.,0.,0.);
  double alpha = acos(sum(v1%v2));

  world.getJointByName("world_door")->A.rot.setRadZ(M_PI_2-alpha);
  world.getJointByName("world_door")->A.pos = ARRAY(world.getBodyByName("marker15")->X.pos)+ world.getJointByName("world_door")->A.rot.getArr()*ARRAY(0.,0.6205,0.2305);
  world.getJointByName("world_door")->A.pos.z = .99;

  world.calc_fwdPropagateFrames();
}





void TEST(Marker){
  MySystem S;
  engine().open(S);
  ors::KinematicWorld world_plan("model_reduced.kvg");
  ors::KinematicWorld world_pr2("model.kvg");

  arr q,qdot; // joints states of pr2 world
  arr qP,qPdot; // joints states of planned world
  arr marker_pose = zeros(20,7);

  world_pr2.getJointState(q,qdot);
  world_plan.getJointState(qP,qPdot);

  /// set list of active joints for remapping between world_pr2 and world_plan
  MT::Array<MT::String> active_joints;
  for (uint i = 0;i<world_plan.joints.d0;i++) {
    if (world_plan.joints(i)->type != 10 && world_plan.joints(i)->name!="frame_door") {
      active_joints.append(world_plan.joints(i)->name);
      cout << world_plan.joints(i)->name << " " << world_plan.joints(i)->type << " "  << world_plan.joints(i)->qIndex<< endl;
    }
  }

  /// read initial robot position and marker position
  bool useRos = MT::getParameter<bool>("useRos", false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial joints.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==world_pr2.q.N &&S.ctrl_obs.get()->qdot.N==world_pr2.q.N)
        break;
    }
    //-- set current state
    q =S.ctrl_obs.get()->q;
    qdot =S.ctrl_obs.get()->qdot;

    world_pr2.setJointState(q);
    transPR2Plan(active_joints,world_pr2,world_plan,q,qP);
    world_plan.setJointState(qP);

    //-- wait for first marker observation
    cout <<"** Waiting for ROS message on initial markers.." <<endl;
    for(;;){
//      S.marker_pose.var->waitForNextRevision();
      S.marker_pose.var->waitForRevisionGreaterThan(10);
      if(S.marker_pose.get()->N > 3)
        break;
    }

    //-- set marker state
    marker_pose = S.marker_pose.get();
    syncMarker(world_plan,marker_pose);

    world_pr2.watch(false);
    world_plan.watch(false);
  }

  /// create trajectory
  arr x,xd;
  double duration = MT::getParameter<double>("duration");
  planTrajHandle(x,world_plan);
  double tau = duration/x.d0;
  getVel(xd,x,tau);

  arr xP,xdP;
  for (uint i = 0;i<x.d0;i++){
    arr q_tmp = q;
    arr qd_tmp = qdot;
    transPlanPR2(active_joints,world_plan,world_pr2,x[i],q_tmp);
    transPlanPR2(active_joints,world_plan,world_pr2,xd[i],qd_tmp);
    xP.append(~q_tmp);
    xdP.append(~qd_tmp);
  }

  MT::Spline xs(x.d0,xP);
  MT::Spline xds(x.d0,xdP);

  /// start motion execution
  while(true){
    //-- sync marker
    S.marker_pose.var->waitForNextRevision();
    marker_pose = S.marker_pose.get();
    syncMarker(world_plan, marker_pose);

    //-- sync robot
    S.ctrl_obs.var->waitForNextRevision();
    q =S.ctrl_obs.get()->q;
    qdot =S.ctrl_obs.get()->qdot;

    world_pr2.setJointState(q);
    transPR2Plan(active_joints,world_pr2,world_plan,q,qP);
    world_plan.setJointState(qP);

    world_plan.gl().update();
  }
  world_plan.watch(true);
}

void TEST(ReplayDemo){
  ors::KinematicWorld world_plan("model_reduced.kvg");
  ors::KinematicWorld world_pr2("model.kvg");

  arr q,qdot; // joints states of pr2 world
  arr qP,qPdot; // joints states of planned world
  arr marker_pose = zeros(20,7);

  world_pr2.getJointState(q,qdot);
  world_plan.getJointState(qP,qPdot);

  /// set list of active joints for remapping between world_pr2 and world_plan
  MT::Array<MT::String> active_joints;
  for (uint i = 0;i<world_plan.joints.d0;i++) {
    if (world_plan.joints(i)->type != 10 && world_plan.joints(i)->name!="frame_door") {
      active_joints.append(world_plan.joints(i)->name);
      cout << world_plan.joints(i)->name << " " << world_plan.joints(i)->type << " "  << world_plan.joints(i)->qIndex<< endl;
    }
  }

  cout << "###########################################################" << endl;
  /// load demonstrations data
  MT::String demoPath = MT::getParameter<MT::String>("demoPath");
  cout << "Demo: " << demoPath << endl;
  // load joint trajectory
  arr xDem; xDem << FILE(STRING(demoPath<<"/pr2_joints").p);

  arr markerDem; markerDem << FILE(STRING(demoPath<<"/pr2_marker0").p);
  marker_pose[4].subRange(0,2) = markerDem[0];
  marker_pose[11].subRange(0,2) = markerDem[1];
  marker_pose[15].subRange(0,2) = markerDem[2];
  marker_pose[17].subRange(0,2) = markerDem[3];

  //-- set door initial position
  syncMarker(world_plan,marker_pose);

  //-- replay trajectory
  for (uint t =0;t<xDem.d0;t++){
    world_plan.setJointState(xDem[t]);
    world_plan.gl().update(STRING(t));
    MT::wait(0.1);
  }
  return;
}

void run(){
  MySystem S;
  engine().open(S);
  ors::KinematicWorld world_plan("model_reduced.kvg");
  ors::KinematicWorld world_pr2("model.kvg");

  arr q,qdot; // joints states of pr2 world
  arr qP,qPdot; // joints states of planned world
  arr marker_pose = zeros(20,7);

  world_pr2.getJointState(q,qdot);
  world_plan.getJointState(qP,qPdot);

  /// set list of active joints for remapping between world_pr2 and world_plan
  MT::Array<MT::String> active_joints;
  for (uint i = 0;i<world_plan.joints.d0;i++) {
    if (world_plan.joints(i)->type != 10 && world_plan.joints(i)->name!="frame_door") {
      active_joints.append(world_plan.joints(i)->name);
      cout << world_plan.joints(i)->name << " " << world_plan.joints(i)->type << " "  << world_plan.joints(i)->qIndex<< endl;
    }
  }

  /// read initial robot position and marker position
  bool useRos = MT::getParameter<bool>("useRos", false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial joints.." <<endl;
    for(;;){
      S.ctrl_obs.var->waitForNextRevision();
      if(S.ctrl_obs.get()->q.N==world_pr2.q.N &&S.ctrl_obs.get()->qdot.N==world_pr2.q.N)
        break;
    }
    //-- set current state
    q =S.ctrl_obs.get()->q;
    qdot =S.ctrl_obs.get()->qdot;

    world_pr2.setJointState(q);
    transPR2Plan(active_joints,world_pr2,world_plan,q,qP);
    world_plan.setJointState(qP);

    cout << "qP "<< qP << endl;

    //-- wait for first marker observation
    cout <<"** Waiting for ROS message on initial markers.." <<endl;
    for(;;){
      S.marker_pose.var->waitForRevisionGreaterThan(10);
      if(S.marker_pose.get()->N > 3)
        break;
    }

    //-- set marker state
    marker_pose = S.marker_pose.get();
    syncMarker(world_plan,marker_pose);

  }else{
    q = ARRAY(0.250694, 0., 1.04935, 0.362997, 1.45626, -1.64196, -2.68619, -0.456745, 0.0659161);
    marker_pose[4].subRange(0,2) = ARR(1.019, 0.30718, 0.57917);
    marker_pose[11].subRange(0,2) = ARR(1.1834, -0.051653, 0.26799);
    marker_pose[15].subRange(0,2) = ARR(0.9855, 0.42244, 0.27362);
    marker_pose[17].subRange(0,2) = ARR(1.1376, 0.066198, 0.58049);
//        world.getBodyByName("marker4")->X.pos = refFrame + marker_pose[4].subRange(0,2);
//        world.getBodyByName("marker11")->X.pos = refFrame + marker_pose[11].subRange(0,2);
//        world.getBodyByName("marker15")->X.pos = refFrame + marker_pose[15].subRange(0,2);
//        world.getBodyByName("marker17")->X.pos = refFrame + marker_pose[17].subRange(0,2);
    world_plan.setJointState(q);
    syncMarker(world_plan,marker_pose);
  }
//  world_pr2.watch(false);
  world_plan.watch(true);

  /// create trajectory
  arr x,xd;
  double duration = MT::getParameter<double>("duration");
  bool optGotoInitPos = MT::getParameter<bool>("optGotoInitPos");

  if (optGotoInitPos) {
    planTrajInit(x,world_plan);
  } else {
    planTrajHandle(x,world_plan);
  }

  double tau = duration/x.d0;
  getVel(xd,x,tau);

  arr xP,xdP;
  for (uint i = 0;i<x.d0;i++){
    arr q_tmp = q;
    arr qd_tmp = qdot;
    transPlanPR2(active_joints,world_plan,world_pr2,x[i],q_tmp);
    transPlanPR2(active_joints,world_plan,world_pr2,xd[i],qd_tmp);
    xP.append(~q_tmp);
    xdP.append(~qd_tmp);
  }

  MT::Spline xs(x.d0,xP);
  MT::Spline xds(x.d0,xdP);


  ors::Joint *trans=world_pr2.getJointByName("worldTranslationRotation");

  /// start motion execution
  cout <<"** Start Motion with" <<endl;
  cout <<"- Duration: " << duration << endl;
  cout <<"- tau: " << tau << endl;
  cout <<"<< Press button to start motion >>" << endl;
  world_plan.watch(true);
  double s = 0.;
  double t = 0.;
  MT::timerStart(true);


  while(s<1.){
    cout <<"t: "<< t <<endl;
    CtrlMsg refs;
    refs.fL = ARR(0., 0., 0.,0.,0.,0.);
    refs.KfL_gainFactor.clear();
    refs.EfL.clear();
    refs.u_bias = zeros(q.N);
    refs.Kq_gainFactor = 1.;
    refs.Kd_gainFactor = 1.;
    refs.gamma = 1.;

    s = t/duration;
    refs.q=xs.eval(s);
    refs.qdot=xds.eval(s)*0.;


//    arr qfb =S.ctrl_obs.get()->q;
//    arr qdref = xds.eval(s);
//    cout << refs.q(trans->qIndex+0) - qfb(trans->qIndex+0) << endl;
//    cout << refs.q(trans->qIndex+1) - qfb(trans->qIndex+1) << endl;
//    cout << refs.q(trans->qIndex+1) - qfb(trans->qIndex+1) << endl;
//    refs.qdot(trans->qIndex+0) = qdref(trans->qIndex+0) + (refs.q(trans->qIndex+0) - qfb(trans->qIndex+0));
//    refs.qdot(trans->qIndex+1) = qdref(trans->qIndex+1) + (refs.q(trans->qIndex+1) - qfb(trans->qIndex+1));
//    refs.qdot(trans->qIndex+2) = qdref(trans->qIndex+2) + (refs.q(trans->qIndex+2) - qfb(trans->qIndex+2));

    refs.velLimitRatio = .5;
    refs.effLimitRatio = 1.;

    S.ctrl_ref.set() = refs;
    S.step();
    world_pr2.setJointState(refs.q);
    world_pr2.gl().update();

    t = t + MT::timerRead(true);
  }

  engine().close(S);
}

int main(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  run();
//  testMarker();
//  testReplayDemo();
  return 0;
}
