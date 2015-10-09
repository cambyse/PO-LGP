#include <Motion/feedbackControl.h>
#include <Hardware/gamepad/gamepad.h>
#include <System/engine.h>
#include <Gui/opengl.h>
#include <Motion/pr2_heuristics.h>
#include <pr2/roscom.h>
#include <Motion/motion.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Algo/spline.h>

struct MySystem:System{
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
  ACCESS(arr, marker_pose);
  MySystem(){
    if(mlr::getParameter<bool>("useRos", false)){
      addModule<RosCom_Spinner>(NULL, Module::loopWithBeat, .001);
      addModule<RosCom_ControllerSync>(NULL, Module::listenFirst);
      addModule<RosCom_ARMarkerSync>(NULL, Module::loopWithBeat, 1.);
    }
    connect();
  }
};

void changeColor(void*){  orsDrawAlpha = .7; }
void changeColor2(void*){  orsDrawAlpha = 1.; }

void planTrajectory(arr &x,ors::KinematicWorld &world) {
  MotionProblem MP(world,false);

  /// load parameter from file

  double costScale = 1e2;
//  arr param = {.1,1e2,1e2,1e2,1e2,1e2,1e1};
  arr param = {0.22899,44.0754,0.115977,0.447064, 0.0262478, 63.4698 ,63.4825};

  param = param/length(param)*costScale;
  uint pC = 0;
  // transition costs
  Task *t;
  t =MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0,MP.T, ARR(0.), param(pC));
  ((TransitionTaskMap*)&t->map)->H_rate_diag = 1.;
  pC++;

  // time points
  uint C = 90;
  uint U = 140;
  uint F =MP.T;

  /// tasks
  // first contact with door
  t =MP.addTask("posC", new DefaultTaskMap(posTMT, world, "endeffL",NoVector));
  t->setCostSpecs(C, C, conv_vec2arr(world.getShapeByName("handle")->X.pos), param(pC));
  pC++;

  t =MP.addTask("vecC", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C, C, ARR(1.), param(pC));
  pC++;

  t =MP.addTask("vecC2", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C-10, C-10, ARR(1.), param(pC));
  pC++;

  t =MP.addTask("vecC3", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(0.,1.,0.),"handle",ors::Vector(0.,0.,1.)));
  t->setCostSpecs(C+10, C+10, ARR(1.), param(pC));
  pC++;

  ors::Vector dir = ors::Vector(0.,-.7,0.2); dir.normalize();
  t =MP.addTask("vecF", new DefaultTaskMap(vecAlignTMT, world, "endeffL", ors::Vector(0.,1.,0.),"handle",dir));
  t->setCostSpecs(U, U, ARR(1.), param(pC));
  pC++;

  // door final position
  t =MP.addTask("door_joint", new TaskMap_qItself(world.getJointByName("frame_door")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(F, F, ARR(-0.65), param(pC));
  pC++;

  /// constraints
  // hold contact endeffector-handle
  t =MP.addTask("contact", new PointEqualityConstraint(world, "endeffL",NoVector, "target",NoVector));
  t->setCostSpecs(U+1, F, {0.}, 1.);
  // door fixation
  t =MP.addTask("door_fixation", new qItselfConstraint(world.getJointByName("frame_door")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(0,U-1, {0.}, 1.);
  t->map.order=1;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T(); uint k=MPF.get_k(); uint n=MPF.dim_x(); double dt =MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<endl;
  arr lambda(T+1,1); lambda.setZero();
  x = repmat(~MP.x0,T+1,1);
  optConstrainedMix(x, lambda, Convert(MPF), OPT(verbose=1,stopTolerance=1e-4));

  displayTrajectory(x,MP.T,MP.world,"world");
  displayTrajectory(x,MP.T,MP.world,"world");

}


void initDoor(ors::KinematicWorld &world, arr &marker_pose){
  arr doorMarker = marker_pose[4];
  arr doorMarkerPos = doorMarker.subRange(0,2);
  ors::Quaternion doorMarkerQuat = ors::Quaternion(doorMarker.subRange(3,6));

  arr wallMarker = marker_pose[17];
  arr wallMarkerPos = wallMarker.subRange(0,2);
  ors::Quaternion wallMarkerQuat = ors::Quaternion(wallMarker.subRange(3,6));

  arr refFrame = conv_vec2arr(world.getBodyByName("torso_lift_link")->X.pos);

  ors::Quaternion door_rot = ors::Quaternion(0,1,0,0);//doorMarkerQuat;//ors::Quaternion(markerQuat0[1]);
  ors::Quaternion trans = world.getBodyByName("torso_lift_link")->X.rot;
//  trans.z=0.; trans.normalize();
//  arr tmp = ~trans.getArr();
//  cout << tmp << endl;
//  trans.setDiff(tmp[2],ors::Vector(0.,0.,1));
//  door_rot = trans*door_rot;
  trans.setRad(-M_PI,door_rot.getZ());
  door_rot = trans*door_rot;
//  trans.setRad(M_PI_2,ors::Vector(0.,0.,1.));
//  door_rot = trans*door_rot;
//  trans.setRad(M_PI_2,door_rot.getY());
//  door_rot = trans*door_rot;
//  trans.setRad(M_PI_2,door_rot.getZ());
//  door_rot = trans*door_rot;
//  trans.setRad(M_PI_2,door_rot.getZ());
//  door_rot = trans*door_rot;

  world.getJointByName("world_door")->A.pos = refFrame + doorMarkerPos + door_rot.getArr()*ARR(0.015,0.375*2,0.);
  world.getJointByName("world_door")->A.pos.z = .99;
  world.getJointByName("world_door")->A.rot = door_rot;

  world.getBodyByName("marker17")->X.pos = doorMarkerPos+refFrame;
  world.getBodyByName("marker17")->X.rot = door_rot;
  world.getBodyByName("marker4")->X.pos = wallMarkerPos+refFrame;
  world.getBodyByName("marker4")->X.rot = door_rot;

  world.calc_fwdPropagateFrames();
  world.calc_fwdPropagateShapeFrames();
}

void transPlanPR2(mlr::Array<mlr::String> &active_joints, ors::KinematicWorld &w_plan, ors::KinematicWorld &w_pr2, const arr &q_plan, arr &q_pr2) {
  for (uint i = 0; i<active_joints.d0;i++){
    uint planIdx = w_plan.getJointByName(active_joints(i))->qIndex;
    uint pr2Idx = w_pr2.getJointByName(active_joints(i))->qIndex;
    q_pr2(pr2Idx) = q_plan(planIdx);
  }
}

void transPR2Plan(mlr::Array<mlr::String> &act_joints, ors::KinematicWorld &w_pr2, ors::KinematicWorld &w_plan, const arr &q_pr2, arr &q_plan) {
  for (uint i = 0; i<act_joints.d0;i++){
    uint pr2Idx = w_pr2.getJointByName(act_joints(i))->qIndex;
    uint planIdx = w_plan.getJointByName(act_joints(i))->qIndex;
    q_plan(planIdx) = q_pr2(pr2Idx);
  }
}

void run(){
  MySystem S;
  engine().open(S);
  ors::KinematicWorld world_plan("model_reduced.kvg");
  ors::KinematicWorld world_pr2("model.kvg");

  // set list of active joints for remapping between pr2 and plan KinematicWorlds
  mlr::Array<mlr::String> active_joints;
  for (uint i = 0;i<world_plan.joints.d0;i++) {
    if (world_plan.joints(i)->type != 10 && world_plan.joints(i)->name!="frame_door") {
      active_joints.append(world_plan.joints(i)->name);
      cout << world_plan.joints(i)->name << " " << world_plan.joints(i)->type << " "  << world_plan.joints(i)->qIndex<< endl;
    }
  }

  arr q,qdot; // joints states of pr2 world
  arr qP,qPdot; // joints states of planned world
  arr marker_pose = zeros(20,7);

  world_pr2.getJointState(q,qdot);
  world_plan.getJointState(qP,qPdot);

  /// read initial robot position and marker position
  bool useRos = mlr::getParameter<bool>("useRos", false);
  if(useRos){
    //-- wait for first q observation!
    cout <<"** Waiting for ROS message on initial configuration.." <<endl;
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
    cout << world_plan.getJointByName("torso_lift_joint")->A.pos << endl;
    cout << world_pr2.getJointByName("torso_lift_joint")->A.pos << endl;
    world_plan.getJointByName("torso_lift_joint")->A.pos.z += q(world_pr2.getJointByName("torso_lift_joint")->qIndex);
    world_plan.getJointByName("torso_lift_joint")->A.pos.y += -0.0472;
    world_plan.getJointByName("torso_lift_joint")->A.pos.x += 0.0155;
    world_plan.calc_fwdPropagateFrames();

//    cout << world_plan.getBodyByName("torso_lift_joint")->X.pos << endl;
//    cout << world_pr2.getBodyByName("torso_lift_joint")->X.pos << endl;
//    world_plan.getJointByName("torso_lift_joint")->A.pos.z = world_pr2.getJointByName("torso_lift_joint")->A.pos.z;

    //-- wait for first marker observation
    for(;;){
      S.marker_pose.var->waitForNextRevision();
      if(S.marker_pose.get()->N > 0 /*&& S.marker_pose.get()->[4].length()>0 &&S.marker_pose.get()[17].length()>0*/)
        break;
    }
    //-- set marker state
    marker_pose = S.marker_pose.get();

    // for debugginig  (move door away)
//    marker_pose[4]= marker_pose[4] + ARR(-.1,0.,0.,0.,0.,0.,0.);
//    marker_pose[17]= marker_pose[17] + ARR(-.1,0.,0.,0.,0.,0.,0.);
  } else {
    qP = ARR(-0, 1.0951, 0.31445, 1.6866, -1.8677, -2.8285, -0.029944, -3.3155);
    marker_pose[4]=ARR(0.94048, -0.32071, 0.43706,-0.39946, 0.58953, 0.55908, -0.42459);
    marker_pose[17]=ARR(0.96383, -0.056259, 0.43725,-0.41846, 0.57182, 0.57531, -0.40854);
    world_plan.getJointByName("torso_lift_joint")->A.pos.z +=0.29398;
    world_plan.calc_fwdPropagateFrames();
    world_plan.setJointState(qP);
    transPlanPR2(active_joints,world_plan,world_pr2,qP,q);
    world_pr2.setJointState(q);
  }

  initDoor(world_plan,marker_pose);
  world_plan.watch(false);
  world_pr2.watch(true);

  /// plan trajectory
  arr x,xd;
  double duration = mlr::getParameter<double>("duration");
  planTrajectory(x,world_plan);
  double tau = duration/x.d0;
  getVel(xd,x,tau);

  arr xPR2,xdPR2;
  for (uint i = 0;i<x.d0;i++){
    arr q_tmp = q;
    arr qd_tmp = qdot;
    transPlanPR2(active_joints,world_plan,world_pr2,x[i],q_tmp);
    transPlanPR2(active_joints,world_plan,world_pr2,xd[i],qd_tmp);
    xPR2.append(~q_tmp);
    xdPR2.append(~qd_tmp);
  }

  cout << xPR2 << endl;
  cout << xdPR2 << endl;
  mlr::Spline xs(x.d0,xPR2);
  mlr::Spline xds(x.d0,xdPR2);

  /// execute trajectory on robot
  cout <<"** GO!" <<endl;
  cout <<"Duration: " << duration << endl;
  cout <<"tau: " << tau << endl;
  world_plan.watch(true);
  double s = 0.;
  double t = 0.;
  mlr::timerStart(true);
  while(s<1.){
    //compute control
    cout <<"t: "<< t <<endl;
    CtrlMsg refs;
    refs.fL = ARR(0., 0., 0.,0.,0.,0.);
    refs.KiFT.clear();
    refs.J_ft_inv.clear();
    refs.u_bias = zeros(q.N);
    refs.Kp = 1.;
    refs.Kd = 1.;
    refs.Ki = 0.;
    refs.gamma = 1.;

    s = t/duration;
    refs.q=xs.eval(s);
    refs.qdot=xds.eval(s)*0.;
    cout << refs.q << endl;

    refs.velLimitRatio = .1;
    refs.effLimitRatio = 1.;
    cout <<"ratios:" <<refs.velLimitRatio <<' ' <<refs.effLimitRatio <<endl;
    S.ctrl_ref.set() = refs;
    S.step();

    t = t + mlr::timerRead(true);
  }

  engine().close(S);
}


int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);
  run();
  return 0;
}
