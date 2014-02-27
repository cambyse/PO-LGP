#include "traj_optimizer.h"


TrajOptimizer::TrajOptimizer(ors::KinematicWorld &_world)
{
  world = _world;
}

void TrajOptimizer::optimizeTrajectory(arr &_goal, arr &_q0) {
  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  // reference goal of right endeffector
  arr refGoal = _goal; //ARRAY(MP.world.getBodyByName("goalRef")->X.pos);

  // reference frame for planning and execution
  arr refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);

  //-- set qLimits of joints
  arr qLimits = { -130, 40,   //30 r_shoulder_pan_joint
                  -30, 80,    //31 r_shoulder_lift_joint
                  -224, 44,   //32 r_upper_arm_roll_joint
                  -133, 0,    //33 r_elbow_flex_joint
                  -360, 360,  //34 r_forearm_roll_joint
                  -130, 0,    //35 r_wrist_flex_joint
                  -360, 360}; //36 r_wrist_roll_joint

  qLimits = M_PI*qLimits/180.;
  qLimits.reshape(7,2);

  //-- create tasks for optimization problem
  TaskCost *c;
  c = MP.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeffR", ors::Vector(0., 0., 0.)));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, refGoal, 1e5);
  MP.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  c = MP.addTaskMap("qLimits", new DefaultTaskMap(qLimitsTMT,world));
  MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e2);
  MP.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e2);

  c = MP.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),0);
  MP.setInterpolatingVelCosts(c,MotionProblem::constant,ARRAY(0.),1e0);


//
//  c = MP.addTaskMap("position", new DefaultTaskMap(posTMT,world,"endeff", ors::Vector(0., 0., 0.)));
//  MP.setInterpolatingCosts(c, MotionProblem::finalOnly,refGoal, 1e1,ARRAY(0.,0.,0.), 1e-4);
//  MP.setInterpolatingVelCosts(c, MotionProblem::finalOnly,ARRAY(0.,0.,0.), 1e1,ARRAY(0.,0.,0.), 0.);

//  c = MP.addTaskMap("qlimits", new DefaultTaskMap(qLimitsTMT,world, NULL, NoVector, NULL, NoVector, qLimits));
//  MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e2);

//  c = MP.addTaskMap("homing", new DefaultTaskMap(qItselfTMT,world, NULL, NoVector, NULL, NoVector));
//  MP.setInterpolatingCosts(c,MotionProblem::constant,ARRAY(0.),1e-3);

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = _q0;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();
  optNewton(x, Convert(MPF), OPT(verbose=0, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));
  MP.costReport(false);

  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPos, xRefVec;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeffR")->index);
    world.kinematicsVec(kinVec,NoArr,MP.world.getBodyByName("endeffR")->index);
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  refPlan = ~cat(~xRefPos,~xRefVec);
  TRef = dt*T;

  displayTrajectory(x,1,world,"reference trajectory");
}
