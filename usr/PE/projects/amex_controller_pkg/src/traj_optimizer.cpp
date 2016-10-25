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
  arr refGoal = _goal; //conv_vec2arr(MP.world.getBodyByName("goalRef")->X.pos);

  // reference frame for planning and execution
  arr refFrame = conv_vec2arr(world.getBodyByName("torso_lift_link")->X.pos);

  //-- create tasks for optimization problem
  Task *c;
  c = MP.addTask("position_right_hand", new TaskMap_Default(posTMT,world,"endeffR", ors::Vector(0., 0., 0.)));
  c->setCostSpecs(MP.T, MP.T, refGoal, 1e5);

//  c = MP.addTask("qLimits", new TaskMap_qLimits());
//  MP.setInterpolatingCosts(c,MotionProblem::constant,{0.},1e0,{0.},1e0);

  c = MP.addTask("final_vel", new TaskMap_qItself());
  MP.setInterpolatingCosts(c,MotionProblem::finalOnly,{0.},1e3);
  c->map.order=1;

  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = _q0;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  arr x(T+1,n); x.setZero();
  optNewton(x, Convert(MPF), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));
  MP.costReport(true);

  //-- Transform trajectory into task space
  arr kinPos, kinVec, xRefPos, xRefVec;
  // store cartesian coordinates and endeffector orientation
  for (uint t=0;t<=T;t++) {
    world.setJointState(x[t]);
    world.kinematicsPos(kinPos,NoArr,MP.world.getBodyByName("endeffR"));
    world.kinematicsVec(kinVec,NoArr,MP.world.getBodyByName("endeffR"));
    xRefPos.append(~kinPos);
    xRefVec.append(~kinVec);
  }

  refPlan = ~cat(~xRefPos,~xRefVec);
  TRef = dt*T;

  displayTrajectory(x,1,world,"reference trajectory");
}
