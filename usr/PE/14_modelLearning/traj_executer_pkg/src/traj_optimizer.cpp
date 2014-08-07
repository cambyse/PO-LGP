#include "traj_optimizer.h"


TrajOptimizer::TrajOptimizer(ors::KinematicWorld &_world)
{
  world = _world;
}

void TrajOptimizer::optimizeTrajectory(arr &_goal, arr &_q0, arr &x) {
  // Create Motion Problem
  MotionProblem MP(world);
  MP.loadTransitionParameters();

  //-- create tasks for optimization problem
  TaskCost *c;

  c = MP.addTask("final_vel", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c,MotionProblem::finalOnly,ARRAY(0.),1e3);
  c->map.order=1;

  c = MP.addTask("position_right_hand", new DefaultTaskMap(qItselfTMT,world));
  MP.setInterpolatingCosts(c, MotionProblem::finalOnly, _goal, 1e5);

//  c = MP.addTask("limits", new DefaultTaskMap(qLimitsTMT,world));
//  MP.setInterpolatingCosts(c, MotionProblem::constant, ARR(0.), 1e5);
//  c->map.order=1;


  //-- create the Optimization problem (of type kOrderMarkov)
  MP.x0 = _q0;

  MotionProblemFunction MPF(MP);
  uint T=MPF.get_T();
  uint k=MPF.get_k();
  uint n=MPF.dim_x();
  double dt = MP.tau;
  cout <<"Problem parameters:"<<" T=" <<T<<" k=" <<k<<" n=" <<n << " dt=" << dt <<" # joints=" <<world.getJointStateDimension()<<endl;

  x = repmat(~MP.x0,T+1,1);
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-3, allowOverstep=true));
  optNewton(x, Convert(MPF), OPT(verbose=0,stopTolerance=1e-3, allowOverstep=true));
  MP.costReport(true);

  TRef = dt*T;

  displayTrajectory(x,300,world,"reference trajectory");
}

double fRand(double fMin, double fMax)
{
  double f = (double)rand() / RAND_MAX;
  return fMin + f * (fMax - fMin);
}

void TrajOptimizer::sampleGoal(arr &_goal,const arr &_q0)
{
  double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, _q0(4)-M_PI, -2. ,_q0(6)-M_PI};
  double qUpperLimit[7] = {0.714602, 0.6, 0.8, 0., _q0(4)+M_PI, 0., _q0(6)+M_PI};

//  arr limits = world.getLimits();
//  cout << limits << endl;
  _goal.resizeAs(_q0);
//  limits(4,0) = ;
//  limits(6,0) = _q0(6)-M_PI;
//  limits(4,1) = _q0(4)+M_PI;
//  limits(6,1) = _q0(6)+M_PI;


  for (uint i = 0;i<_q0.N;i++) {
//    _goal(i) = fRand( max(ARR(limits(i,0),_q0(i)-M_PI_2)) , min(ARR(limits(i,1),_q0(i)+M_PI_2)));
    _goal(i) = fRand( qLowerLimit[i], qUpperLimit[i] );
  }
}

