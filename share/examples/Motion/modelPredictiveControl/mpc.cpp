#include "mpc.h"

#define VISUALIZE 1

MPC::MPC(MotionProblem &_P, arr &_x):
  P(_P),
x(_x)
{
//  P = &_P;
//  MotionProblem c = _P;
//  x = _x;
  arr kinPos;

  x_cart.clear();
  // store cartesian coordinates and endeffector orientation
  for (uint j=0;j<x.d0-1;j++) {
    P.world.setJointState(x[j]);
    P.world.kinematicsPos(kinPos,NoArr, P.world.getBodyByName("endeff")->index);
    x_cart.append(~kinPos);
  }

}

void MPC::replan(arr &_goal, arr &_q) {
  P.T = P.T - 1;

  P.taskCosts.clear();

  x_bk.append(~_q);

  if (P.T < 3) {
    x = x.rows(x.d0-3,x.d0);
    return;
  }

  if (x_bk.d0 > 2){
    arr prefix(2,x_bk.d1);
    prefix[1] = x_bk[x_bk.d0-2];
    prefix[0] = x_bk[x_bk.d0-3];
    P.prefix = prefix;
  }

  TaskCost *c2;
  c2 = P.addTask("position", new DefaultTaskMap(posTMT,P.world,"endeff", ors::Vector(0., 0., 0.)));
  P.setInterpolatingCosts(c2, MotionProblem::finalOnly, _goal, 1e4);
  c2 = P.addTask("position", new DefaultTaskMap(posTMT,P.world,"endeff", ors::Vector(0., 0., 0.)));
  c2->map.order=1;
  P.setInterpolatingCosts(c2, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c2 = P.addTask("orientation", new DefaultTaskMap(vecTMT,P.world,"endeff",ors::Vector(0., 0., 1.)));
  P.setInterpolatingCosts(c2, MotionProblem::finalOnly, ARRAY(1.,0.,0.), 1e4);
  c2 = P.addTask("orientation", new DefaultTaskMap(vecTMT,P.world,"endeff",ors::Vector(0., 0., 1.)));
  c2->map.order=1;
  P.setInterpolatingCosts(c2,MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e3);

  c2 = P.addTask("contact", new DefaultTaskMap(collTMT,-1,NoVector,-1,NoVector,ARR(0.1)));
  P.setInterpolatingCosts(c2, MotionProblem::constant, ARRAY(0.), 1e0);

  x = x.rows(1,x.d0);
  MotionProblemFunction F(P);
  optNewton(x, Convert(F), OPT(verbose=0, stopIters=20, damping=1e-3, maxStep=1.));


  // transform trajectory in cartesian space for visualization
  arr kinPos;
  x_cart.clear();
  // store cartesian coordinates and endeffector orientation
  for (uint j=0;j<x.d0-1;j++) {
    P.world.setJointState(x[j]);
    P.world.kinematicsPos(kinPos,NoArr, P.world.getBodyByName("endeff")->index);
    x_cart.append(~kinPos);
  }
}

MPC::~MPC() {

}
