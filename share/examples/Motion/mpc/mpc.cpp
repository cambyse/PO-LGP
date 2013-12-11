#include "mpc.h"

#define VISUALIZE 1

MPC::MPC(uint _plan_time_factor, ors::Graph &_orsG):
  plan_time_factor(_plan_time_factor),
  orsG(&_orsG)
{
  P = new MotionProblem(&_orsG);
  P->loadTransitionParameters();
  F = new MotionProblemFunction(*P);
  T=F->get_T(); uint k=F->get_k(); n=F->dim_x(); dt=P->tau;
  cout <<"Problem parameters:"<<" T="<<T<<" k="<<k<<" n="<<n<<" dt="<<dt<<" # joints=" <<_orsG.getJointStateDimension()<<endl;

  plan_time = dt*plan_time_factor; // reoptimize trajectory at plan_time

  TaskCost *c;
  c = P->addTaskMap("position", new DefaultTaskMap(posTMT,*P->ors,"endeff", ors::Vector(0., 0., 0.)));
  P->setInterpolatingCosts(c, MotionProblem::finalOnly,
                           ARRAY(P->ors->getBodyByName("goalRef")->X.pos), 1e4,
                           ARRAY(0.,0.,0.), 1e-3);
  P->setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                              ARRAY(0.,0.,0.), 1e3,
                              ARRAY(0.,0.,0.), 0.);


  yRef = arr(T+1,n);
  optNewton(yRef, Convert(*F), OPT(verbose=1, stopIters=20, useAdaptiveDamping=false, damping=1e-3, maxStep=1.));

  arr knots = linspace(0,T*dt,T);
  s = new Spline(knots,yRef,2);

  // init times
  t_prev = 0.;
  t_plan_prev = 0.;

#ifdef VISUALIZE
  // transform trajectory in cartesian space for visualization
  arr kinPos;
  y_cart.clear();
  // store cartesian coordinates and endeffector orientation
  for (uint j=0;j<yRef.d0-1;j++) {
    orsG->setJointState(yRef[j]);
    orsG->calcBodyFramesFromJoints();
    orsG->kinematicsPos(kinPos,P->ors->getBodyByName("endeff")->index);
    y_cart.append(~kinPos);
  }
#endif
}

arr MPC::iterate(double _t, arr &_state, arr &_goal, double _simRate) {
  P->swift->computeProxies(*orsG);

  // save trajectory every dt steps
  if (_t >= (t_prev+dt)) {
    y_bk.append(~_state);
    t_prev = _t;
  }

  // check for reoptimize
  if (_t >= (t_plan_prev+plan_time-1e-10) && y_bk.d0>2) {
    replanTrajectory(_state,_goal,_t);
    t_plan_prev = _t;
  }

  // return desired next state at (_t + _simRate)
  return s->eval(_t+_simRate);
}

void MPC::replanTrajectory(arr &_state, arr &_goal, double _t) {
  P->T = P->T-plan_time_factor;
  cout << "P.T: " << P->T << endl;

  arr prefix(2,n);
  prefix[1] = y_bk[y_bk.d0-2];
  prefix[0] = y_bk[y_bk.d0-3];
  P->prefix = prefix;

  // reset costs
  P->costMatrix.clear();
  TaskCost *c;
  c = P->addTaskMap("position", new DefaultTaskMap(posTMT,*P->ors,"endeff", ors::Vector(0., 0., 0.)));

  P->setInterpolatingCosts(c, MotionProblem::finalOnly,
                           _goal, 1e4,
                           ARRAY(0.,0.,0.), 1e-3);
  P->setInterpolatingVelCosts(c, MotionProblem::finalOnly,
                              ARRAY(0.,0.,0.), 1e3,
                              ARRAY(0.,0.,0.), 0);

  // update reference trajectory
  yRef = yRef.rows(plan_time_factor,yRef.d0);
  MT::timerStart();
  optNewton(yRef, Convert(*F), OPT(verbose=1, stopIters=10, useAdaptiveDamping=false, damping=1e-3, maxStep=1., stopTolerance=1e-2));
  std::cout <<"optimization time: " <<MT::timerRead() <<"sec" <<std::endl;

  arr knots = linspace(_t,T*dt,P->T);
  s = new Spline(knots,yRef,2);

#ifdef VISUALIZE
  // transform trajectory in cartesian space for visualization
  arr kinPos;
  y_cart.clear();
  // store cartesian coordinates and endeffector orientation
  for (uint j=0;j<yRef.d0-1;j++) {
    orsG->setJointState(yRef[j]);
    orsG->calcBodyFramesFromJoints();
    orsG->kinematicsPos(kinPos,P->ors->getBodyByName("endeff")->index);
    y_cart.append(~kinPos);
  }
#endif

}

MPC::~MPC() {

}
