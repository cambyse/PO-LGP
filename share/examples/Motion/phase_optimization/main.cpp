#include <Motion/taskMaps.h>
#include <Motion/motion.h>
#include <Motion/phase_optimization.h>
#include <Optim/lagrangian.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>

void TEST(PhaseOptimization){
  /// create reference motion
  mlr::KinematicWorld world("test.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MP(world,false);
  MP.T = 100;
  MP.tau = 0.01;
  mlr::Shape *grasp = world.getShapeByName("endeff");
  mlr::Body *target = world.getBodyByName("target");
  Task *t;
  t = MP.addTask("tra", new TaskMap_Transition(world));
  t->map.order=1;
  t->setCostSpecs(0, MP.T, {0.}, 1e-2);
  t =MP.addTask("pos", new TaskMap_Default(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,conv_vec2arr(target->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr X(MP.T+1,q.N); X.setZero();
  optConstrained(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  /// optimize phase of trajectory
  uint k = 2;
  PhaseOptimization P(X,k);
  arr sOpt = P.getInitialization();
  checkJacobianCP(Convert(P),sOpt,1e-3);
  optNewton(sOpt, Convert(P),OPT(verbose=2,stopTolerance=1e-4));
  arr Xres;
  P.getSolution(Xres,sOpt);

  /// evaluate solution
  arr A,Aopt;
  getAcc(A,X,1);
  getAcc(Aopt,Xres,1);
  arr V,Vopt;
  getVel(V,X,1);
  getVel(Vopt,Xres,1);

  cout << "\nAcc costs before phaseOpt: " << sumOfSqr(A) << endl;
  cout << "Acc costs after phaseOpt: " << sumOfSqr(Aopt) << endl;

  cout << "\nVel costs before phaseOpt: " << sumOfSqr(V) << endl;
  cout << "Vel costs after phaseOpt: " << sumOfSqr(Vopt) << endl;

  displayTrajectory(X,MP.T,world,"reference traj");
  displayTrajectory(Xres,MP.T,world,"optimized traj");
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  testPhaseOptimization();
  return 0;
}
