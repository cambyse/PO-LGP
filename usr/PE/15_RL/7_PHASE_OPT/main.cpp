#include <Optim/optimization.h>
#include <Kin/taskMaps.h>
#include <Plot/plot.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include "../src/plotUtil.h"
#include "../src/phase_optimization.h"


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// create reference motion
  mlr::KinematicWorld world("test.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  KOMO MP(world,false);
  MP.T = 100;
  MP.tau = 0.01;
  mlr::Shape *grasp = world.getShapeByName("endeff");
  mlr::Body *target = world.getBodyByName("target");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARR(target->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr X(MP.T+1,q.N); X.setZero();
  optConstrainedMix(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));


  double T = 1.;
  uint N = X.d0;
  arr time = linspace(0,T,N-1);

  //-- optimize phase of trajectory
  uint k = 3;
  PhaseOptimization P(X,k,1);
  arr sOpt = P.getInitialization();
  checkGradient(Convert(P), sOpt, 1e-3);
//  checkAllGradients(
  optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=1e-4));
  arr Xres;
  P.getSolution(Xres,sOpt);

  arr A,Aopt;
  getAcc(A,X,1);
  getAcc(Aopt,Xres,1);

  arr V,Vopt;
  getVel(V,X,1);
  getVel(Vopt,Xres,1);

  cout << "Acc: " << sumOfSqr(A) << endl;
  cout << "Acc res: " << sumOfSqr(Aopt) << endl;

  cout << "Vel: " << sumOfSqr(V) << endl;
  cout << "Vel res: " << sumOfSqr(Vopt) << endl;


  displayTrajectory(X,MP.T,world,"reference traj");
  displayTrajectory(Xres,MP.T,world,"optimized traj");

  write(LIST<arr>(time,X),"data/X.dat");
  write(LIST<arr>(time,Xres),"data/Xres.dat");
  write(LIST<arr>(time,sOpt),"data/sOpt.dat");

  return 0;
}
