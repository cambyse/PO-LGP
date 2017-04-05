#include <Core/array.h>
#include <Gui/plot.h>
#include <Algo/spline.h>
#include <Optim/optimization.h>
#include <KOMO/komo.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

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
  mlr::Body *target2 = world.getBodyByName("target2");
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  t =MP.addTask("pos1", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T*.5,MP.T*.5,ARR(target->X.pos),1e2);
  t =MP.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MP.T-3,MP.T,ARR(target2->X.pos),1e2);
  MotionProblemFunction MPF(MP);
  arr X(MP.T+1,q.N); X.setZero();
  optConstrainedMix(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

//  X = ~ARR(0.,0.);
//  X.append(ARR(0.5,1.0));
//  X.append(ARR(1.2,2.));
//  X.append(ARR(2., 0.5));
//  X.append(ARR(3.,0.8));
//  X.append(ARR(4.,1.8));
//  X.append(ARR(7.,2.));

  uint T = X.d0-1;
  uint D = 10;

  arr Y = zeros(D,2);
  uint k = 1;
  mlr::Spline S(T,Y,k);
  arr A = S.basis;

  double w = 1e3;
  arr XX = X;
  arr AA = A;
  XX[0] = XX[0]*sqrt(w);
  AA[0] = AA[0]*sqrt(w);
  XX[XX.d0-1] = XX[XX.d0-1]*sqrt(w);
  AA[AA.d0-1] = AA[AA.d0-1]*sqrt(w);

  arr beta = inverse(~AA*AA)*~AA*XX;
  mlr::Spline S2(100,beta,k);
  arr path = S2.eval();

  arr Xres;
  for (uint i=0;i<=MP.T;i++) {
    Xres.append(~S2.eval(double(i)/MP.T));
  }

  cout << path << endl;
  cout << Xres << endl;

  displayTrajectory(X,MP.T,world,"reference");
  displayTrajectory(Xres,MP.T,world,"spline");

  write(LIST<arr>(X),"data/X.dat");
  write(LIST<arr>(Xres),"data/Xres.dat");
  write(LIST<arr>(beta),"data/beta.dat");
  write(LIST<arr>(path),"data/path.dat");

  return 0;
}

