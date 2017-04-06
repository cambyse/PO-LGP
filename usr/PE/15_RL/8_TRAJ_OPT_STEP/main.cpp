#include <Core/array.h>
#include <Gui/plot.h>
#include <Algo/spline.h>
#include <Optim/optimization.h>
#include <KOMO/komo.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>
#include "../src/plotUtil.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// create reference motion
  mlr::KinematicWorld world("test.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MPref(world,false);
  MPref.T = 50;
  MPref.tau = 0.01;
  mlr::Shape *grasp = world.getShapeByName("endeff");
  mlr::Body *target = world.getBodyByName("target");
  mlr::Body *target2 = world.getBodyByName("target2");
  Task *t;
  t = MPref.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0, MPref.T, ARR(0.), 1e-1);
  t =MPref.addTask("pos1", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MPref.T*.5,MPref.T*.5,ARR(target2->X.pos),1e2);
  t =MPref.addTask("pos2", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(MPref.T-3,MPref.T,ARR(target->X.pos),1e2);
  MotionProblemFunction MPFref(MPref);
  arr Xref(MPref.T+1,q.N); Xref.setZero();
  optConstrainedMix(Xref, NoArr, Convert(MPFref), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  world.setJointState(q);
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.01;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-1);

  t = MP.addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP.T,MP.T,Xref[Xref.d0-1],1e3);

  MP.x0 = Xref[0];
//  MP.postfix = repmat(~Xref[Xref.d0-1],t->map.order,1);

  MotionProblemFunction MPF(MP);
  arr X = Xref;
  world.gl().resize(800,800);
  arr P1,P2;
  drawPoints(world,Xref,P2,"endeff",1);
  for(uint l=0;l<100; l++){
    drawPoints(world,X,P1,"endeff",0);

    OptNewton(X, Convert(MPF),OPT(verbose=1, stopTolerance = 1e-7, maxStep = 1e-1)).step();
    world.watch(true);
  }
//  X = MP.getInitialization();
//  optConstrainedMix(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-7));



  displayTrajectory(Xref,MPref.T,world,"Xref");
  displayTrajectory(X,MP.T,world,"X");
  world.watch(true);


  return 0;


  //  optNewton(X, Convert(MPF), OPT(verbose=0, stopIters=100,stopTolerance = 1e-6));

  //  ScalarFunction KOMF = Convert(MPF);
  //  cout << KOMF(NoArr,NoArr,X) << endl;

//  arr phi, J;
//  ObjectiveTypeA tt;
//  ConstrainedProblemMix v = Convert(MPF);

//  for(uint l=0;l<100; l++){
//    v(phi, J, tt, X);
//    J= unpack(J);
//    arr g = ~phi*J;
//    double alpha = 1e-4;
//    g=~g;
//    cout <<g.reshape(X.d0,X.d1) << endl;
//    X = X - alpha*g;
//  }

  /*
  LagrangianProblemMix UCP(KOMF,ConstrainedMethodType::augmentedLag);

  cout << X << endl;
  arr a = newton.x;
  for(uint l=0;l<20; l++){
    newton.step();
  }
  cout << newton.x << endl;
  return 0;
  for (uint i=1; i<3;i++) {
    arr L;
    double c = UCP.lagrangian(L,NoArr,X);
    double alpha = 1e-7;
    X = X - alpha*repmat(L,X.d0,X.d1);
  }
  //  LagrangianProblemMix KOMF = Convert(MPF);
  //  KOMF(X);
  //  optConstrainedMix(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  displayTrajectory(Xref,MPref.T,world,"Xref");
  displayTrajectory(X,MP.T,world,"X");
  return 0;

  uint T = Xref.d0-1;
  uint D = 10;

  arr Y = zeros(D,2);
  uint k = 1;
  mlr::Spline S(T,Y,k);
  arr A = S.basis;

  double w = 1e3;
  arr XX = Xref;
  arr AA = A;
  XX[0] = XX[0]*sqrt(w);
  AA[0] = AA[0]*sqrt(w);
  XX[XX.d0-1] = XX[XX.d0-1]*sqrt(w);
  AA[AA.d0-1] = AA[AA.d0-1]*sqrt(w);

  arr beta = inverse(~AA*AA)*~AA*XX;
  mlr::Spline S2(100,beta,k);
  arr path = S2.eval();

  arr Xres;
  for (uint i=0;i<=MPref.T;i++) {
    Xres.append(~S2.eval(double(i)/MPref.T));
  }

  cout << path << endl;
  cout << Xres << endl;

  displayTrajectory(Xref,MPref.T,world,"reference");
  displayTrajectory(Xres,MPref.T,world,"spline");

  write(LIST<arr>(Xref),"data/X.dat");
  write(LIST<arr>(Xres),"data/Xres.dat");
  write(LIST<arr>(beta),"data/beta.dat");
  write(LIST<arr>(path),"data/path.dat");
//*/
  return 0;
}

