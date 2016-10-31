#include <Core/array.h>
#include <Gui/plot.h>
#include <Algo/spline.h>
#include <Optim/optimization.h>
#include <Motion/motion.h>
#include <Ors/ors.h>
#include <Motion/taskMaps.h>
#include "../src/plotUtil.h"
#include "../src/phase_optimization.h"

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  /// create reference motion
  ors::KinematicWorld world("test.ors");
  arr q, qdot;
  world.getJointState(q, qdot);
  world.swift();
  MotionProblem MPref(world,false);
  MPref.T = 50;
  MPref.tau = 0.01;
  ors::Shape *grasp = world.getShapeByName("endeff");
  Task *t;
  arr timepoints = ARR(20.,30.,40.,50.);
  t = MPref.addTask("tra", new TransitionTaskMap(world));
  t->map.order=1;
  t->setCostSpecs(0, MPref.T, ARR(0.), 1e-1);

  for (uint i=0;i<timepoints.d0;i++) {
      mlr::String str; str << "target" <<i;
      t =MPref.addTask(str, new DefaultTaskMap(posTMT, grasp->index) );
      t->setCostSpecs(timepoints(i),timepoints(i),ARR(world.getBodyByName(str)->X.pos),1e3);
  }


  MotionProblemFunction MPFref(MPref);
  arr Xref(MPref.T+1,q.N); Xref.setZero();
  optConstrainedMix(Xref, NoArr, Convert(MPFref), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-3));

  arr P2;
  drawPoints(world,Xref,P2,"endeff",1);

  world.gl().resize(800,800);
  displayTrajectory(Xref,MPref.T,world,"Xref");
//  world.watch(true);

  /// convert to task space traj
  arr y(MPref.T+1,3);
  for (uint t=0;t<y.d0;t++) {
    arr tmp;
    world.setJointState(Xref[t]);
    world.kinematicsPos(tmp,NoArr,grasp->body,&grasp->rel.pos);
    y[t] = tmp;
  }
  cout << y << endl;

  world.setJointState(q);
  MotionProblem MP(world,false);
  MP.T = 50;
  MP.tau = 0.01;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-4);

  // final position constraint
  t = MP.addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP.T,MP.T,Xref[Xref.d0-1],1e3);

  // endeffector position constraint
  t =MP.addTask("pos_fix", new DefaultTaskMap(posTMT, grasp->index) );
  t->setCostSpecs(timepoints(1),timepoints(2),y.subRange(0,timepoints(2)),1e2);

  cout << t->prec << endl;
  MP.x0 = Xref[0];

  MotionProblemFunction MPF(MP);
  arr X = Xref;
  world.gl().resize(800,800);
  arr P1;

//  for(uint l=0;l<100; l++){
//    OptNewton(X, Convert(MPF),OPT(verbose=1, stopTolerance = 1e-7, maxStep = 1e-1)).step();
//    OptNewton(X, LagrangianProblemMix(Convert(MPF), augmentedLag), OPT(verbose=1, stopTolerance = 1e-7, maxStep = 1e-1)).step();
//    world.watch(true);
//  }

  optConstrainedMix(X, NoArr, Convert(MPF), OPT(verbose=0, stopIters=100, maxStep=1., stepInc=2., aulaMuInc=2,stopTolerance = 1e-7));
  cout << X << endl;
  drawPoints(world,X,P1,"endeff",0);

  displayTrajectory(X,MP.T,world,"Xref");



  /// do phase optimization
  uint k = 2;
  PhaseOptimization P(X,k,1);
  arr s0 = linspace(0,1,P.T-1); s0.flatten();
  arr sOpt = P.getInitialization();
  checkAllGradients(Convert(P), sOpt, 1e-3);
  optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=1,stopTolerance=1e-4));
  arr Xres;
  P.getSolution(Xres,sOpt);

  X.clear();
  for (uint i=0;i<sOpt.d0;i++) {
    X.append(~P.p->eval(s0(i)));
  }

  arr P3;
  drawPoints(world,Xres,P3,"endeff",2);
  displayTrajectory(Xres,MP.T,world,"Xref");

  arr Ares,Aref,A;
  getAcc(Aref,Xref,1.);
  getAcc(A,X,1.);
  getAcc(Ares,Xres,1.);

  cout << "sumOfSqr(Aref): " << sumOfSqr(Aref) << endl;
  cout << "sumOfSqr(A): " << sumOfSqr(A) << endl;
  cout << "sumOfSqr(Ares): " << sumOfSqr(Ares) << endl;

  arr Vres,Vref,V;
  getVel(Vref,Xref,1.);
  getVel(V,X,1.);
  getVel(Vres,Xres,1.);

  cout << "sumOfSqr(Vref): " << sumOfSqr(Vref) << endl;
  cout << "sumOfSqr(V): " << sumOfSqr(V) << endl;
  cout << "sumOfSqr(Vres): " << sumOfSqr(Vres) << endl;


  world.watch(true);
  return 0;

/*
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
  arr P1;
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
//  TermTypeA tt;
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

