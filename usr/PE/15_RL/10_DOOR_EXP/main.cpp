#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Ors/ors.h>
#include "../src/plotUtil.h"
#include "../src/phase_optimization.h"
#include "../src/traj_factory.h"

arr createDoorDemonstration(ors::KinematicWorld &world, arr &tp) {
  arr q; world.getJointState(q);
  world.setJointState(q,q*0.);

  MotionProblem MP(world,false);
  MP.T = 150;
  MP.tau = 0.02;
  MP.x0 = q;

  tp = floor(ARRAY(0.30*MP.T,0.50*MP.T,0.7*MP.T,1.*MP.T));

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-3);


  t =MP.addTask("pre_pos", new DefaultTaskMap(posTMT,world,"endeffL"));
  t->setCostSpecs(tp(0),tp(0), ARRAY(world.getBodyByName("handle")->X.pos - ors::Vector(0.1,0,0.)), 1e2);
  t =MP.addTask("pre_vec", new DefaultTaskMap(vecAlignTMT,world,"endeffL",ors::Vector(0.,1.,0.),__null,ors::Vector(0.,0.,-1.)));
  t->setCostSpecs(tp(0),tp(0), ARR(1.), 1e2);
  t =MP.addTask("handle_pos", new TaskMap_qItself(world.getJointByName("door_handle")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(tp(2)-2,tp(2), ARR(0.3), 1e3);
  t =MP.addTask("door_pos", new TaskMap_qItself(world.getJointByName("frame_door")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(tp(3)-2,tp(3), ARR(-0.45), 1e2);


  //--constraints
  t = MP.addTask("qLimits", new LimitsConstraint());
  t->setCostSpecs(0., MP.T, ARR(0.), 1.);
  t =MP.addTask("c1", new PointEqualityConstraint(world, "endeffC1",NoVector, "cp1",NoVector));
  t->setCostSpecs(tp(1)+1, tp(3), ARR(0.), 1.);
  t =MP.addTask("c2", new PointEqualityConstraint(world, "endeffC2",NoVector, "cp2",NoVector));
  t->setCostSpecs(tp(1)+1, tp(3), ARR(0.), 1.);
  t =MP.addTask("door_fix", new qItselfConstraint(world.getJointByName("frame_door")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(0, tp(2)-1, ARR(0.), 1.);
  t =MP.addTask("handle_fix", new qItselfConstraint(world.getJointByName("door_handle")->qIndex,world.getJointStateDimension()));
  t->setCostSpecs(0, tp(1), ARR(0.), 1.);

  MotionProblemFunction MPF(MP);
  arr X = MP.getInitialization();
  OptOptions o; o.maxStep = 1.;
  o.stopTolerance = 1e-4; o.constrainedMethod=anyTimeAula; o.verbose=1; o.aulaMuInc=1.1;

  if (MT::getParameter<bool>("loadDemoFromFile")){
    X << FILE("data/X.dat");
  }else{
    optConstrainedMix(X, NoArr, Convert(MPF), o);
    MP.costReport(true);
    write(LIST<arr>(X),"data/X.dat");
  }

  return X;
}

int main(int argc,char **argv){
  MT::initCmdLine(argc,argv);

  arr tp;
  ors::KinematicWorld world("robot_door.ors");
  arr xDem = createDoorDemonstration(world,tp);

  TrajFactory tf;
  arr yC1, yC2;

  tf.compFeatTraj(xDem,yC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(xDem,yC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
  yC1.subRange(0,tp(1)-1)= 0.;
  yC2.subRange(0,tp(1)-1)= 0.;

  world.gl().resize(800,800);
  world.gl().add(drawRedPoints,&(yC1));
  world.gl().add(drawRedPoints,&(yC2));


  /// step-wise model based improvement
  MotionProblem MP(world,false);
  MP.T = 150;
  MP.tau = 0.02;
  MP.x0 = xDem[0];

  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  ((TransitionTaskMap*)&t->map)->H_rate_diag(world.getJointByName("frame_door")->qIndex) = 0.;
  ((TransitionTaskMap*)&t->map)->H_rate_diag(world.getJointByName("door_handle")->qIndex) = 0.;

  // final position constraint
  t = MP.addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP.T,MP.T,xDem[xDem.d0-1],1e3);

  // endeffector position constraint
  t =MP.addTask("c1", new DefaultTaskMap(posTMT,world, "endeffC1"));
  t->setCostSpecs(tp(1),tp(3),yC1.subRange(0,tp(3)),1e3);
  t =MP.addTask("c2", new DefaultTaskMap(posTMT,world, "endeffC2"));
  t->setCostSpecs(tp(1),tp(3),yC2.subRange(0,tp(3)),1e3);

  MotionProblemFunction MPF(MP);
  arr X = xDem;

  OptOptions o; o.maxStep = 1e-1;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=1; o.aulaMuInc=1.1;

//  optConstrainedMix(X, NoArr, Convert(MPF), o);
  arr wC1,wC2;
  tf.compFeatTraj(X,wC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(X,wC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
  world.gl().add(drawBluePoints,&(wC1));
  world.gl().add(drawBluePoints,&(wC2));

  ConstrainedProblemMix CPM = Convert(MPF);
  UnconstrainedProblemMix UPM(CPM, o.constrainedMethod);
  OptNewton opt(X, UPM, o);



  for(uint l=0;l<10; l++){
    opt.step();

    tf.compFeatTraj(X,wC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
    tf.compFeatTraj(X,wC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
    world.setJointState(xDem[0]);
    world.watch(true);
  }
//  displayTrajectory(X,X.d0,world,"X");


  /// phase optimization
  uint k = 2;
  PhaseOptimization P(X,k,1);
  arr sOpt = P.getInitialization();
  optConstrained(sOpt, NoArr, Convert(P),OPT(verbose=0,stopTolerance=1e-4));
  arr Xres;
  P.getSolution(Xres,sOpt);
  arr A,Aopt;
  getAcc(A,X,1);
  getAcc(Aopt,Xres,1);
  cout << "Acc: " << sumOfSqr(A) << endl;
  cout << "Acc res: " << sumOfSqr(Aopt) << endl;

  arr vC1,vC2;
  tf.compFeatTraj(Xres,vC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(Xres,vC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
//  world.gl().add(drawGreenPoints,&(vC1));
//  world.gl().add(drawGreenPoints,&(vC2));

  world.setJointState(xDem[0]);
  displayTrajectory(Xres,X.d0,world,"X");


  /// model free exploration
  arr zC1 = vC1;
  arr zC2 = vC2;
  arr off1 = ARR(0.00,-0.04,0.0);
  arr off2 = ARR(0.00,-0.02,0.0);
  for (uint t =0;t<xDem.d0;t++){
    world.setJointState(xDem[t]);
    arr R = world.getBodyByName("handle")->X.rot.getArr();
    zC1[t] = zC1[t] + R*off1;
    zC2[t] = zC2[t] + R*off2;
  }
//  zC1.subRange(0,tp(1)-1)= 0.;
//  zC2.subRange(0,tp(1)-1)= 0.;

  world.gl().add(drawRedPoints,&(zC1));
  world.gl().add(drawRedPoints,&(zC2));



  MotionProblem MP2(world,false);
  MP2.T = 150;
  MP2.tau = 0.02;
  MP2.x0 = xDem[0];

  t = MP2.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP2.T, ARR(0.), 1e-2);
  ((TransitionTaskMap*)&t->map)->H_rate_diag(world.getJointByName("frame_door")->qIndex) = 0.;
  ((TransitionTaskMap*)&t->map)->H_rate_diag(world.getJointByName("door_handle")->qIndex) = 0.;

  // final position constraint
  t = MP2.addTask("qT", new TaskMap_qItself());
  t->setCostSpecs(MP2.T,MP2.T,xDem[xDem.d0-1],1e1);

  // endeffector position constraint
  t =MP2.addTask("c1", new DefaultTaskMap(posTMT,world, "endeffC1"));
  t->setCostSpecs(tp(1)-10,tp(3),zC1.subRange(0,tp(3)),1e3);
  t =MP2.addTask("c2", new DefaultTaskMap(posTMT,world, "endeffC2"));
  t->setCostSpecs(tp(1)-10,tp(3),zC2.subRange(0,tp(3)),1e3);

  MotionProblemFunction MPF2(MP2);
  arr Xmf = Xres;

  OptOptions o2; o2.maxStep = 1e-1;
  o2.stopTolerance = 1e-3; o2.constrainedMethod=anyTimeAula; o2.verbose=1; o2.aulaMuInc=1.1;

  arr uC1,uC2;

//  for(uint l=0;l<10; l++){
//    OptNewton(Xmf, UnconstrainedProblemMix(Convert(MPF2), o2.constrainedMethod), o2).step();
//    tf.compFeatTraj(Xmf,uC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
//    tf.compFeatTraj(Xmf,uC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
//    displayTrajectory(Xmf,Xmf.d0,world,"Xmf");
//  }
  optConstrainedMix(Xmf, NoArr, Convert(MPF2), o2);

  tf.compFeatTraj(Xmf,uC1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(Xmf,uC2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
  world.gl().add(drawGreenPoints,&(uC1));
  world.gl().add(drawGreenPoints,&(uC2));

  world.setJointState(xDem[0]);
  displayTrajectory(Xmf,Xmf.d0,world,"X");


  /// repeat door motions from beginning
  /// check distance to door as criterion of success


  for (;;){
    world.setJointState(xDem[0]);
    world.watch(true);
    displayTrajectory(Xmf,Xmf.d0,world,"Xmf");
    world.watch(true);
  }

  return 0;
}

