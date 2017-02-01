#include <Algo/spline.h>
#include <Core/array.h>
#include <Gui/plot.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Optim/optimization.h>
#include <Kin/kin.h>
#include <pr2/roscom.h>
#include <System/engine.h>
#include "../12_MBMF_LEARNING/task_manager.h"

#include "../src/plotUtil.h"
#include <pr2/roscom.h>
#include <System/engine.h>

//#include "../12_MBMF_LEARNING/motion_interface.h"
#include "../17_FINAL/src/motion_interface.h"
#include "../src/traj_factory.h"


void transformTrajectory(mlr::KinematicWorld &world, const arr &X, arr &XT) {
  arr T = ARR(80.,X.d0);

  arr Pdemo1,P1,Pdemo2,P2;
  TrajFactory tf;
  tf.compFeatTraj(X,Pdemo1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(X,P1,world,new DefaultTaskMap(posTMT,world,"endeffC1"));
  tf.compFeatTraj(X,Pdemo2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));
  tf.compFeatTraj(X,P2,world,new DefaultTaskMap(posTMT,world,"endeffC2"));

  arr qG;
  tf.compFeatTraj(X,qG,world,new TaskMap_qItself(world,"l_gripper_joint"));
  cout << qG << endl;


  world.gl().add(drawRedLine,&(Pdemo1));
  world.gl().add(drawGreenLine,&(Pdemo2));

  world.gl().add(drawRedPoints,&(P1));
  world.gl().add(drawGreenPoints,&(P2));
  displayTrajectory(X,-1,world,"demo");

  arr offset = ARR(0.0,-0.04,0.0);

  world.setJointState(X[T(0)]);
  arr R1 = world.getShapeByName("endeffC1")->X.rot.getArr();
  arr R2 = world.getShapeByName("endeffC2")->X.rot.getArr();

  XT = X;
  uint qIdx = world.getJointByName("l_gripper_joint")->qIndex;
  for (uint t=0; t<T(0); t++){
    /// linear transition from trajectory to contact point
    P1[t] = P1[t] + t/T(0)*R1*offset;
    P2[t] = P2[t] - t/T(0)*R2*offset;
    qG[t] = qG[t] + t/T(0)*.02;
    XT(t,qIdx) = XT(t,qIdx) + t/T(0)*.02;
  }
  for (uint t=T(0); t<T(1); t++){
    world.setJointState(X[t]);
    /// linear transition from trajectory to contact point
    R1 = world.getShapeByName("endeffC1")->X.rot.getArr();
    P1[t] = P1[t] + R1*offset;
    R2 = world.getShapeByName("endeffC2")->X.rot.getArr();
    P2[t] = P2[t] - R2*offset;
    qG[t] = qG[t] + .02;
    XT(t,qIdx) = XT(t,qIdx) + .02;
  }
/*
  MotionProblem MP(world,false);
  MP.T = T(1)-1;
  MP.tau = 0.01;
  MP.x0 = X[0];

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-3);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = world.getHmetric();


  t =MP.addTask("qGripper", new TaskMap_qItself(world,"l_gripper_joint"));
  t->setCostSpecs(0,MP.T, P1, 1e2);

//  t =MP.addTask("posC1", new DefaultTaskMap(posTMT,world,"endeffC1"));
//  t->setCostSpecs(0,MP.T, P1, 1e2);
//  t =MP.addTask("posC2", new DefaultTaskMap(posTMT,world,"endeffC2"));
//  t->setCostSpecs(0,MP.T, P2, 1e2);

  MotionProblemFunction MPF(MP);
  XT = X;
  OptOptions o;
  o.stopTolerance = 1e-2; o.constrainedMethod=anyTimeAula; o.verbose=1;
  optConstrainedMix(XT, NoArr, Convert(MPF), o);

*/
  displayTrajectory(X,-1,world,"demo");
  displayTrajectory(XT,-1,world,"trans");

  world.watch(true);

}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  bool useRos = mlr::getParameter<bool>("useRos");


  TaskManager *tm = new DonutTask();
  mlr::KinematicWorld world("model.kvg");
  cout << world.getJointByName("l_gripper_joint")->qIndex << endl;

  Motion_Interface *mi = new Motion_Interface(world);

  arr X;
//  mi->recordDemonstration(X,20.);
//  write(LIST<arr>(X),"data/Xdemo.dat");

  X << FILE("data/Xdemo.dat");


  cout << ">> Demonstration loaded <<" << endl;

  arr x0 = X[0];
  mi->gotoPosition(x0);
  mi->executeTrajectory(X,20.);
/*
  arr XT;
  transformTrajectory(world,X,XT);

  x0 = XT[0];
  mi->gotoPosition(x0);
  mi->executeTrajectory(XT,20.);
*/
  mi->~Motion_Interface();
  return 0;

}
