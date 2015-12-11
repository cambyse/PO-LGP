#include "task_manager.h"
#include <Motion/pr2_heuristics.h>
#include "traj_factory.h"
#include "plotUtil.h"

void ButtonTask::addConstraints(MotionProblem *MP, const arr &X)
{
  TrajFactory tf;
  arr yC1;
  tf.compFeatTraj(X,yC1,MP->world,new DefaultTaskMap(posTMT,MP->world,"endeffL"));

  arr prec = constraintTime*1e3;
  Task *t;
  t = MP->addTask("posC1", new DefaultTaskMap(posTMT,MP->world,"endeffL"));
  t->target = yC1;
  t->prec = prec;
  cout << prec << endl;
}

void ButtonTask::addModelConstraints(MotionProblem *MP,arr &target)
{
  arr prec = constraintTime;
  Task *t;
  /// add contact constraint
  t = MP->addTask("posC1", new PointEqualityConstraint(MP->world,"endeffL",NoVector,"b1"));
  t->target = 0.;
  t->prec = prec*1.;
  /// add final external joint state
  t = MP->addTask("posC1", new TaskMap_qItself(MP->world,"b1_b2"));
  t->setCostSpecs(constraintCP(1),constraintCP(1),target,1e1);
}

void ButtonTask::updateVisualization(ors::KinematicWorld &world,arr &X, arr &Y) {
  drawLine(world,X,Pdemo1f,"endeffC1",0,0,constraintCP(0));
  drawLine(world,X,Pdemo1c,"endeffC1",2,constraintCP(0),constraintCP(1));
  drawLine(world,X,Pdemo2f,"endeffC2",0,0,constraintCP(0));
  drawLine(world,X,Pdemo2c,"endeffC2",2,constraintCP(0),constraintCP(1));

  if (&Y) {
    drawLine(world,Y,PX1f,"endeffC1",1,0,constraintCP(1));
    drawLine(world,Y,PX2f,"endeffC2",1,0,constraintCP(1));
  }
}

void ButtonTask::computeConstraintTime(const arr &F,const arr &X) {
  constraintTime = zeros(X.d0); constraintTime.flatten();
  for (uint t=floor(X.d0/2.);t<X.d0;t++){
    constraintTime.subRange(t-3,t) = 1.;
  }
  constraintCP = ARR(constraintTime.findValue(1.),F.d0); constraintCP.flatten();
  cout << "constraintTime: " << constraintTime << endl;
  cout << "constraintCP: " << constraintCP << endl;
}


bool ButtonTask::transformTrajectory(arr &Xn, const arr &x, arr &Xdemo){
  arr C1demo;
  TrajFactory tf;
  tf.compFeatTraj(Xdemo,C1demo,*world,new DefaultTaskMap(posTMT,*world,"endeffL"));
  arr C1trans = C1demo;

  world->gl().resize(800,800);
  updateVisualization(*world,Xdemo);
  world->gl().update();

  arr offsetC1 = ARR(x(0),x(1),0.);

  double s;
  for (uint t=0;t<constraintCP(0);t++) {
    s = t/constraintCP(0);
    C1trans[t] = C1trans[t] +s*offsetC1;
  }
  for (uint t=constraintCP(0);t<constraintCP(1);t++) {
    C1trans[t] = C1trans[t] + offsetC1;
  }

  MotionProblem MP(*world,false);
  MP.T = Xdemo.d0-1;
  MP.tau = mlr::getParameter<double>("duration")/MP.T;
  MP.x0 = Xdemo[0];

  /// tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-1);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);

  t =MP.addTask("posC1", new DefaultTaskMap(posTMT,*world,"endeffL"));
  t->setCostSpecs(0,MP.T, C1trans, 1e3);

  MotionProblemFunction MPF(MP);
  Xn = Xdemo;
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0;
  optConstrainedMix(Xn, NoArr, Convert(MPF), o);

//  arr Pn1,Pn2;
//  drawPoints(*world,Xn,Pn1,"endeffC1",1);
//  drawPoints(*world,Xn,Pn2,"endeffC2",1);

  // check if gripper limits are exceeded
//  cout << " parameter min=" << Xn.col(24).min() << " max=" << Xn.col(24).max() << endl;
//  cout << " parameter lim upper=" << world->getJointByName("l_gripper_joint")->limits(1) << " lower=" << world->getJointByName("l_gripper_joint")->limits(0) << endl;

  return true;
}

bool ButtonTask::success(const arr &X, const arr &Y) {
  return true;
}

double ButtonTask::reward(const arr &Z){
  return exp(-.05*sumOfAbs(Z)/Z.d0);
}
