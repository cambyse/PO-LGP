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

void ButtonTask::addModelConstraints(MotionProblem *MP,arr &target) {
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
  drawLine(world,X,Pdemo1f,"endeffL",0,0,conStart(0));
  drawLine(world,X,Pdemo1c,"endeffL",2,conStart(0),conEnd(0));
  drawLine(world,X,Pdemo2f,"endeffL",0,conEnd(0),X.d0);

  if (&Y) {
    drawLine(world,Y,PX1f,"endeffL",1,0,Y.d0);
  }
}

void ButtonTask::computeConstraintTime(const arr &F,const arr &X) {
  constraintTime = zeros(X.d0); constraintTime.flatten();
  for (uint t=0;t<X.d0;t++){
    if (fabs(F(t,2))>7.)
    constraintTime.subRange(t-3,t) = 1.;
  }

  /// compute contact changepoints
  arr conDiff; conDiff.resizeAs(constraintTime); conDiff.setZero();
  for(uint t=1; t<conDiff.d0; t++)  conDiff(t) = constraintTime(t) - constraintTime(t-1);

  conDiff.findValues(conStart,1.);
  conDiff.findValues(conEnd,-1.);

  cout << "constraintTime: " << constraintTime << endl;
  cout << "constraintStart " << conStart << endl;
  cout << "constraintEnd " << conEnd << endl;
  write(LIST<arr>(constraintTime),STRING(mlr::getParameter<mlr::String>("folder")<<"constraintTime.dat"));
  write(LIST<arr>(ARR(conStart)),STRING(mlr::getParameter<mlr::String>("folder")<<"conStart.dat"));
  write(LIST<arr>(ARR(conEnd)),STRING(mlr::getParameter<mlr::String>("folder")<<"conEnd.dat"));
}


bool ButtonTask::transformTrajectory(arr &Xn, const arr &x, arr &Xdemo){
  arr C1demo;
  TrajFactory tf;
  tf.compFeatTraj(Xdemo,C1demo,*world,new DefaultTaskMap(posTMT,*world,"endeffL"));
  arr C1trans = C1demo;

  arr offsetC1 = ARR(x(0),x(1),0.);

  double s;
  for (uint t=0;t<conStart(0);t++) {
    s = t/double(conStart(0));
    C1trans[t] = C1trans[t] +s*offsetC1;
  }
  for (uint t=conStart(0);t<conEnd(0);t++) {
    C1trans[t] = C1trans[t] + offsetC1;
  }
  for (uint t=conEnd(0);t<Xdemo.d0;t++) {
    s = (t-conEnd(0))/double(Xdemo.d0);
    C1trans[t] = C1trans[t] + (1.-s)*offsetC1;
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
  optConstrained(Xn, NoArr, Convert(MPF), o);
  return true;
}

void ButtonTask::getParamLimit(arr& paramLimit) {
  paramLimit.clear();
  paramLimit.append(~ARR(-0.02,0.02)); // hand opening
  paramLimit.append(~ARR(-0.02,0.02)); // hand position
}

void ButtonTask::getDofLimit(arr& dofLimit) {
  dofLimit.clear();
  dofLimit.append(~ARR(-0.02,0.02)); // pushing depth
}

bool ButtonTask::transformTrajectoryDof(arr& Xn, const arr& x_dof, arr& Xdemo){
  MotionProblem MP(*world,false);
  MP.T = Xdemo.d0-1;
  MP.tau = mlr::getParameter<double>("duration")/MP.T;
  MP.x0 = Xdemo[0];

  arr prec = constraintTime;
  arr prec_inv = -1.*(prec-1.);

  arr tmp = Xdemo.col(world->getJointByName("b2_b1")->qIndex); tmp.flatten();
  double b2_b1Min = tmp.min();
  uint mIdx = tmp.minIndex();

  arr param = x_dof + b2_b1Min;

  /// tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-1);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);
  /// homing trajectory
  t = MP.addTask("homing_traj", new TaskMap_qItself());
  t->target = Xdemo;
  t->prec = prec_inv*1e-2;
  /// add contact constraint
  t = MP.addTask("b2_b1_con", new PointEqualityConstraint(MP.world,"endeffL",NoVector,"b1_shape"));
  t->target = zeros(prec.d0,3);
  t->prec = prec*1.;
  /// final position constraint
  t = MP.addTask("b2_b1_T", new qItselfConstraint(world->getJointByName("b2_b1")->qIndex,world->getJointStateDimension()));
  t->setCostSpecs(mIdx,mIdx,param,1.);
  /// joint fixation constraint
  t = MP.addTask("b2_b1_fix", new qItselfConstraint(world->getJointByName("b2_b1")->qIndex,world->getJointStateDimension()));
  t->target = ARR(0.);
  t->prec = prec_inv;

  MotionProblemFunction MPF(MP);
  Xn = Xdemo;

  OptOptions o;
  o.stopTolerance = 1e-4; o.constrainedMethod=anyTimeAula; o.verbose=0;
  o.stepInc = 2.; o.aulaMuInc = 2.; o.maxStep = 1.;
  optConstrained(Xn, NoArr, Convert(MPF), o);

  world->gl().resize(800,800); updateVisualization(*world,Xn); world->gl().update();

  return true;
}


bool ButtonTask::success(const arr &X, const arr &Y) {
  bool result;
  cout << "Enter result:  success [1] or failure [0]: "<<endl;
  std::cin >> result;
  return result;
}

double ButtonTask::reward(const arr &Z){
  arr tmp = Z.col(2); tmp.flatten();
  return exp(-sumOfAbs(tmp)/double(tmp.d0)*0.1);
}
