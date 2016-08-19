#include "task_manager.h"
#include "../src/traj_factory.h"
#include <Motion/pr2_heuristics.h>
//#include "../src/plotUtil.h"


void DonutTask::createSynthethicDemonstration(arr &X, ors::KinematicWorld &world_){
  world = new ors::KinematicWorld(world_);
  MotionProblem MP(world_,false);
  MP.T = T;
  MP.tau = tau;
  MP.x0 = zeros(world_.getJointStateDimension());

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(world_));
  t->map.order=1;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-1);

  t =MP.addTask("posC", new DefaultTaskMap(posTMT,MP.world,"endeff"));
  arr target = ARR(MP.world.getBodyByName("target0")->X.pos);
  t->setCostSpecs(constraintTime(0),constraintTime(1), target, 1e2);

  t =MP.addTask("posT", new DefaultTaskMap(posTMT,world_,"endeff"));
  t->setCostSpecs(MP.T,MP.T, ARR(world_.getBodyByName("target1")->X.pos), 1e2);

  MotionProblemFunction MPF(MP);
  X = MP.getInitialization();
  OptOptions o; //o.maxStep = 1.;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0; o.aulaMuInc=1.1;
  optConstrainedMix(X, NoArr, Convert(MPF), o);
}

void DonutTask::applyModelFreeExploration(arr &X, const arr &X_base, const arr &param){
  /// transform trajectory in feature space
  TrajFactory *tf = new TrajFactory();
  arr Y_base;
  tf->compFeatTraj(X_base,Y_base,*world,new DefaultTaskMap(posTMT,*world,"endeff"));
  arr Y = Y_base;
  for (uint t=0; t<constraintTime(0); t++){
    Y[t] = Y[t] + t/constraintTime(0)*ARR(0.,0.,param(0));
  }
  for (uint t=constraintTime(0);t<constraintTime(1);t++){
    Y[t] = Y[t] + ARR(0.,0.,param(0));
  }
  for (uint t=constraintTime(1);t<T;t++){
    Y[t] = Y[t] + (1.-(t-constraintTime(1))/(T-constraintTime(1)))*ARR(0.,0.,param(0));
  }

  MotionProblem MP(*world,false);
  MP.T = T;
  MP.tau = tau;
  MP.x0 = zeros(world->getJointStateDimension());

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);

  t =MP.addTask("posT", new DefaultTaskMap(posTMT,*world,"endeff"));
  t->setCostSpecs(0,MP.T, Y, 1e2);

  MotionProblemFunction MPF(MP);
  if (X.N == 0) {X = MP.getInitialization();}
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0;
  optConstrainedMix(X, NoArr, Convert(MPF), o);
}



void DonutTask::addConstraintTaskMaps(MotionProblem &MP, arr param)
{
  Task *t;
  t =MP.addTask("posC", new DefaultTaskMap(posTMT,MP.world,"endeff"));
  arr target = ARR(MP.world.getBodyByName("target0")->X.pos) + ARR(0.,0.,param(0));
  t->setCostSpecs(constraintTime(0),constraintTime(1), target, 1e2);
}

void DonutTask::computeCostGrid(arr &F_grid,const arr &param_grid, arr &X_base){
  /// create Fgrid
  F_grid.resize(param_grid.d0);
  arr X=X_base;
  for (uint i = 0;i<param_grid.d0;i++) {
    applyModelFreeExploration(X,X_base,param_grid[i]);
    F_grid(i) = rewardFunction(X);
  }
}

double DonutTask::rewardFunction(const arr &X)
{
  TrajFactory *tf = new TrajFactory();
  arr Y;
  tf->compFeatTraj(X,Y,*world,new DefaultTaskMap(posTMT,*world,"endeff"));
  arr A;
  getAcc(A,Y,0.1);
  return -sumOfAbs(A);
}

void DoorTask::initTask(ors::KinematicWorld &world_, arr &Xdemo_)
{
  Xdemo = Xdemo_;
  world = new ors::KinematicWorld(world_);
  T = Xdemo.d0-1;
  uint qI = world->getJointByName("l_gripper_joint")->qIndex;
  Cdemo = zeros(Xdemo.d0); Cdemo.flatten();

  // TODO: check if contact data is greater than 0
  for (uint t=0;t<Xdemo.d0;t++) {
    if (Xdemo(t,qI)<0.04) {
      Cdemo(t) = 1.;
      if (constraintTime.N ==0) {
        constraintTime = ARR(t,Xdemo.d0-1);
      }
    }
  }
  cout <<"Cdemo: " << ~Cdemo << endl;
}

void DoorTask::createSynthethicDemonstration(arr &X, ors::KinematicWorld &world)
{

}

void DoorTask::applyModelFreeExploration(arr &X, const arr &X_base, const arr &param)
{
  /// transform trajectory in feature space
  TrajFactory *tf = new TrajFactory();
  arr Y1_base,Y2_base;
  tf->compFeatTraj(X_base,Y1_base,*world,new DefaultTaskMap(posTMT,*world,"endeffC1"));
  tf->compFeatTraj(X_base,Y2_base,*world,new DefaultTaskMap(posTMT,*world,"endeffC2"));

  arr Y1 = Y1_base;
  arr Y2 = Y2_base;
  arr offset = ARR(param(0),0.,0.);

  world->setJointState(X_base[constraintTime(0)]);
  arr R1 = world->getShapeByName("endeffC1")->X.rot.getArr();
  arr R2 = world->getShapeByName("endeffC2")->X.rot.getArr();

  for (uint t=0; t<constraintTime(0); t++){
    /// linear transition from trajectory to contact point
    Y1[t] = Y1[t] + t/constraintTime(0)*R1*offset;
    Y2[t] = Y2[t] - t/constraintTime(0)*R2*offset;
  }
  for (uint t=constraintTime(0);t<=constraintTime(1);t++){
    world->setJointState(X_base[t]);
    R1 = world->getShapeByName("endeffC1")->X.rot.getArr();
    Y1[t] = Y1[t] + R1*offset;
    R2 = world->getShapeByName("endeffC2")->X.rot.getArr();
    Y2[t] = Y2[t] - R2*offset;
  }
  //  for (uint t=constraintTime(1);t<T;t++){
  //    Y[t] = Y[t] + (1.-(t-constraintTime(1))/(T-constraintTime(1)))*ARR(0.,0.,param(0));
  //  }

  MotionProblem MP(*world,false);
  MP.T = T;
  MP.tau = tau;
  MP.x0 = X_base[0];

  //--tasks
  Task *t;
  t = MP.addTask("tra", new TransitionTaskMap(*world));
  t->map.order=2;
  t->setCostSpecs(0, MP.T, ARR(0.), 1e-2);
  ((TransitionTaskMap*)&t->map)->H_rate_diag = pr2_reasonable_W(*world);


  t =MP.addTask("posC1", new DefaultTaskMap(posTMT,*world,"endeffC1"));
  t->setCostSpecs(0,MP.T, Y1, 1e2);
  t =MP.addTask("posC2", new DefaultTaskMap(posTMT,*world,"endeffC2"));
  t->setCostSpecs(0,MP.T, Y2, 1e2);

  MotionProblemFunction MPF(MP);
  if (X.N == 0) {X = MP.getInitialization();}
  OptOptions o;
  o.stopTolerance = 1e-3; o.constrainedMethod=anyTimeAula; o.verbose=0;
  optConstrainedMix(X, NoArr, Convert(MPF), o);
}

void DoorTask::addConstraintTaskMaps(MotionProblem &MP, arr param)
{
  /// compute feature trajectory
  TrajFactory tf;
  arr vC1,vC2;
  tf.compFeatTraj(Xdemo,vC1,MP.world,new DefaultTaskMap(posTMT,MP.world,"endeffC1"));
  tf.compFeatTraj(Xdemo,vC2,MP.world,new DefaultTaskMap(posTMT,MP.world,"endeffC2"));

  arr zC1 = vC1;
  arr zC2 = vC2;
  arr offset = ARR(param(0),0.,0.);
  arr prec = Cdemo*1e3;
  for (uint t=0;t<Xdemo.d0;t++) {
    MP.world.setJointState(Xdemo[t]);
    arr R = MP.world.getShapeByName("endeffC1")->X.rot.getArr();
    zC1[t] = zC1[t] + R*offset;
    R = MP.world.getShapeByName("endeffC2")->X.rot.getArr();
    zC2[t] = zC2[t] + R*offset;
  }
  Task *t;
  t = MP.addTask("posC1", new DefaultTaskMap(posTMT,MP.world,"endeffC1"));
  t->target = zC1;
  t->prec = prec;
  t = MP.addTask("posC2", new DefaultTaskMap(posTMT,MP.world,"endeffC2"));
  t->target = zC2;
  t->prec = prec;
}

void DoorTask::computeCostGrid(arr &F_grid, const arr &param_grid, arr &X_base)
{
  /// create Fgrid
  F_grid.resize(param_grid.d0);
  arr X=X_base;
  for (uint i = 0;i<param_grid.d0;i++) {
    applyModelFreeExploration(X,X_base,param_grid[i]);
    F_grid(i) = rewardFunction(X);
  }
}

double DoorTask::rewardFunction(const arr &X)
{
  TrajFactory *tf = new TrajFactory();
  arr Y;
  tf->compFeatTraj(X,Y,*world,new DefaultTaskMap(posTMT,*world,"endeffC1"));
  arr A;
  getAcc(A,X,0.1);
  return -sumOfAbs(A);
}
