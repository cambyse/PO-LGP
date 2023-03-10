#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

struct TaskManager
{
  uint paramDim;
  arr paramLimit;
  uint T;
  double tau;
  mlr::KinematicWorld *world;
  mlr::String name;
  arr constraintTime;
  uint gridSize;
  arr Xdemo;
  arr Cdemo;

  TaskManager() {};
  virtual void initTask(mlr::KinematicWorld &world_, arr &Xdemo_) = 0;
  virtual void createSynthethicDemonstration(arr &X,mlr::KinematicWorld &world) = 0;
  virtual void applyModelFreeExploration(arr &X,const arr &X_base,const arr &param) = 0;
  virtual void addConstraintTaskMaps(KOMO &MP, arr param) = 0;
  virtual void computeCostGrid(arr &F_grid,const arr &param_grid, arr &X) = 0;
  virtual double rewardFunction(const arr &X) = 0;
  virtual ~TaskManager() {};
};

struct DonutTask:TaskManager {
  DonutTask() {
    name = "donut";
    paramDim = 1;
    paramLimit = ARR(-.5,.5);
    T = 100;
    tau = 0.05;
    constraintTime = ARR(T/2-2,T/2+2);
    gridSize = 100;
    arr Xdemo;
  }
  virtual void initTask(mlr::KinematicWorld &world_, arr &Xdemo_) {};
  virtual void createSynthethicDemonstration(arr &X,mlr::KinematicWorld &world);
  virtual void applyModelFreeExploration(arr &X, const arr &X_base, const arr &param);
  virtual void addConstraintTaskMaps(KOMO &MP, arr param);
  virtual void computeCostGrid(arr &F_grid,const arr &param_grid, arr &X_base);
  virtual double rewardFunction(const arr &X);
};

struct DoorTask:TaskManager {
  DoorTask () {
    name = "door1";
    paramDim = 1;
    paramLimit = ARR(-0.4,.4);
    tau = 0.05;
    gridSize = 50;
  }
  virtual void initTask(mlr::KinematicWorld &world_, arr &Xdemo_);
  virtual void createSynthethicDemonstration(arr &X,mlr::KinematicWorld &world);
  virtual void applyModelFreeExploration(arr &X, const arr &X_base, const arr &param);
  virtual void addConstraintTaskMaps(KOMO &MP, arr param);
  virtual void computeCostGrid(arr &F_grid,const arr &param_grid, arr &X_base);
  virtual double rewardFunction(const arr &X);
};
#endif // TASK_MANAGER_H
