#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

struct TaskManager
{
  enum TaskType {DOOR=1,GRASP=2,BUTTON=3};
  arr constraintTime;
  arr constraintCP;
  mlr::Array<uint> conStart;
  mlr::Array<uint> conEnd;
  mlr::KinematicWorld *world;
  TaskType type;
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  arr PX1f,PX1c,PX2f,PX2c;
  TaskManager() {};
  uint nParam,nDof;
  virtual void addConstraints(KOMO *MP, const arr &X) = 0;
  virtual void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr) = 0;
  virtual void computeConstraintTime(const arr &F,const arr &X) = 0;
  virtual bool transformTrajectory(arr &Xn, const arr &theta, arr& Xdemo) = 0;
  virtual bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo) = 0;
  virtual bool success(const arrA &X, const arr &Y) = 0;
  virtual double reward(const arr &Z) = 0;
  virtual double cost(const arr &Z) = 0;

  virtual void getParamLimit(arr &paramLimit) = 0;
  virtual void getDofLimit(arr &dofLimit) = 0;

  virtual ~TaskManager() {};
};

struct DoorTask:TaskManager {
  DoorTask(mlr::KinematicWorld &world_) {world = new mlr::KinematicWorld(world_); type = DOOR;}
  void addConstraints(KOMO *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &theta, arr& Xdemo);
  bool success(const arrA &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  double reward(const arr &Z);
  double cost(const arr &Z);
  void getDofLimit(arr &dofLimit) {};
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo) {return true;};

};

struct GraspTask:TaskManager {
  GraspTask(mlr::KinematicWorld &world_) {world = new mlr::KinematicWorld(world_); type = GRASP;}
  void addConstraints(KOMO *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &theta, arr& Xdemo);
  bool success(const arrA &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  double reward(const arr &Z);
  double cost(const arr &Z);
  void getDofLimit(arr &dofLimit) {};
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo) {return true;};
};

struct ButtonTask:TaskManager {
  ButtonTask(mlr::KinematicWorld &world_) {
    world = &world_;//new mlr::KinematicWorld(world_);
    type = BUTTON;
    nParam = 2;
    nDof = 1;
  }
  void addConstraints(KOMO *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &theta, arr& Xdemo);
  bool success(const arrA &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  void getDofLimit(arr &dofLimit);
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo);
  double reward(const arr &Z);
  double cost(const arr &Z);
  void addModelConstraints(KOMO *MP, arr& target);
};

#endif // TASK_MANAGER_H
