#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

struct TaskManager
{
  enum TaskType {DOOR=1,GRASP=2};
  arr constraintTime;
  arr constraintCP;
  mlr::KinematicWorld *world;
  TaskType type;
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  arr PX1f,PX1c,PX2f,PX2c;
  TaskManager() {};
  virtual void addConstraints(MotionProblem *MP, const arr &X) = 0;
  virtual void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr) = 0;
  virtual void computeConstraintTime(const arr &F,const arr &X) = 0;
  virtual bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo) = 0;
  virtual bool success(const arr &X, const arr &Y) = 0;
  virtual double reward(const arr &Z) = 0;
  virtual ~TaskManager() {};
};

struct DoorTask:TaskManager {
  DoorTask(mlr::KinematicWorld &world_) {world = new mlr::KinematicWorld(world_); type = DOOR;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
};

struct GraspTask:TaskManager {
  GraspTask(mlr::KinematicWorld &world_) {world = new mlr::KinematicWorld(world_); type = GRASP;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
};

#endif // TASK_MANAGER_H
