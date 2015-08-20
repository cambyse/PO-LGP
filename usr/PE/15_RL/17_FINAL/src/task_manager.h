#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

struct TaskManager
{
  enum TaskType {DOOR=1,GRASP=2};
  arr constraintTime;
  arr constraintCP;
  ors::KinematicWorld *world;
  TaskType type;

  TaskManager() {};
  virtual void addConstraints(MotionProblem *MP, const arr &X) = 0;
  virtual void updateVisualization(ors::KinematicWorld &world, arr &X) = 0;
  virtual void computeConstraintTime(const arr &F,const arr &X) = 0;
  virtual bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo) = 0;
  virtual bool success(const arr &X, const arr &Y) = 0;
  virtual double reward(const arr &Z) = 0;
  virtual ~TaskManager() {};
};

struct DoorTask:TaskManager {
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  DoorTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_); type = DOOR;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
};

struct GraspTask:TaskManager {
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  GraspTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_); type = GRASP;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
};

#endif // TASK_MANAGER_H
