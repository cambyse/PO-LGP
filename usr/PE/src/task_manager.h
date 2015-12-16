#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
//#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

struct TaskManager
{
  enum TaskType {DOOR=1,GRASP=2};
  arr constraintTime;
  arr constraintCP;
  mlr::Array<uint> conStart;
  mlr::Array<uint> conEnd;
  ors::KinematicWorld *world;
  TaskType type;
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  arr PX1f,PX1c,PX2f,PX2c;
  TaskManager() {};
  virtual void addConstraints(MotionProblem *MP, const arr &X) = 0;
  virtual void updateVisualization(ors::KinematicWorld &world, arr &X, arr &Y=NoArr) = 0;
  virtual void computeConstraintTime(const arr &F,const arr &X) = 0;
  virtual bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo) = 0;
  virtual bool transformTrajectoryDof(arr &Xn, const arr &x, arr& Xdemo) = 0;
  virtual bool success(const arr &X, const arr &Y) = 0;
  virtual double reward(const arr &Z) = 0;
  virtual void getParamLimit(arr &paramLimit) = 0;
  virtual void getDofLimit(arr &dofLimit) = 0;

  virtual ~TaskManager() {};
};

struct DoorTask:TaskManager {
  DoorTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_); type = DOOR;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  double reward(const arr &Z);
  void getDofLimit(arr &dofLimit) {};
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo) {};

};

struct GraspTask:TaskManager {
  GraspTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_); type = GRASP;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  double reward(const arr &Z);
  void getDofLimit(arr &dofLimit) {};
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo) {};

};

struct ButtonTask:TaskManager {
  ButtonTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_); type = GRASP;}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X, arr &Y=NoArr);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  void getParamLimit(arr &paramLimit);
  void getDofLimit(arr &dofLimit);
  bool transformTrajectoryDof(arr &Xn, const arr &x_dof, arr& Xdemo);
  double reward(const arr &Z);

  void addModelConstraints(MotionProblem *MP, arr& target);
};

#endif // TASK_MANAGER_H
