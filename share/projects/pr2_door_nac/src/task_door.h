#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>

struct DoorTask {
  arr constraintTime;
  arr constraintCP;
  mlr::KinematicWorld *world;
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  DoorTask(mlr::KinematicWorld &world_) {world = new mlr::KinematicWorld(world_);}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(mlr::KinematicWorld &world, arr &X);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
  ~DoorTask(){
     delete world;
      world = NULL;
  }
};


#endif // TASK_MANAGER_H
