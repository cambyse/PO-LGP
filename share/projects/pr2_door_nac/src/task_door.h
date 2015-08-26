#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

struct DoorTask {
  arr constraintTime;
  arr constraintCP;
  ors::KinematicWorld *world;
  arr Pdemo1f,Pdemo1c,Pdemo2f,Pdemo2c;
  DoorTask(ors::KinematicWorld &world_) {world = new ors::KinematicWorld(world_);}
  void addConstraints(MotionProblem *MP, const arr &X);
  void updateVisualization(ors::KinematicWorld &world, arr &X);
  void computeConstraintTime(const arr &F,const arr &X);
  bool transformTrajectory(arr &Xn, const arr &x, arr& Xdemo);
  bool success(const arr &X, const arr &Y);
  double reward(const arr &Z);
};


#endif // TASK_MANAGER_H
