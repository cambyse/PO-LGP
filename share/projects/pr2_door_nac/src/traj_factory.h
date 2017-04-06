#ifndef TRAJ_FACTORY_H
#define TRAJ_FACTORY_H

#include <Core/array.h>
#include <Kin/kin.h>
#include <Kin/taskMaps.h>

struct TrajFactory{

  TrajFactory(){};

  void transform(const arr &y, const arr &trans, arr &y_trans, double s_mu, double s_std);

  /// transform a trajectory from joint space into a feature space defined by a TaskMap
  void compFeatTraj(const arr &x, arr &y, mlr::KinematicWorld &world, TaskMap *tm);

  /// transform a trajectory from feature space into joint space
  void compJointTraj(const arr &xInit, const arr &y, arr &x, MotionProblem &MP, TaskMap *tm);

};

#endif // TRAJ_FACTORY_H
