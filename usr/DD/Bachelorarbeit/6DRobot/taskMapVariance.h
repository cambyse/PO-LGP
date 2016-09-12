#ifndef TASKMAPVARIANCE_H
#define TASKMAPVARIANCE_H

#include <Motion/taskMap.h>
#include <Motion/taskMap_default.h>
#include <Algo/gaussianProcess.h>

struct TaskMapVariance : TaskMap {

  GaussianProcess& gp;
  TaskMap_Default taskMap;

  void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t = -1) { HALT("can only be of higher order") }
  void phi(arr& y, arr& J, const WorldL& G, double tau, int t = -1);
  uint dim_phi(const ors::KinematicWorld& G) { return 1; }

  TaskMapVariance(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName);

};

struct TaskMapGPGradient : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default taskMap;
  TaskMap_Default positionMap;

  void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t = -1);
  uint dim_phi(const ors::KinematicWorld& G) { return 1; }

  TaskMapGPGradient(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector);

};

struct TaskMapGP : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default taskMap;

  void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t = -1);
  uint dim_phi(const ors::KinematicWorld& G) { return 1; }

  TaskMapGP(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName);

};

struct TaskMapGP1D : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMapGP1D(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName);
};

struct TaskMap1DPosOrientation : TaskMap {
  TaskMap_Default orientationMap;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMap1DPosOrientation(const ors::KinematicWorld& world, const char* shapeName, const ors::Vector& vec);
};

#endif // TASKMAPVARIANCE_H
