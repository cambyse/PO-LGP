#ifndef TASKMAPVARIANCE_H
#define TASKMAPVARIANCE_H

#include <Motion/taskMap.h>
#include <Motion/taskMap_default.h>
#include <Algo/gaussianProcess.h>
#include <Core/module.h>

struct TaskMapVariance : TaskMap {

  GaussianProcess& gp;
  TaskMap_Default taskMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1) { HALT("can only be of higher order") }
  void phi(arr& y, arr& J, const WorldL& G, double tau, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMapVariance(GaussianProcess& gp, const mlr::KinematicWorld& world, const char* shapeName);

};

struct TaskMapGPGradient : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default taskMap;
  TaskMap_Default positionMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMapGPGradient(GaussianProcess& gp, const mlr::KinematicWorld& world, const char* shapeName, mlr::Vector vector);

};

struct TaskMapGPGradientThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default taskMap;
  TaskMap_Default positionMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMapGPGradientThread(Access<GaussianProcess>& gp, const mlr::KinematicWorld& world, const char* shapeName, mlr::Vector vector);

};


struct TaskMapGP : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default taskMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMapGP(GaussianProcess& gp, const mlr::KinematicWorld& world, const char* shapeName);

};

struct TaskMapGP1D : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t);
  uint dim_phi(const mlr::KinematicWorld &G) { return 1; }

  TaskMapGP1D(GaussianProcess& gp, const mlr::KinematicWorld& world, const char* shapeName);
};

struct TaskMapGP1DThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t);
  uint dim_phi(const mlr::KinematicWorld &G) { return 1; }

  TaskMapGP1DThread(Access<GaussianProcess>& gp, const mlr::KinematicWorld& world, const char* shapeName);
};

struct TaskMapGPVariance1DThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t);
  uint dim_phi(const mlr::KinematicWorld &G) { return 1; }

  TaskMapGPVariance1DThread(Access<GaussianProcess>& gp, const mlr::KinematicWorld& world, const char* shapeName);
};

struct TaskMap1DPosOrientation : TaskMap {
  TaskMap_Default orientationMap;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const mlr::KinematicWorld &G, int t);
  uint dim_phi(const mlr::KinematicWorld &G) { return 1; }

  TaskMap1DPosOrientation(const mlr::KinematicWorld& world, const char* shapeName, const mlr::Vector& vec);
};

#endif // TASKMAPVARIANCE_H
