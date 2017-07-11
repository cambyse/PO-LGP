#ifndef TASKMAPGPISP_H
#define TASKMAPGPISP_H

#include <Kin/taskMap.h>
#include <Kin/taskMap_default.h>
#include <Algo/gpOp.h>




struct TaskMap_GPISP : TaskMap {
  GaussianProcessOptimized& gp;
  TaskMap_Default posMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMap_GPISP(GaussianProcessOptimized& gp, const mlr::KinematicWorld& world, const char* shapeName);
};


struct TaskMap_GPISPNormalOrientation : TaskMap {
  GaussianProcessOptimized& gp;
  TaskMap_Default posMap;
  TaskMap_Default oriMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }

  TaskMap_GPISPNormalOrientation(GaussianProcessOptimized& gp, const mlr::KinematicWorld& world, const char* shapeName, mlr::Vector vector);
};

//=====================================================================================================

struct TaskMap_GPSIPVarianceGeodesic : TaskMap {
  GaussianProcessOptimized& gp;
  TaskMap_Default posMap;

  void phi(arr& y, arr& J, const mlr::KinematicWorld& G, int t = -1) { HALT("can only be of higher order") }
  void phi(arr& y, arr& J, const WorldL& G, double tau, int t = -1);
  uint dim_phi(const mlr::KinematicWorld& G) { return 1; }
  uint dim_phi(const WorldL& G, int t) { return 1; }

  TaskMap_GPSIPVarianceGeodesic(GaussianProcessOptimized& gp, const mlr::KinematicWorld& world, const char* shapeName);
};




#if 0
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

struct TaskMapGPGradientThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default taskMap;
  TaskMap_Default positionMap;

  void phi(arr& y, arr& J, const ors::KinematicWorld& G, int t = -1);
  uint dim_phi(const ors::KinematicWorld& G) { return 1; }

  TaskMapGPGradientThread(Access<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName, ors::Vector vector);

};

struct TaskMapGP1D : TaskMap {
  GaussianProcess& gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMapGP1D(GaussianProcess& gp, const ors::KinematicWorld& world, const char* shapeName);
};

struct TaskMapGP1DThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMapGP1DThread(Access<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName);
};

struct TaskMapGPVariance1DThread : TaskMap {
  Access<GaussianProcess> gp;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMapGPVariance1DThread(Access<GaussianProcess>& gp, const ors::KinematicWorld& world, const char* shapeName);
};

struct TaskMap1DPosOrientation : TaskMap {
  TaskMap_Default orientationMap;
  TaskMap_Default positionMap;

  void phi(arr &y, arr &J, const ors::KinematicWorld &G, int t);
  uint dim_phi(const ors::KinematicWorld &G) { return 1; }

  TaskMap1DPosOrientation(const ors::KinematicWorld& world, const char* shapeName, const ors::Vector& vec);
};


#endif

#endif // TASKMAPGPISP_H
