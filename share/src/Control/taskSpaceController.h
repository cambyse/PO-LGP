#ifndef TASKSPACECONTROLLER_H
#define TASKSPACECONTROLLER_H

#include <Ors/ors.h>
#include <Motion/taskMaps.h>
#include <Core/array.h>
#include <pr2/roscom.h>
#include <Algo/spline.h>
#include <Core/module.h>


struct LinTaskSpaceAccLaw {
  TaskMap* map;

  ors::KinematicWorld* world;

  mlr::String name;


  arr yRef;
  arr yDotRef;
  arr yDDotRef;

  arr Kp;
  arr Kd;
  arr C;

  arr trajectory;
  arr trajectoryDot;
  arr trajectoryDDot;

  mlr::Spline* trajectorySpline;
  mlr::Spline* trajectoryDotSpline;
  mlr::Spline* trajectoryDDotSpline;

  bool trajectoryActive = false;
  bool trajectoryDotActive = false;
  bool trajectoryDDotActive = false;

  LinTaskSpaceAccLaw(TaskMap* map, ors::KinematicWorld* world, mlr::String name = "") : map(map), world(world), name(name) {this->setRef();} // TODO is this the best Way

  void setRef(const arr& yRef = NoArr, const arr& yDotRef = NoArr, const arr& yDDotRef = NoArr);

  void setGains(arr Kp, arr Kd);

  void setC(arr C);

  void setTrajectory(uint trajLength, const arr& traj = NoArr, const arr& trajDot = NoArr, const arr& trajDDot = NoArr);
  void setSpline(mlr::Spline* yS = NULL, mlr::Spline* yDotS = NULL, mlr::Spline* yDDotS = NULL);

  void setTargetEvalSpline(double s);

  void setTrajectoryActive(bool active); // TODO

  arr getPhi();
  void getPhi(arr& y, arr& J);
  uint getPhiDim();

  arr getC();
  arr getKp();
  arr getKd();

  void getRef(arr& yRef, arr& yDotRef, arr& yDDotRef);
  arr getRef();
  arr getDotRef();
  arr getDDotRef();

  bool getTrajectoryActive(); //TODO
  bool getTrajectoryDotActive(); //TODO
  bool getTrajectoryDDotActive(); //TODO

  double getCosts();

};


struct TaskSpaceController {
  mlr::Array<LinTaskSpaceAccLaw*> taskSpaceAccLaws;
  ors::KinematicWorld* world;

  bool gravity = false;

  TaskSpaceController(ors::KinematicWorld* world) : world(world) {}
  ~TaskSpaceController() {}

  void addLinTaskSpaceAccLaw(LinTaskSpaceAccLaw* law);
  void calcOptimalControlProjected(arr& Kp, arr& Kd, arr& u0);
  void generateTaskSpaceTrajectoryFromJointSpace(const arr& jointSpaceTrajectory, const arr& jointSpaceTrajectoryDot = NoArr, const arr& jointSpaceDDotTrajectory = NoArr);
  void generateTaskSpaceSplines();

  void setGravity(bool gravity);
};




#endif // TASKSPACECONTROLLER_H
