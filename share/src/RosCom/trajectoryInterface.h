#pragma once

#include <Core/array.h>
#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Control/ctrlMsg.h>
#include <Core/module.h>

struct MySystem {
};

struct TrajectoryInterface {
  struct sTrajectoryInterface *S;
  arr q,qdot;
  ors::KinematicWorld *world_pr2;
  ors::KinematicWorld *world_plan;
  bool useRos, fixBase,fixTorso;

  CtrlMsg refs;

  /// logging variables
  arr logXdes,logXref,logX,logFL,logFR,logT,logU,logM;

  TrajectoryInterface(ors::KinematicWorld &world_, ors::KinematicWorld& world_pr2_);
  ~TrajectoryInterface(){ threadCloseModules(); }

  /// execute trajectory X in T seconds
  void executeTrajectory(arr &X_pr2, double T, bool recordData = false, bool displayTraj=false);
  void executeTrajectoryPlan(arr &X_plan, double T, bool recordData = false, bool displayTraj=false);

  /// go to robot configuration x
  void gotoPosition(arr x_pr2, double T=5., bool recordData = false, bool displayTraj=false);
  void gotoPositionPlan(arr x_plan, double T=5., bool recordData = false, bool displayTraj=false);

  /// send zero gains and record trajectory of T seconds
  void recordDemonstration(arr &X_pr2, double T, double dt=0.05, double T_start=2.);

  void moveLeftGripper(double d);
  void moveRightGripper(double d);


  /// stop the motion
  void pauseMotion(bool sendZeroGains = false);

  /// save last trajectory to file
  void logging(mlr::String folder, uint id=0);

  /// get robot state
  void getState(arr& q_pr2);
  void getStatePlan(arr& q_plan);
};
