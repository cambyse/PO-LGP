#pragma once

#include <Core/array.h>
#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Control/ctrlMsg.h>
#include <Core/thread.h>
#include <RosCom/subscribeAlvarMarkers.h>

struct TrajectoryInterface {
  struct sTrajectoryInterface *S;
  arr q,qdot;
  mlr::KinematicWorld *world_robot;
  mlr::KinematicWorld *world_plan;
  bool useRos, fixBase,fixTorso, useMarker;

  CtrlMsg refs;
  ar::AlvarMarkers markers;

  /// logging variables
  arr logXdes,logXref,logXplan,logX,logFL,logFR,logT,logU;
  arrA logM;

  TrajectoryInterface(mlr::KinematicWorld &world_plan_, mlr::KinematicWorld& world_robot_);
  ~TrajectoryInterface(){ threadCloseModules(); }

  /// execute trajectory X in T seconds
  void executeTrajectory(arr &X_robot, double T, bool recordData = false, bool displayTraj=false, bool reverseMotion=false);
  void executeTrajectoryPlan(arr &X_plan, double T, bool recordData = false, bool displayTraj=false, bool reverseMotion=false);

  /// go to robot configuration x
  void gotoPosition(mlr::String filename, double T=5., bool recordData = false, bool displayTraj=false);
  void gotoPosition(arr x_robot, double T=5., bool recordData = false, bool displayTraj=false);
  void gotoPositionPlan(arr x_plan, double T=5., bool recordData = false, bool displayTraj=false);

  /// send zero gains and record trajectory of T seconds
  void recordDemonstration(arr &X_robot, double T, double dt=0.05, double T_start=2.);

  void moveLeftGripper(double d);
  void moveRightGripper(double d);

  /// stop the motion
  void pauseMotion(bool sendZeroGains = false);

  /// save last trajectory to file
  void logging(mlr::String folder, mlr::String name, uint id=0);

  /// get robot state
  void getState(arr& q_robot);
  void getStatePlan(arr& q_plan);

  void syncState();
  void syncMarker();

  /// save robot state
  void saveState(mlr::String filename);
};
