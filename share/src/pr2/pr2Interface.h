#ifndef PR2INTERFACE_H
#define PR2INTERFACE_H

#include <Core/module.h>
#include <Core/array.h>
#include <pr2/roscom.h>
#include <Ors/ors.h>
#include <pr2/pr2DynamicSimulation.h>
#include <Control/taskSpaceController.h>
#include <pr2/rosalvar.h>

struct PR2Interface : Module {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(AlvarMarkers, ar_pose_markers)

  ors::KinematicWorld* realWorld;
  ors::KinematicWorld* modelWorld;

  DynamicSimulation* dynamicSimulation;

  TaskSpaceController* controller;

  CtrlMsg ctrlMsg;

  bool logState = true;
  arr logT, logQRef, logQObs, logQDotRef, logQDotObs, logUObs, logU0, logKp, logKd, logFLObs, logFRObs, logKiFt, logJ_ft_inv, logFRef;
  std::map<mlr::String, arr> logMap;

  arr lGripperRef, rGripperRef, torsoLiftRef;

  bool useROS = false;

  PR2Interface();
  ~PR2Interface() {threadCloseModules();}
  virtual void step();

  void initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* realWorldSimulation, ors::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void startInterface();
  void sendCommand(const arr& u0, const arr& Kp, const arr& Kd, const arr& K_ft, const arr& J_ft_inv, const arr& fRef, const double& gamma);
  void goToPosition(arr pos, double executionTime = 10.0);
  void goToTasks(mlr::Array<LinTaskSpaceAccLaw*> laws, double executionTime = 10.0, bool useMotionPlanner = true);
  void goToTask(TaskMap* map, arr ref, double executionTime = 10.0);
  void goToJointState(arr jointState, double executionTime = 10.0);
  void executeTrajectory(double executionTime);
  void moveTorsoLift(arr torsoLiftRef);
  void moveLGripper(arr lGripperRef);
  void moveRGripper(arr rGripperRef);
  void logStateSave(mlr::String name = "noname", mlr::String folder = "data");
  void clearLog();
};

void showTrajectory(const arr& traj, ors::KinematicWorld& _world, bool copyWorld = true, double delay = 0.03, mlr::String text = "");

#endif // PR2INTERFACE_H
