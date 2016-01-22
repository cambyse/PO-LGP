#ifndef PR2INTERFACE_H
#define PR2INTERFACE_H

#include <Core/module.h>
#include <Core/array.h>
#include <pr2/roscom.h>
#include <Ors/ors.h>
#include <pr2/pr2DynamicSimulation.h>
#include <Control/taskSpaceController.h>

struct PR2Interface : Module {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)

  ors::KinematicWorld* realWorld;
  ors::KinematicWorld* modelWorld;

  DynamicSimulation* dynamicSimulation;

  TaskSpaceController* controller;

  CtrlMsg ctrlMsg;

  bool logState = true;
  arr logQRef, logQObs, logQDotRef, logQDotObs, logU0, logKp, logKd, logFLObs, logFRObs;
  std::map<mlr::String, arr> logMap;

  bool useROS = false;

  PR2Interface();
  ~PR2Interface() {threadCloseModules();}
  virtual void step();

  void initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* realWorldSimulation, ors::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void initialize(ors::KinematicWorld* realWorld, ors::KinematicWorld* modelWorld, TaskSpaceController* controller = NULL);
  void startInterface();
  void sendCommand(arr u0, arr Kp, arr Kd);
  void goToPosition(arr pos, double executionTime = 10.0);
  void goToTasks(mlr::Array<LinTaskSpaceAccLaw*> laws, double executionTime = 10.0, bool useMotionPlanner = true);
  void goToTask(TaskMap* map, arr ref, double executionTime = 10.0);
  void goToJointState(arr jointState, double executionTime = 10.0);
  void executeTrajectory(double executionTime);
  void logStateSave(mlr::String name = "", mlr::String folder = "data/");
  void clearLog();
};

void showTrajectory(const arr& traj, ors::KinematicWorld& _world, bool copyWorld = true, double delay = 0.03, mlr::String text = "");

#endif // PR2INTERFACE_H
