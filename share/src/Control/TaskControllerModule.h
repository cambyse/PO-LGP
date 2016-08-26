#pragma once

#include <Core/module.h>
#include <Control/ctrlMsg.h>
#include <Control/taskController.h>
#include <Control/RTControllerSimulation.h>

/// Struct for logging data
struct SetOfDataFiles {

  std::map<mlr::String, ofstream> logMap;

  mlr::Array<ofstream*> files;

  void open(const StringA& names, const char* folderName);
  void write(const arrA& data);

  ~SetOfDataFiles();
};


/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Module {
  struct sTaskControllerModule *s;

  //protected access points
  ACCESS(arr, ctrl_q_real)
  ACCESS(arr, ctrl_q_ref)

  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(mlr::String, effects)
  ACCESS(ors::KinematicWorld, modelWorld)
  ACCESS(bool, fixBase)
  ACCESS(arr, pr2_odom)

//private:
  ors::KinematicWorld realWorld;
  TaskController *taskController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  mlr::String robot;
  bool oldfashioned;
  bool useRos;
  bool requiresInitialSync;
  bool syncModelStateWithReal; //< whether the step() should reinit the state from the ros message
  bool verbose;
  bool useDynSim;
  bool log;
  RTControllerSimulation* dynSim;

  arr q_history, qdot_last, a_last, q_lowPass, qdot_lowPass, qddot_lowPass, aErrorIntegral, u_lowPass;
  arr model_error_g;

  SetOfDataFiles logFiles;

public:
  TaskControllerModule(const char* robot="pr2");
  ~TaskControllerModule();

  void open();
  void step();
  void close();
};
