#pragma once

#include <Kin/kin.h>
#include "activity.h"

// ============================================================================
/// A typical 'control activity' that adds a CtrlTask to the task list of the task controller.
/// Der
struct ControlActivity : Activity{
  struct TaskControllerModule *taskController;
  struct TaskMap *map;
  struct CtrlTask* task;
  double stopTolerance;
  bool conv;

  ControlActivity():taskController(NULL), map(NULL), task(NULL), stopTolerance(1e-2), conv(false){}
  virtual ~ControlActivity();
  virtual void configure(); ///< calls configure2 and registers
  virtual void activitySpinnerStep(double dt); ///< calls step2, then checks for isConv and sets facts accordingly

  virtual void configureControl(const char *name, Graph& specs, mlr::KinematicWorld& world) = 0;
  virtual void stepControl(double dt){}
  virtual bool isConv();
};

//===========================================================================

struct FollowReferenceActivity : ControlActivity {
  arr ref;
  arr old_y;
  uint stuck_count;
  double trajectoryDuration; ///< -1 if this is only a point reference instead of a trajectory

  FollowReferenceActivity(){}
  FollowReferenceActivity(const Graph&){ NIY; }
  virtual void configureControl(const char *name, Graph& specs, mlr::KinematicWorld& world);
  virtual void stepControl(double dt);
  virtual bool isConv();
};

//===========================================================================

//struct PushForce : ControlActivity {
//  arr forceVec;
//  PushForce(ActionMachine& actionMachine, const char* effName, arr forceVec, double timeOut=-1.);
//  virtual void step(ActionMachine& M);
//  virtual bool finishedSuccess(ActionMachine& M);
//};

//===========================================================================

struct HomingActivity : ControlActivity {
  mlr::Joint *wheeljoint;

  virtual void configureControl(const char *name, Graph& specs, mlr::KinematicWorld& world);
  virtual void stepControl(double dt);
  virtual bool isConv();
};
stdOutPipe(HomingActivity)
