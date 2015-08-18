#pragma once

#include <Ors/ors.h>
#include "activity.h"

// ============================================================================
/// A typical 'control activity' that adds a CtrlTask to the task list of the task controller.
/// Der
struct ControlActivity : Activity{
  struct ControlActivityManager *controlManager;
  struct TaskMap *map;
  struct CtrlTask* task;
  double stopTolerance;
  bool conv;

  virtual ~ControlActivity();

  // Implement base class methods: modify KB
  virtual void configure(Node *fact); ///< calls configureControl and registers
  virtual void step(double dt); ///< calls stepControl, then checks for isConv and sets facts accordingly

  // actually do control related stuff
  virtual void configureControl(const char *name, Graph& specs, ors::KinematicWorld& world) = 0;
  virtual void stepControl(double dt) = 0;
  virtual bool isConv() = 0;
};

//===========================================================================
struct FollowReferenceActivity : ControlActivity {
  arr ref;
  arr old_y;
  uint stuck_count;

  double trajectoryDuration; ///< -1 if this is only a point reference instead of a trajectory
  double stopTolerance;

  virtual void configureControl(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void stepControl(double dt);
  virtual bool isConv();
};

//===========================================================================
struct HomingActivity : ControlActivity {
  double stopTolerance;
  ors::Joint *wheeljoint;

  virtual void configureControl(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void stepControl(double dt);
  virtual bool isConv();
};
stdOutPipe(HomingActivity)


