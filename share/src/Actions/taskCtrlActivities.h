#include <Ors/ors.h>
#include "activity.h"

/// A typical 'control activity' that adds a CtrlTask to the task list of the task controller.
/// Der
struct TaskCtrlActivity : Activity{
  struct TaskControllerModule *taskController;
  struct TaskMap *map;
  struct CtrlTask* task;
  double stopTolerance;
  bool conv;

  ~TaskCtrlActivity();
  virtual void configure(Node *fact); ///< calls configure2 and registers
  virtual void step(double dt); ///< calls step2, then checks for isConv and sets facts accordingly

  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world) = 0;
  virtual void step2(double dt) = 0;
  virtual bool isConv() = 0;
};

//===========================================================================
struct FollowReferenceActivity : TaskCtrlActivity {
  arr ref;
  double trajectoryDuration; ///< -1 if this is only a point reference instead of a trajectory
  double stopTolerance;

  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt);
  virtual bool isConv();
};

//===========================================================================
struct HomingActivity : TaskCtrlActivity {
  double stopTolerance;

  virtual void configure2(const char *name, Graph& specs, ors::KinematicWorld& world);
  virtual void step2(double dt){}
  virtual bool isConv();
};
stdOutPipe(HomingActivity)


