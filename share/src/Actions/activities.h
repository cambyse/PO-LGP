#include <FOL/activity.h>

struct FollowReferenceActivity : Activity {
  struct TaskControllerModule *taskController;
  struct DefaultTaskMap *map;
  struct CtrlTask* task;
  arr ref;
  double trajectoryDuration; ///< -1 if this is only a point reference instead of a trajectory
  double stopTolerance;
  bool conv;

  FollowReferenceActivity(){}
  ~FollowReferenceActivity();
  virtual void configure(Item *fact);
  virtual void step(RelationalMachine& RM, double dt);
  void write(ostream &os) const;
};
stdOutPipe(FollowReferenceActivity)

struct HomingActivity : Activity {
  struct TaskControllerModule *taskController;
  struct TaskMap *map;
  struct CtrlTask* task;
  double stopTolerance;
  bool conv;

  HomingActivity(){}
  ~HomingActivity();
  virtual void configure(Item *fact);
  virtual void step(RelationalMachine& RM, double dt);
  void write(ostream &os) const;
};
stdOutPipe(HomingActivity)
