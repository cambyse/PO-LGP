#ifndef motion_FeedbackControlTasks_h
#define motion_FeedbackControlTasks_h

#include <Ors/ors.h>

struct JoystickState;
struct SkinPressure;

//===========================================================================
//
// FeedbackControlTasks
//

//TODO: feedback controllers must have a stopping condition: when to stop the feedback motion primitive!

struct FeedbackControlTaskAbstraction {
  TaskVariableList TVs;
  bool requiresInit;
  bool done;
  uint count;
  FeedbackControlTaskAbstraction():requiresInit(true), done(false), count(0) {}
  virtual void initTaskVariables(const ors::KinematicWorld& ors)=0; ///< reactive update of the task variables' goals
  virtual void updateTaskVariableGoals(const ors::KinematicWorld& ors)=0; ///< reactive update of the task variables' goals
};


#define _FeedbackRobotTask(name) \
  struct name##_FeedbackControlTask:public FeedbackControlTaskAbstraction { \
    ~name##_FeedbackControlTask(){ listDelete(TVs); } \
    virtual void initTaskVariables(const ors::KinematicWorld& ors); \
    virtual void updateTaskVariableGoals(const ors::KinematicWorld& ors);   };

_FeedbackRobotTask(DoNothing)
_FeedbackRobotTask(Stop)
_FeedbackRobotTask(Homing)
_FeedbackRobotTask(OpenHand)

struct CloseHand_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  SkinPressure *skinPressure;
  ~CloseHand_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::KinematicWorld& ors);
  virtual void updateTaskVariableGoals(const ors::KinematicWorld& ors);
};

struct Reach_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  arr reachPoint;
  ~Reach_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::KinematicWorld& ors);
  virtual void updateTaskVariableGoals(const ors::KinematicWorld& ors);
};

struct Joystick_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  JoystickState *joyState;
  SkinPressure *skinPressure;
  double joyRate, defaultEff_vprec;
  ~Joystick_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::KinematicWorld& ors);
  virtual void updateTaskVariableGoals(const ors::KinematicWorld& ors);
};

#endif
