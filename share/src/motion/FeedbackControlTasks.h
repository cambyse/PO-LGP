#ifndef motion_FeedbackControlTasks_h
#define motion_FeedbackControlTasks_h

#include <MT/ors.h>
struct JoystickState;
struct SkinPressure;

//===========================================================================
//
// FeedbackControlTasks
//

struct FeedbackControlTaskAbstraction {
  TaskVariableList TVs;
  bool requiresInit;
  FeedbackControlTaskAbstraction():requiresInit(true) {}
  virtual void initTaskVariables(const ors::Graph& ors)=0; ///< reactive update of the task variables' goals
  virtual void updateTaskVariableGoals(const ors::Graph& ors)=0; ///< reactive update of the task variables' goals
};


#define _FeedbackRobotTask(name) \
  struct name##_FeedbackControlTask:public FeedbackControlTaskAbstraction { \
    ~name##_FeedbackControlTask(){ listDelete(TVs); } \
    virtual void initTaskVariables(const ors::Graph& ors); \
    virtual void updateTaskVariableGoals(const ors::Graph& ors);   };

_FeedbackRobotTask(DoNothing)
_FeedbackRobotTask(Stop)
_FeedbackRobotTask(Homing)
_FeedbackRobotTask(OpenHand)

struct CloseHand_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  SkinPressure *skinPressure;
  ~CloseHand_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::Graph& ors);
  virtual void updateTaskVariableGoals(const ors::Graph& ors);
};

struct Reach_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  arr reachPoint;
  ~Reach_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::Graph& ors);
  virtual void updateTaskVariableGoals(const ors::Graph& ors);
};

struct Joystick_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  JoystickState *joyState;
  SkinPressure *skinPressure;
  double joyRate;
  ~Joystick_FeedbackControlTask() { listDelete(TVs); }
  virtual void initTaskVariables(const ors::Graph& ors);
  virtual void updateTaskVariableGoals(const ors::Graph& ors);
};

#endif