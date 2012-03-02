#ifndef motion_FeedbackControlTasks_h
#define motion_FeedbackControlTasks_h

#include <MT/ors.h>

//===========================================================================
//
// FeedbackControlTasks
//

struct FeedbackControlTaskAbstraction {
  TaskVariableList TVs;
  bool requiresInit;
  FeedbackControlTaskAbstraction():requiresInit(true) {}
  virtual void initTaskVariables(const ors::Graph& ors, const arr& skinState)=0; ///< reactive update of the task variables' goals
  virtual void updateTaskVariableGoals(const ors::Graph& ors, const arr& skinState)=0; ///< reactive update of the task variables' goals
};


#define _FeedbackRobotTask(name) \
  struct name##_FeedbackControlTask:public FeedbackControlTaskAbstraction { \
    ~name##_FeedbackControlTask(){ listDelete(TVs); } \
    virtual void initTaskVariables(const ors::Graph& ors, const arr& skinState); \
    virtual void updateTaskVariableGoals(const ors::Graph& ors, const arr& skinState);   };

_FeedbackRobotTask(DoNothing)
_FeedbackRobotTask(Stop)
_FeedbackRobotTask(Homing)
_FeedbackRobotTask(OpenHand)
_FeedbackRobotTask(CloseHand)


struct Reach_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  arr reachPoint;
  ~Reach_FeedbackControlTask(){ listDelete(TVs); }
  virtual void initTaskVariables(const ors::Graph& ors, const arr& skinState);
  virtual void updateTaskVariableGoals(const ors::Graph& ors, const arr& skinState);
};

struct Joystick_FeedbackControlTask:public FeedbackControlTaskAbstraction {
  intA joyState;
  double joyRate;
  arr skinState;
  ~Joystick_FeedbackControlTask(){ listDelete(TVs); }
  virtual void initTaskVariables(const ors::Graph& ors, const arr& skinState);
  virtual void updateTaskVariableGoals(const ors::Graph& ors, const arr& skinState);
};

#endif