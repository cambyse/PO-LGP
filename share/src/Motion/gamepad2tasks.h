#include <Motion/feedbackControl.h>

struct Gamepad2Tasks{
  FeedbackMotionControl& MP;
  PDtask *endeffR, *endeffL, *base, *baseQuat, *head, *limits, *qitself;
  Gamepad2Tasks(FeedbackMotionControl& _MP);
  bool updateTasks(arr &gamepadState);
};

