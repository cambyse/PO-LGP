#include <Motion/feedbackControl.h>

struct Joystick2Tasks{
  FeedbackMotionControl& MP;
  PDtask *endeffR, *endeffL, *base, *baseQuat, *limits, *qitself;
  Joystick2Tasks(FeedbackMotionControl& _MP);
  bool updateTasks(arr &joystickState, double dt);
};

