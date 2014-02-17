#include <Motion/feedbackControl.h>

struct Joystick2Tasks{
  FeedbackMotionControl& MP;
  PDtask *endeffR, *endeffL, *base, *limits, *qitself;
  Joystick2Tasks(FeedbackMotionControl& _MP);
  void updateTasks(arr &joystickState, double dt);
};

