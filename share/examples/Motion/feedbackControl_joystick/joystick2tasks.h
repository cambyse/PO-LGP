#include <Motion/feedbackControl.h>

struct Joystick2Tasks{
  FeedbackMotionControl& MP;
  PDtask *endeffR, *endeffL, *base;
  Joystick2Tasks(FeedbackMotionControl& _MP);
  void updateTasks(const arr& joystickState);
};

