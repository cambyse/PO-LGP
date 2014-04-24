#include <Motion/feedbackControl.h>

struct Gamepad2Tasks{
  FeedbackMotionControl& MP;
  PDtask *endeffR, *endeffL, *base, *baseQuat, *head, *limits, *coll,  *gripperL, *gripperR;
  Gamepad2Tasks(FeedbackMotionControl& _MP);
  bool updateTasks(arr& gamepadState);
};

bool stopButtons(const arr& gamepadState);
