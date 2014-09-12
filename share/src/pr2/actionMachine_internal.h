#pragma once

#include "actionMachine.h"

#include <Motion/feedbackControl.h>

struct sActionMachine{
  ors::KinematicWorld world;
  FeedbackMotionControl MP;
  //  Gamepad2Tasks j2t;
  arr q, qdot;
  CtrlMsg refs;
  sActionMachine() : world("model.kvg"), MP(world, true) {};
};
