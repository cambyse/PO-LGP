#pragma once

#include "actionMachine.h"

#include <Motion/feedbackControl.h>

struct sActionMachine{
  ors::KinematicWorld world;
  FeedbackMotionControl feedbackController;
  //  Gamepad2Tasks j2t;
  arr q, qdot;
  const arr q0;
  CtrlMsg refs;
  sActionMachine() : world("model.kvg"), feedbackController(world, true), q0(world.q) {};
};
