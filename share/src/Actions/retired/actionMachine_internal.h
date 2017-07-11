#pragma once

#include "actionMachine.h"

#include <Control/taskControl.h>

struct sActionMachine{
  mlr::KinematicWorld world;
  TaskControlMethods taskController;
  //  Gamepad2Tasks j2t;
  arr q, qdot;
  const arr q0;
  CtrlMsg refs;
  sActionMachine() : world("model.kvg"), taskController(world, true), q0(world.q) {};
};
