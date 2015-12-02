#pragma once

#include "activity.h"
#include <pr2/roscom.h>
#include <Motion/feedbackControl.h>

// ============================================================================

struct GamepadControlActivity : Activity, Module {
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSnew(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESSname(arr, gamepadState)
  ACCESSname(arr, pr2_odom)

  struct TaskControllerModule *taskController;
  struct Gamepad2Tasks *g2t;

  GamepadControlActivity();
  virtual ~GamepadControlActivity();

  virtual void open();
  virtual void step();
  virtual void close();
};
