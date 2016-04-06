#pragma once

#include "activity.h"
#include <RosCom/roscom.h>
#include <Control/taskController.h>

// ============================================================================

struct GamepadControlActivity : Activity, Module {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(arr, gamepadState)
  ACCESS(arr, pr2_odom)

  struct TaskControllerModule *taskController;
  struct Gamepad2Tasks *g2t;

  GamepadControlActivity();
  virtual ~GamepadControlActivity();

  virtual void open();
  virtual void step();
  virtual void close();
};
