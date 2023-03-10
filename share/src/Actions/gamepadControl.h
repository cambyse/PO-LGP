#pragma once

#include "activity.h"
#include <Control/ctrlMsg.h>
#include <Control/taskControl.h>

// ============================================================================

struct GamepadControlActivity : Activity, Thread {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESS(arr, gamepadState)
  ACCESS(arr, pr2_odom)

  struct TaskControlThread *taskController;
  struct Gamepad2Tasks *g2t;

  GamepadControlActivity();
  virtual ~GamepadControlActivity();

  virtual void open();
  virtual void step();
  virtual void close();
};
