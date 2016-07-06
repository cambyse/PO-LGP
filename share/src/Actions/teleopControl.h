#pragma once

#include "activity.h"
#include <Control/taskController.h>
#include <Control/ctrlMsg.h>

// ============================================================================

struct TeleopControlActivity : Activity, Module {
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(mlr::Array<CtrlTask*>, ctrlTasks)
//  ACCESS(arr, gamepadState)
  ACCESS(arr, pr2_odom)
  ACCESS(arr, gamepadState)
  ACCESS(bool, initmapper)


  // calibrated_pose is pos + orientation (quaternion)
  ACCESS(floatA, calibrated_pose_rh)
  ACCESS(floatA, calibrated_pose_lh)

  // distance of the two finger tips
  ACCESS(float, calibrated_gripper_rh)
  ACCESS(float, calibrated_gripper_lh)

  ACCESS(arr, drive)

  struct TaskControllerModule *taskController;
  struct Teleop2Tasks *t2t;

  TeleopControlActivity();
  virtual ~TeleopControlActivity();

  virtual void open();
  virtual void step();
  virtual void close();
};
