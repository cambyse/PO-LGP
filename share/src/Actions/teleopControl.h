#pragma once

#include "activity.h"
#include <pr2/roscom.h>
#include <Control/taskController.h>

// ============================================================================

struct TeleopControlActivity : Activity, Module {
  ACCESSname(CtrlMsg, ctrl_ref)
  ACCESSname(CtrlMsg, ctrl_obs)
  ACCESSnew(mlr::Array<CtrlTask*>, ctrlTasks)
//  ACCESSname(arr, gamepadState)
  ACCESSname(arr, pr2_odom)

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
