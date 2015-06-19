#pragma once

#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================
struct Calibrator: Module {
  ACCESS(arr, gamepadState);
  ACCESS(arrf, poses);
  ACCESS(arrf, calibrated_pose_rh);
  ACCESS(arrf, calibrated_pose_lh);
  ACCESS(float, calibrated_gripper_rh);
  ACCESS(float, calibrated_gripper_lh);

  ACCESS(arrf, poses_rh);
  ACCESS(arrf, poses_lh);

  bool calibration_phase; ///< indicates if we're in the calibration phase

  // arrf posesFront, posesSide, posesOpen, posesClosed;
  arrf posesSide, posesOpen, posesClosed;

  ors::Transformation transf_neck, transf_neck_planar;
  ors::Quaternion quat_neck;
  float radius;

  // arrf center;
  // float radius;

  float m_rh, m_lh;
  float q_rh, q_lh;

  MocapID mid;

  Calibrator();

  void open() {}
  void close() {}
  void step();

  void fixCoordinates(arrf &poses);
  void calibrate();
  void transform(const arrf& poses_raw);
};
