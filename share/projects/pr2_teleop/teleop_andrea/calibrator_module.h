#pragma once

#include <Motion/feedbackControl.h>
#include <System/engine.h>
// #include <pr2/actionMachine.h>
// #include <pr2/actions.h>
#include <Mocap/mocapdata.h>

// ============================================================================
struct Calibrator: Module {
  ACCESS(arr, gamepadState);
  ACCESS(floatA, poses);
  ACCESS(floatA, calibrated_pose_rh);
  ACCESS(floatA, calibrated_pose_lh);
  ACCESS(float, calibrated_gripper_rh);
  ACCESS(float, calibrated_gripper_lh);

  ACCESS(floatA, poses_rh);
  ACCESS(floatA, poses_lh);

  bool calibration_phase; ///< indicates if we're in the calibration phase

  // floatA posesFront, posesSide, posesOpen, posesClosed;
  floatA posesSide, posesOpen, posesClosed;

  floatA center;
  float radius;

  float m_rh, m_lh;
  float q_rh, q_lh;

  MocapID mid;

  Calibrator();

  void open() {}
  void close() {}
  void step();

  void fixCoordinates(floatA &poses);
  void calibrate();
  void transform(const floatA& poses_raw);
};
