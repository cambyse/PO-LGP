#pragma once

#include <Hardware/gamepad/gamepad.h>
#include <Motion/feedbackControl.h>

#include <Mocap/mocapdata.h>
#include <Actions/actions.h>
#include <pr2/roscom.h>

// ============================================================================
struct PDExecutor: Module {
  ACCESS(arr, gamepadState);
  ACCESS(arrf, poses);

  // Msgs that are send to ROS
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);

  // FeedbackMotionControl stuff
  ors::KinematicWorld world;
  FeedbackMotionControl fmc;
  arr q, qdot;

  bool started, useros;

  // Tasks
  CtrlTask* limits;
  CtrlTask* collisions;

  CtrlTask* effPosR;
  CtrlTask* gripperR;
  CtrlTask* effOrientationR;
  // CtrlTask* effOrientationRX;
  // CtrlTask* effOrientationRY;
  // CtrlTask* effOrientationRZ;

  CtrlTask* effPosL;
  CtrlTask* gripperL;
  CtrlTask* effOrientationL;
  // CtrlTask* effOrientationLX;
  // CtrlTask* effOrientationLY;
  // CtrlTask* effOrientationLZ;

  CtrlTask* effHead, *effHead_ref;

  MocapID mid;
  ors::Transformation transf_mocap_robot;
  ors::Transformation transf;

  PDExecutor();

  void step();
  void open();
  void close();

  void activateTasks(bool active);
  void teleop();

  void rigidTransf(arrf &poses);
  void trackHand(const arrf &thumb, const arrf &index, CtrlTask *effPos, CtrlTask *gripper, CtrlTask *effOrientation, bool right);
  void trackHead();
  arr makeHandOrientation(const arrf &thumb, const arrf &index, bool right);
  void runOperationalSpaceControl();

  void visualizeSensors();
  void initRos();
  void sendRosCtrlMsg();
};
