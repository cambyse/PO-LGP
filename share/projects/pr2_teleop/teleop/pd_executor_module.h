#pragma once

#include <Motion/feedbackControl.h>
#include <System/engine.h>

#ifdef WITH_ROS
  #include <Actions/actions.h>
#endif

// ============================================================================
struct PDExecutor: Module {
  // calibrated_pose is pos + orientation (quaternion)
  ACCESS(floatA, calibrated_pose_rh);
  ACCESS(floatA, calibrated_pose_lh);
  // distance of the two finger tips
  ACCESS(float, calibrated_gripper_rh);
  ACCESS(float, calibrated_gripper_lh);
  // Msgs that are send to ROS
#ifdef WITH_ROS
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
#endif

  ACCESS(floatA, poses_rh);
  ACCESS(floatA, poses_lh);

  // FeedbackMotionControl stuff
  ors::KinematicWorld world;
  FeedbackMotionControl fmc;
  arr q, qdot;

  // ros stuff
#ifdef WITH_ROS
  // RosCom_ControllerSync* roscom;
#endif

  // Tasks
  PDtask* limits;
  PDtask* collision;

  PDtask* effPosR;
  PDtask* gripperR;
  PDtask* effOrientationR;
  // PDtask* effOrientationRX;
  // PDtask* effOrientationRY;
  // PDtask* effOrientationRZ;

  PDtask* effPosL;
  PDtask* gripperL;
  PDtask* effOrientationL;
  // PDtask* effOrientationLX;
  // PDtask* effOrientationLY;
  // PDtask* effOrientationLZ;

  PDExecutor();

  void step();
  void open();
  void close();

  void visualizeSensors();
  void initRos();
  void sendRosCtrlMsg();
};
