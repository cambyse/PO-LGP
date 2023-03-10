#pragma once

#include <Control/taskControl.h>

#include <Actions/actions.h>
#include <RosCom/roscom.h>

// ============================================================================
struct PDExecutor: Thread{
  // calibrated_pose is pos + orientation (quaternion)
  ACCESS(arrf, calibrated_pose_rh);
  ACCESS(arrf, calibrated_pose_lh);
  // distance of the two finger tips
  ACCESS(float, calibrated_gripper_rh);
  ACCESS(float, calibrated_gripper_lh);
  // Msgs that are send to ROS
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);

  ACCESS(arrf, poses_rh);
  ACCESS(arrf, poses_lh);

  // TaskControlMethods stuff
  mlr::KinematicWorld world;
  TaskControlMethods fmc;
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

  PDExecutor();

  void step();
  void open();
  void close();

  void visualizeSensors();
  void initRos();
  void sendRosCtrlMsg();
};
