#pragma once

#include <Motion/feedbackControl.h>
#include <Motion/teleop2tasks.h>

//#ifdef WITH_ROS
//  #include <Actions/actions.h>
  #include <pr2/roscom.h>
//#endif
//#ifdef MT_ROS
    #include <geometry_msgs/PoseWithCovarianceStamped.h>
//#endif
// ============================================================================
struct PDExecutor: Module {
  // calibrated_pose is pos + orientation (quaternion)
  ACCESS(floatA, calibrated_pose_rh)
  ACCESS(floatA, calibrated_pose_lh)

  // distance of the two finger tips
  ACCESS(float, calibrated_gripper_rh)
  ACCESS(float, calibrated_gripper_lh)

  // Msgs that are send to ROS
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
  ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom)
  ACCESS(bool, fixBase)

  ACCESS(bool, initmapper)
  // FeedbackMotionControl stuff
  ACCESS(arr, drive)


  ors::KinematicWorld world;
  ors::KinematicWorld worldreal;
  FeedbackMotionControl fmc;
  Teleop2Tasks tele2tasks;
  arr q, qdot ;

  bool inited, useros;
  arr error;


  // Tasks

  PDExecutor();

  void step();
  void open();
  void close();

  void initRos();
  void sendRosCtrlMsg();
};
