#pragma once

#include <Control/taskControl.h>

//#ifdef WITH_ROS
  #include <Actions/actions.h>
  #include <RosCom/roscom.h>
//#endif
//#ifdef MT_ROS
    #include <geometry_msgs/PoseWithCovarianceStamped.h>
//#endif
// ============================================================================
struct PDExecutor: Thread{
  // calibrated_pose is pos + orientation (quaternion)
  ACCESS(floatA, calibrated_pose_rh);
  ACCESS(floatA, calibrated_pose_lh);
  // distance of the two finger tips
  ACCESS(float, calibrated_gripper_rh);
  ACCESS(float, calibrated_gripper_lh);
  // Msgs that are send to ROS
// #ifdef WITH_ROS
  ACCESS(CtrlMsg, ctrl_ref);
  ACCESS(CtrlMsg, ctrl_obs);
// #endif
//#ifdef MT_ROS
  ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom);
//#endif
  ACCESS(bool, fixBase);


  ACCESS(floatA, poses_rh);
  ACCESS(floatA, poses_lh);

  ACCESS(bool, initmapper);  
  // TaskControlMethods stuff
  ACCESS(arr, drive);
  ACCESS(bool,calisaysokay);
  ACCESS(bool,taped);

  mlr::KinematicWorld world;
  mlr::KinematicWorld worldreal;
  TaskControlMethods fmc;
  arr q, qdot ;

  bool inited, useros;
  arr error;

  // ros stuff
// #ifdef WITH_ROS
  // RosCom_ControllerSync* roscom;
// #endif

  // Tasks
  CtrlTask* limits;
  CtrlTask* collision;

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
  CtrlTask* base;
  CtrlTask* fc; 

  PDExecutor();

  void step();
  void open();
  void close();

  void visualizeSensors();
  void initRos();
  void sendRosCtrlMsg();
};
