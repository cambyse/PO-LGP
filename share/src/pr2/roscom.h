#pragma once
// roscom makes MLR speak with ROS.
//===========================================================================
#include <Core/module.h>
#include <Core/array.h>
#include <Core/geo.h>
#include <Ors/ors.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>


//===========================================================================
//
// utils
//

bool rosOk();
void rosCheckInit();

//===========================================================================
/** MACRO to create a MLR module that integrates data from ROS into the MLR
 *  module system.
 *
 *  This macro creates a class "ROS_#var_name" that subscribes the given
 *  topic_name with the given msg_type.  The ros msg is basically copied into
 *  the MLR module system.
 *
 *  You have to add the created class and the ACCESS variable to your System
 *  and you're good to go.
 *
 *  See ../../examples/pr2/generic_ros_sync/main.cpp for more.
 */
#define ROSSUB(topic_name, msg_type, var_name) \
  class ROSSUB_##var_name : public Module { \
  public: \
    ACCESS(msg_type, var_name) \
    ROSSUB_##var_name() : Module(#var_name) {} \
    void open() { \
      rosCheckInit(); \
      this->_nh = new ros::NodeHandle; \
      this->_sub  = this->_nh->subscribe( \
        topic_name, 1, &ROSSUB_##var_name::callback, this); \
    } \
    void step() {} \
    void close() { \
      this->_nh->shutdown(); \
      delete _nh; \
    } \
    void callback(const msg_type::ConstPtr& msg) { \
      this->var_name.set() = *msg; \
    } \
  private: \
    ros::NodeHandle* _nh; \
    ros::Subscriber _sub; \
    \
  };


//===========================================================================
//
// variable declarations
//

//-- a basic message type for communication with the PR2 controller
struct CtrlMsg{
  arr q, qdot, fL, fR, u_bias, J_ft_inv;
  arr Kp, Kd, Ki;
  double velLimitRatio, effLimitRatio, gamma;
  CtrlMsg():Kp(ARR(1.)), Kd(ARR(1.)), Ki(ARR(0.)), velLimitRatio(1.), effLimitRatio(1.){}
  CtrlMsg(const arr& q, const arr& qdot,
          const arr& fL, const arr& fR,
          const arr& u_bias, const arr& J_ft_inv,
          double velLimitRatio, double effLimitRatio, double gamma)
    :q(q), qdot(qdot), fL(fL), fR(fR), u_bias(u_bias), J_ft_inv(J_ft_inv), velLimitRatio(velLimitRatio), effLimitRatio(effLimitRatio), gamma(gamma){}
};
//inline void operator<<(ostream& os, const CtrlMsg& m){ os<<"BLA"; }
//inline void operator>>(istream& os, CtrlMsg& m){  }

//===========================================================================

////-- the message that defines the motor level controller: feedback regulators on q, q_dot, fL and fR
//struct JointControllerRefsMsg{
//  arr q_ref, qdot_ref, fL_ref, fR_ref, u_bias;
//  arr Kq_matrix, Kd_matrix, KfL_matrix, KfR_matrix;
//};
//inline void operator<<(ostream& os, const JointControllerRefsMsg& m){ os<<"JointControllerRefsMsg - NIY"; }
//inline void operator>>(istream& os, JointControllerRefsMsg& m){  }

////===========================================================================

////-- the message that defines the state (of the controller) on the joint level
//struct JointControllerStateMsg{
//  arr q_real, qdot_real, fL_real, fR_real, u_cmd, u_real;
//};
//inline void operator<<(ostream& os, const JointControllerStateMsg& m){ os<<"JointControllerStateMsg - NIY"; }
//inline void operator>>(istream& os, JointControllerStateMsg& m){  }

//===========================================================================
//
// modules
//
/* TODO: allow modules to set default loopWithBeat, listenFirst, etc
options. (In their constructor?)
*/
//===========================================================================
/// This module only calls ros:spinOnce() in step() and loops full speed -- to sync the process with the ros server
BEGIN_MODULE(RosCom_Spinner)
END_MODULE()

//===========================================================================
/// This module syncs the controller state and refs with the real time hardware controller (marc_controller_...)
BEGIN_MODULE(RosCom_ControllerSync)
  ACCESS(CtrlMsg, ctrl_ref)
  ACCESS(CtrlMsg, ctrl_obs)
//  ACCESS(JointControllerRefsMsg, ctrl_refs)
//  ACCESS(JointControllerStateMsg, ctrl_state)
END_MODULE()

// Helper function so sync ors with the real PR2
/**
 * This starts the initial sync of the world with ctrl_obs from the robot.
 *
 * This is verbose (helps debugging) and retries to connect to the robot multiple times.
 *
 * If useRos==false then nothing happens.
 */
void initialSyncJointStateWithROS(ors::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

/**
 * Sync the world with ctrl_obs from the robot.
 *
 * If useRos==false then nothing happens.
 */
void syncJointStateWitROS(ors::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

//===========================================================================
/// This module syncs the kinect
BEGIN_MODULE(RosCom_KinectSync)
  ACCESS(byteA, kinect_rgb)
  ACCESS(uint16A, kinect_depth)
END_MODULE()

//===========================================================================
/// This module syncs the left & right eye
BEGIN_MODULE(RosCom_CamsSync)
  ACCESS(byteA, rgb_leftEye)
  ACCESS(byteA, rgb_rightEye)
END_MODULE()

//===========================================================================
/// This module syncs the left & right arm cams
BEGIN_MODULE(RosCom_ArmCamsSync)
  ACCESS(byteA, rgb_leftArm)
  ACCESS(byteA, rgb_rightArm)
END_MODULE()

//===========================================================================
/// Sync the FT sensor
BEGIN_MODULE(RosCom_ForceSensorSync)
  ACCESS(arr, wrenchL)
  ACCESS(arr, wrenchR)
END_MODULE()

//===========================================================================
/// Sync the AR maker alvar
ROSSUB("/ar_pose_marker", ar_track_alvar_msgs::AlvarMarkers, ar_pose_marker)

//===========================================================================
