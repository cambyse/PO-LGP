#pragma once

#include <Core/module.h>
#include <Motion/feedbackControl.h>
#include <pr2/roscom.h>
#include <pr2/rosalvar.h>
#include <FOL/relationalMachine.h>

#ifdef MLR_ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Module {
  //protected access points
  ACCESSnew(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESSnew(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESSnew(mlr::Array<CtrlTask*>, ctrlTasks)
  ACCESSnew(mlr::String, effects)
  ACCESSnew(ors::KinematicWorld, modelWorld)
  ACCESSnew(AlvarMarkers, ar_pose_marker)
  ACCESSnew(bool, fixBase)
  ACCESSnew(arr, pr2_odom)

  ACCESSnew(sensor_msgs::JointState, jointState)
  ACCESSnew(arr, q_ref)

//private:
  ors::KinematicWorld realWorld;
  FeedbackMotionControl *feedbackController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  arr q0; //< homing pose
  mlr::String robot;
  bool useRos;
  bool requiresInitialSync;
  bool syncModelStateWithRos; //< whether the step() should reinit the state from the ros message
  bool verbose;

public:
  TaskControllerModule(const char* robot="pr2");
  ~TaskControllerModule();

  void open();
  void step();
  void close();
};
