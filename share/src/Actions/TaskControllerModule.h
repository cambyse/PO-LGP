#pragma once

#include <Core/module.h>
#include <Motion/feedbackControl.h>
#include <pr2/roscom.h>
#include <pr2/rosalvar.h>
#include <FOL/relationalMachine.h>

#ifdef MT_ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif

extern struct TaskControllerModule *taskControllerModule();

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Module {
  //protected access points
  ACCESSnew(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESSnew(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESSnew(MT::Array<CtrlTask*>, ctrlTasks)
  ACCESSnew(MT::String, effects)
  ACCESSnew(ors::KinematicWorld, modelWorld)
  ACCESSnew(AlvarMarkers, ar_pose_marker)
  ACCESSnew(bool, fixBase)
  ACCESSnew(arr, pr2_odom)

  //non-protected members
//private:
  ors::KinematicWorld realWorld;
  FeedbackMotionControl *feedbackController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  const arr q0; //< homing pose
  bool useRos;
  bool syncModelStateWithRos; //< whether the step() should reinit the state from the ros message
  bool verbose;

public:
  TaskControllerModule();
  ~TaskControllerModule();

  /// @name module implementations
  void open();
  void step();
  void close();
};
