#pragma once

#include <Core/module.h>
#include <Motion/feedbackControl.h>
#include <pr2/roscom.h>
#include <pr2/rosalvar.h>

#ifdef MT_ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#endif

extern struct TaskControllerModule *taskControllerModule();

/// The task controller generates the message send to the RT_Controller
/// the problem is defined by the list of CtrlTasks
struct TaskControllerModule : Module {
  //protected access points
  ACCESS(CtrlMsg, ctrl_ref) //< the message send to the RTController
  ACCESS(CtrlMsg, ctrl_obs) //< the message received from the RTController
  ACCESS(MT::Array<CtrlTask*>, ctrlTasks)
  ACCESS(MT::String, effects)
  ACCESS(ors::KinematicWorld, modelWorld)
  ACCESS(AlvarMarkers, ar_pose_marker)
  ACCESS(bool, fixBase)
#ifdef MT_ROS
  ACCESS(geometry_msgs::PoseWithCovarianceStamped, pr2_odom)
#endif

  //non-protected members
//private:
  ors::KinematicWorld realWorld;
  FeedbackMotionControl *feedbackController;
  arr q_real, qdot_real; //< real state
  arr q_model, qdot_model; //< model state
  const arr q0; //< homing pose
  ofstream fil; //< file for diagnostics
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
