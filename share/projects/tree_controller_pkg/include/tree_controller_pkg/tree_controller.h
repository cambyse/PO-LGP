#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_mechanism_model/tree.h>
#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <tree_controller_pkg/SetVecTarget.h>
#include <tree_controller_pkg/GetVecTarget.h>
#include <tree_controller_pkg/SetPosTarget.h>
#include <tree_controller_pkg/GetPosTarget.h>
#include <tree_controller_pkg/GetTaskState.h>
#include <tree_controller_pkg/SetJointGains.h>
#include <tree_controller_pkg/GetJointGains.h>
#include <tree_controller_pkg/GetJointState.h>
#include <tree_controller_pkg/GetFilterGains.h>
#include <tree_controller_pkg/SetFilterGains.h>
#include <tree_controller_pkg/GetTaskGains.h>
#include <tree_controller_pkg/SetTaskGains.h>
#include <tree_controller_pkg/StartLogging.h>
#include <tree_controller_pkg/StopLogging.h>

namespace tree_controller_ns{

enum
{
  StoreLen = 40000
};

class TreeControllerClass: public pr2_controller_interface::Controller
{
private:
  pr2_mechanism_model::JointState* joint_state_;
  pr2_mechanism_model::Tree tree_;

  KDL::JntArray jnt_pos_;
  KDL::JntArrayVel jnt_vel_;
  KDL::JntArray jnt_efforts_;

  // Ors related variables
  FeedbackMotionControl* MP;
  ors::KinematicWorld* world;
  PDtask *taskPos, *taskVec, *taskHome, *taskLimits;
  arr u;
  double tau_control, tau_plan;
  arr Kd,Kp;
  arr q, qd, qdd;
  arr des_q, des_qd;
  arr controlIdx;
  arr p_effort,d_effort,i_effort;
  arr y,yd,yVec,ydVec;
  arr state,stateVec;

  // Limits
  arr lowerEffortLimits, upperEffortLimits;
  arr lowerJointLimits, upperJointLimits;

  // Logging
  volatile int storage_index_;
  bool LOGGING;

  // Filter
  double q_filt;
  double qd_filt;

  // Service for communication
  ros::ServiceServer setPosTargetSrv_;    ros::ServiceServer getPosTargetSrv_;
  ros::ServiceServer setVecTargetSrv_;    ros::ServiceServer getVecTargetSrv_;
  ros::ServiceServer setJointGainsSrv_;   ros::ServiceServer getJointGainsSrv_;
  ros::ServiceServer getFilterGainsSrv_;  ros::ServiceServer setFilterGainsSrv_;
  ros::ServiceServer getTaskGainsSrv_;    ros::ServiceServer setTaskGainsSrv_;
  ros::ServiceServer startLoggingSrv_;    ros::ServiceServer stopLoggingSrv_;
  ros::ServiceServer getJointStateSrv_;
  ros::ServiceServer getTaskStateSrv_;

  // Bookkeeping variables
  arr q_bk;
  arr qd_bk;
  arr des_q_bk;
  arr des_qd_bk;
  arr u_bk;
  arr p_effort_bk;
  arr d_effort_bk;
  arr dt_bk;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  bool setPosTarget(tree_controller_pkg::SetPosTarget::Request &req, tree_controller_pkg::SetPosTarget::Response &resp);
  bool getPosTarget(tree_controller_pkg::GetPosTarget::Request &req, tree_controller_pkg::GetPosTarget::Response &resp);
  bool setVecTarget(tree_controller_pkg::SetVecTarget::Request &req, tree_controller_pkg::SetVecTarget::Response &resp);
  bool getVecTarget(tree_controller_pkg::GetVecTarget::Request &req, tree_controller_pkg::GetVecTarget::Response &resp);
  bool getTaskState(tree_controller_pkg::GetTaskState::Request &req, tree_controller_pkg::GetTaskState::Response &resp);
  bool setJointGains(tree_controller_pkg::SetJointGains::Request &req, tree_controller_pkg::SetJointGains::Response &resp);
  bool getJointGains(tree_controller_pkg::GetJointGains::Request &req, tree_controller_pkg::GetJointGains::Response &resp);
  bool getJointState(tree_controller_pkg::GetJointState::Request &req, tree_controller_pkg::GetJointState::Response &resp);
  bool getFilterGains(tree_controller_pkg::GetFilterGains::Request &req, tree_controller_pkg::GetFilterGains::Response &resp);
  bool setFilterGains(tree_controller_pkg::SetFilterGains::Request &req, tree_controller_pkg::SetFilterGains::Response &resp);
  bool getTaskGains(tree_controller_pkg::GetTaskGains::Request &req, tree_controller_pkg::GetTaskGains::Response &resp);
  bool setTaskGains(tree_controller_pkg::SetTaskGains::Request &req, tree_controller_pkg::SetTaskGains::Response &resp);
  bool startLogging(tree_controller_pkg::StartLogging::Request &req, tree_controller_pkg::StartLogging::Response &resp);
  bool stopLogging(tree_controller_pkg::StopLogging::Request &req, tree_controller_pkg::StopLogging::Response &resp);
};
}
