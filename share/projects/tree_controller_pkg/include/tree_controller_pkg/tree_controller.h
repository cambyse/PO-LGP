#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_mechanism_model/tree.h>
#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Algo/MLcourse.h>
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
#include <tree_controller_pkg/JointState.h>
#include <tree_controller_pkg/SetNaturalGains.h>
#include <tree_controller_pkg/SetControlParam.h>
#include <tree_controller_pkg/GetControlParam.h>

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
  pr2_mechanism_model::RobotState* robot_;

  KDL::JntArray jnt_pos_;
  KDL::JntArrayVel jnt_vel_;
  KDL::JntArray jnt_efforts_;

  // Ors related variables
  FeedbackMotionControl* MP;
  ors::KinematicWorld* world;
  CtrlTask *taskPos, *taskVec, *taskHome, *taskLimits;
  arr u;
  double tau_control, tau_plan;
  arr Kp,Ki,Kd;
  arr i_claim, integral;
  arr q, qd, qdd;
  arr des_q, des_qd;
  arr controlIdx;
  arr p_effort,i_effort, d_effort;
  arr y,yd,yVec,ydVec;
  arr state,stateVec;
  arr measured_effort;

  ros::Publisher joint_pub;
  tree_controller_pkg::JointState joint_pub_state;

  // Limits
  arr lowerEffortLimits, upperEffortLimits;

  // Logging
  volatile int storage_index_;
  bool LOGGING;

  // Filter
  arr q_filt;
  arr qd_filt;
  arr gram, q_hist, beta;
  uint filter_range;
  double delta;

  // Service for communication
  ros::ServiceServer setPosTargetSrv_;    ros::ServiceServer getPosTargetSrv_;
  ros::ServiceServer setVecTargetSrv_;    ros::ServiceServer getVecTargetSrv_;
  ros::ServiceServer setJointGainsSrv_;   ros::ServiceServer getJointGainsSrv_;
  ros::ServiceServer getFilterGainsSrv_;  ros::ServiceServer setFilterGainsSrv_;
  ros::ServiceServer getTaskGainsSrv_;    ros::ServiceServer setTaskGainsSrv_;
  ros::ServiceServer startLoggingSrv_;    ros::ServiceServer stopLoggingSrv_;
  ros::ServiceServer getJointStateSrv_;
  ros::ServiceServer getTaskStateSrv_;
  ros::ServiceServer setNaturalGainsSrv_;
  ros::ServiceServer setControlParamSrv_;    ros::ServiceServer getControlParamSrv_;

  // Bookkeeping variables
  arr q_bk;
  arr qd_bk;
  arr q_filt_bk;
  arr qd_filt_bk;
  arr des_q_bk;
  arr des_qd_bk;
  arr des_qdd_bk;
  arr u_bk;
  arr p_effort_bk;
  arr i_effort_bk;
  arr d_effort_bk;
  arr dt_bk;
  arr taskPos_y_bk;
  arr taskPos_yRef_bk;
  arr taskVec_y_bk;
  arr taskVec_yRef_bk;
  arr measured_effort_bk;


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
  bool setNaturalGains(tree_controller_pkg::SetNaturalGains::Request &req, tree_controller_pkg::SetNaturalGains::Response &resp);
  bool setControlParam(tree_controller_pkg::SetControlParam::Request &req, tree_controller_pkg::SetControlParam::Response &resp);
  bool getControlParam(tree_controller_pkg::GetControlParam::Request &req, tree_controller_pkg::GetControlParam::Response &resp);
};
}
