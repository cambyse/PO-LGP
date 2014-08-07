#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_mechanism_model/tree.h>
#include <Ors/ors.h>
#include <Motion/feedbackControl.h>
#include <Algo/MLcourse.h>
#include <tree_controller_pkg/GetJointGains.h>
#include <tree_controller_pkg/SetJointGains.h>
#include <tree_controller_pkg/GetJointState.h>
#include <tree_controller_pkg/SetCommand.h>
#include <tree_controller_pkg/StartLogging.h>
#include <tree_controller_pkg/StopLogging.h>
#include <tree_controller_pkg/JointState.h>




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
  arr u;
  double tau_control, tau_plan;
  arr Kp,Kd;
//  arr i_claim, integral;
  arr q, qd, qdd;
  arr des_q, des_qd;
  arr controlIdx;
  arr p_effort, d_effort,gp_effort;
  arr measured_effort;
  double qd_filt;

  ros::Publisher joint_pub;
  tree_controller_pkg::JointState joint_pub_state;

  // Limits
  arr lowerEffortLimits, upperEffortLimits;

  // Logging
  volatile int storage_index_;
  bool LOGGING;

  // Service for communication
  ros::ServiceServer setJointGainsSrv_;   ros::ServiceServer getJointGainsSrv_;
  ros::ServiceServer getJointStateSrv_;
  ros::ServiceServer startLoggingSrv_;    ros::ServiceServer stopLoggingSrv_;
  ros::ServiceServer setCommandSrv_;

  // Bookkeeping variables
  arr q_bk;
  arr qd_bk;
  arr q_filt_bk;
  arr qd_filt_bk;
  arr des_q_bk;
  arr des_qd_bk;
  arr u_bk;
  arr p_effort_bk;
  arr i_effort_bk;
  arr d_effort_bk;
  arr gp_effort_bk;
  arr dt_bk;
  arr measured_effort_bk;


public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  bool setJointGains(tree_controller_pkg::SetJointGains::Request &req, tree_controller_pkg::SetJointGains::Response &resp);
  bool getJointGains(tree_controller_pkg::GetJointGains::Request &req, tree_controller_pkg::GetJointGains::Response &resp);
  bool getJointState(tree_controller_pkg::GetJointState::Request &req, tree_controller_pkg::GetJointState::Response &resp);
  bool startLogging(tree_controller_pkg::StartLogging::Request &req, tree_controller_pkg::StartLogging::Response &resp);
  bool stopLogging(tree_controller_pkg::StopLogging::Request &req, tree_controller_pkg::StopLogging::Response &resp);
  bool setCommand(tree_controller_pkg::SetCommand::Request &req,tree_controller_pkg::SetCommand::Response &resp);
};
}
