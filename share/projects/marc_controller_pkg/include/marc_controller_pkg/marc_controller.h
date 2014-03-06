#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/tree.h>
#include <Ors/ors.h>
#include <marc_controller_pkg/JointState.h>

namespace marc_controller_ns{

class TreeControllerClass: public pr2_controller_interface::Controller
{
private:
  //pr2_mechanism_model::JointState* joint_state_;
  pr2_mechanism_model::Tree pr2_tree;
//  pr2_mechanism_model::RobotState* pr2_robot;

  KDL::JntArray jnt_pos_;
  KDL::JntArrayVel jnt_vel_;
  KDL::JntArray jnt_efforts_;

  // Ors related variables
  arr u, Kd, Kp;
  arr q, qd;
  arr q_ref, qdot_ref;

  uintA pr2_jointIndex;

  ros::Publisher jointState_publisher;
  ros::Subscriber jointReference_subscriber;
  marc_controller_pkg::JointState jointStateMsg;

  // Limits
  arr lowerEffortLimits, upperEffortLimits;
  arr lowerJointLimits, upperJointLimits;

  // Filter
  double q_filt;
  double qd_filt;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  void jointReferenceSubscriber(const marc_controller_pkg::JointState::ConstPtr& msg);
};

}
