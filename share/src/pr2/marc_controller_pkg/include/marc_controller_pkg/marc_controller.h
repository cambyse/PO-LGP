#include <ros/ros.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/tree.h>
#include <geometry_msgs/WrenchStamped.h>
#include <marc_controller_pkg/JointState.h>
#include <Ors/ors.h>

namespace marc_controller_ns{

class TreeControllerClass: public pr2_controller_interface::Controller
{
private:
  Mutex mutex; //callbacks are not thread safe!!!!!!!!!!!!!
  ors::KinematicWorld world;
  //  pr2_mechanism_model::Tree pr2_tree;

  /* KDL::JntArray jnt_pos_; */
  /* KDL::JntArrayVel jnt_vel_; */
  /* KDL::JntArray jnt_efforts_; */

  // Ors related variables
  arr u, Kd, Kp;
  arr q, qd;
  arr q_ref, qdot_ref;
  arr fL_ref, fR_ref;
  arr Kq_gainFactor, Kd_gainFactor, Kf_gainFactor;
  arr u_bias;

  //force related things
  arr fL_obs, fR_obs;

  //matching joint indices
  //  uintA ROS_qIndex;
  MT::Array<pr2_mechanism_model::JointState*> ROS_joints;
  ors::Joint *j_worldTranslationRotation;

  ros::Publisher jointState_publisher;
  ros::Publisher baseCommand_publisher;
  ros::Subscriber jointReference_subscriber;
  ros::Subscriber forceSensor_subscriber;
  marc_controller_pkg::JointState jointStateMsg;

  // Limits
  arr limits;

  // Filter
  double q_filt;
  double qd_filt;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  void jointReference_subscriber_callback(const marc_controller_pkg::JointState::ConstPtr& msg);
  void forceSensor_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};

}
