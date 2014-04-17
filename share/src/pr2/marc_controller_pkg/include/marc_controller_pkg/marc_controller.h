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
  ors::KinematicWorld world;
  pr2_mechanism_model::Tree pr2_tree;

  KDL::JntArray jnt_pos_;
  KDL::JntArrayVel jnt_vel_;
  KDL::JntArray jnt_efforts_;

  // Ors related variables
  arr u, Kd, Kp;
  arr q, qd;
  arr q_ref, qdot_ref;
  double Kp_gainFactor, Kd_gainFactor, fL_gainFactor, fR_gainFactor;


  //force related things
  ors::Shape *ftL_shape, *ftR_shape;
  arr fL_obs;
  arr fL_ref, fR_ref;
  arr y_fL, J_fL;
  arr y_fR, J_fR;

  uintA ROS_qIndex;

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
