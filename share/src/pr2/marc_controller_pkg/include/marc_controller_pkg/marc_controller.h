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

  // Ors related variables
  arr u, Kd_base, Kp_base;
  arr q, qd;
  arr q_ref, qdot_ref;
  arr Kp, Kd, Ki;
  arr u_bias;
  double velLimitRatio, effLimitRatio;

  //force related things
  arr fL_obs, fR_obs, fL_ref, fR_ref;
  arr err, J_ft_inv;
  double gamma;

  //matching joint indices
  MT::Array<pr2_mechanism_model::JointState*> ROS_joints;
  ors::Joint *j_worldTranslationRotation;

  //subscriber and publishers
  ros::Publisher jointState_publisher;
  ros::Publisher baseCommand_publisher;
  ros::Subscriber jointReference_subscriber;
  ros::Subscriber l_ft_subscriber;
  ros::Subscriber r_ft_subscriber;
  marc_controller_pkg::JointState jointStateMsg;

  // Limits
  arr limits;

  // Filter
  double q_filt;
  double qd_filt;

  // internal: counter for sparse messages
  uint msgBlock;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh);
  virtual void starting();
  virtual void update();
  virtual void stopping();

  void jointReference_subscriber_callback(const marc_controller_pkg::JointState::ConstPtr& msg);
  void l_ft_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void r_ft_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
};

}
