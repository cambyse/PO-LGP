#include "tree_controller_pkg/tree_controller.h"
#include <pluginlib/class_list_macros.h>


namespace tree_controller_ns {

/// Controller initialization in non-realtime
bool TreeControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  robot_ = robot;
  std::string joint_name;

  if (!tree_.init(robot))
  {
    ROS_ERROR("Could not load robot tree");
    return false;
  }

  /// Print joint state
  joint_state_ = robot->getJointState(joint_name);
  jnt_pos_.resize(tree_.size());
  jnt_vel_.resize(tree_.size());
  jnt_efforts_.resize(tree_.size());
  ROS_INFO("Tree sizes: %d",tree_.size());
  int i;
  tree_.getPositions(jnt_pos_);
  for(i=0;i<tree_.size();i++) {
    ROS_INFO("Joint Namea %d: %s: %f",i,tree_.getJoint(i)->joint_->name.c_str(), jnt_pos_(i));
  }

  /// Initialize Services
  setJointGainsSrv_ =  n.advertiseService("set_joint_gains", &TreeControllerClass::setJointGains, this);
  getJointGainsSrv_ =  n.advertiseService("get_joint_gains", &TreeControllerClass::getJointGains, this);
  getJointStateSrv_ =  n.advertiseService("get_joint_state", &TreeControllerClass::getJointState, this);
  startLoggingSrv_ = n.advertiseService("start_logging", &TreeControllerClass::startLogging, this);
  stopLoggingSrv_ = n.advertiseService("stop_logging", &TreeControllerClass::stopLogging, this);
  setCommandSrv_ = n.advertiseService("set_command", &TreeControllerClass::setCommand, this);

  /// init ORS
  tau_control = 0.001;
  controlIdx = {30,31,32,33,34,35,36};
  qd_filt = 0.;

  q = arr(controlIdx.d0);
  qd = arr(controlIdx.d0);       qd.setZero();
  qdd = arr(controlIdx.d0);      qdd.setZero();
  des_q = arr(controlIdx.d0);
  des_qd = arr(controlIdx.d0);   des_qd.setZero();
  gp_effort = arr(controlIdx.d0);   gp_effort.setZero();
  u = arr(controlIdx.d0); u.setZero();
  measured_effort = arr(controlIdx.d0);

  /// Initialize Logging
  LOGGING = false;
  storage_index_ = 0;
  q_bk = arr(StoreLen,controlIdx.d0);
  qd_bk = arr(StoreLen,controlIdx.d0);
  q_filt_bk = arr(StoreLen,controlIdx.d0);
  qd_filt_bk = arr(StoreLen,controlIdx.d0);
  des_q_bk = arr(StoreLen,controlIdx.d0);
  des_qd_bk = arr(StoreLen,controlIdx.d0);
  u_bk = arr(StoreLen,controlIdx.d0);
  measured_effort_bk = arr(StoreLen,controlIdx.d0);
  p_effort_bk = arr(StoreLen,controlIdx.d0);
  d_effort_bk = arr(StoreLen,controlIdx.d0);
  gp_effort_bk = arr(StoreLen,controlIdx.d0);
  dt_bk = arr(StoreLen);


  /// Initialize joint state
  tree_.getPositions(jnt_pos_);
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    des_q(i) = jnt_pos_(controlIdx(i));
  }

  /// Define torque and joint limits for the used joints
  for (uint i =0;i<controlIdx.d0;i++) {
    double low_tmp, high_tmp;
    tree_.getJoint(controlIdx(i))->getLimits(low_tmp,high_tmp);
    lowerEffortLimits.append(-0.9*tree_.getJoint(controlIdx(i))->joint_->limits->effort);
    upperEffortLimits.append(0.9*tree_.getJoint(controlIdx(i))->joint_->limits->effort);
  }
  cout << "lowEffortLimits: " << lowerEffortLimits << endl;
  cout << "highEffortLimits: " << upperEffortLimits << endl;


  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting()
{
  //                                          ROS_GAINS
  //                                          P    D    I
  // 30: r_shoulder_pan_joint,   150 60    2400   18  800
  // 31: r_shoulder_lift_joint,  150 60    1200   10  700
  // 32: r_upper_arm_roll_joint,  30  4    1000    6  600
  // 33: r_elbow_flex_joint,      30 10     700    4  450
  // 34: r_forearm_roll_joint,    10  2     300    6  300
  // 35: r_wrist_flex_joint,       6  2     300    4  300
  // 36: r_wrist_roll_joint,       6  2     300    4  300

  Kp = {150.0, 150.0, 30., 30., 10., 6., 6.};
  Kd = {20.0, 20.0, 2.,5.,1.,1.,1.};

  /// Initialize joint state
  tree_.getPositions(jnt_pos_);
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    des_q(i) = jnt_pos_(controlIdx(i));
  }
  qd.setZero();
  qdd.setZero();

  /// init filter param 
  LOGGING = true;
}

/// Controller update loop in realtime
void TreeControllerClass::update()
{
  /// pull pos & vel
  tree_.getPositions(jnt_pos_);
  tree_.getVelocities(jnt_vel_);
  tree_.getEfforts(jnt_efforts_);

  /// Convert KDL to ORS
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    qd(i) = qd_filt*qd(i)  + (1.-qd_filt)*jnt_vel_.qdot(controlIdx(i));
    measured_effort(i) = jnt_efforts_(controlIdx(i));
  }

  /// PD Control
  p_effort = Kp % (des_q - q);
  d_effort = Kd % (des_qd - qd);

  /// Convert ORS to KDL
  for (uint i =0;i<controlIdx.d0;i++) {
    u(i) = p_effort(i) + d_effort(i) + gp_effort(i);

    tree_.getJoint(controlIdx(i))->commanded_effort_ = u(i);
    tree_.getJoint(controlIdx(i))->enforceLimits();

    // Additional Safety Check
    if (tree_.getJoint(controlIdx(i))->commanded_effort_>upperEffortLimits(i) || tree_.getJoint(controlIdx(i))->commanded_effort_ < lowerEffortLimits(i)) {
      ROS_ERROR("SAFETY CHECK FAILED! Max u(%d): %f , Min u(%d): %f, u(%d): %f",i,upperEffortLimits(i),i,lowerEffortLimits(i),i,u(i));
      tree_.getJoint(controlIdx(i))->commanded_effort_ = 0.;
    }
  }

  /// Logging
  int index = storage_index_;
  if (LOGGING && (index < StoreLen)) {
    q_bk[index] = q;
    qd_bk[index] = qd;
    des_q_bk[index] = des_q;
    des_qd_bk[index] = des_qd;
    u_bk[index] = u;
    d_effort_bk[index] = d_effort;
    p_effort_bk[index] = p_effort;
    gp_effort_bk[index] = gp_effort;
    measured_effort_bk[index] = measured_effort;
    dt_bk(index) = robot_->getTime().now().toSec();

    storage_index_++;
  }
}

/// Controller stopping in realtime
void TreeControllerClass::stopping()
{}


bool TreeControllerClass::setJointGains(tree_controller_pkg::SetJointGains::Request &req, tree_controller_pkg::SetJointGains::Response &resp){
  for(uint i =0;i<Kp.d0;i++) {
    Kp(i) = req.pos_gains[i];
    Kd(i) = req.vel_gains[i];
  }
  qd_filt = req.qd_filt;
  return true;
}
bool TreeControllerClass::getJointGains(tree_controller_pkg::GetJointGains::Request &req, tree_controller_pkg::GetJointGains::Response &resp){
  resp.pos_gains.resize(Kp.d0);
  resp.vel_gains.resize(Kd.d0);
  for(uint i =0;i<Kd.d0;i++) {
    resp.pos_gains[i] = Kp(i);
    resp.vel_gains[i] = Kd(i);
  }
  resp.qd_filt = qd_filt;
  return true;
}
bool TreeControllerClass::getJointState(tree_controller_pkg::GetJointState::Request &req, tree_controller_pkg::GetJointState::Response &resp){
  resp.q.resize(q.d0);
  resp.qd.resize(q.d0);
  for(uint i =0;i<q.d0;i++) {
    resp.q[i] = q(i);
    resp.qd[i] = qd(i);
  }
  return true;
}
bool TreeControllerClass::startLogging(tree_controller_pkg::StartLogging::Request &req, tree_controller_pkg::StartLogging::Response &resp){
  storage_index_ = 0;
  LOGGING=true;
  return true;
}
bool TreeControllerClass::stopLogging(tree_controller_pkg::StopLogging::Request &req, tree_controller_pkg::StopLogging::Response &resp){
  LOGGING = false;
  write(LIST<arr>(q_bk),STRING("q_bk.output"));
  write(LIST<arr>(qd_bk),STRING("qd_bk.output"));
  write(LIST<arr>(q_filt_bk),STRING("q_filt_bk.output"));
  write(LIST<arr>(qd_filt_bk),STRING("qd_filt_bk.output"));
  write(LIST<arr>(des_q_bk),STRING("des_q_bk.output"));
  write(LIST<arr>(des_qd_bk),STRING("des_qd_bk.output"));
  write(LIST<arr>(u_bk),STRING("u_bk.output"));
  write(LIST<arr>(measured_effort_bk),STRING("measured_effort_bk.output"));
  write(LIST<arr>(p_effort_bk),STRING("p_effort_bk.output"));
  write(LIST<arr>(gp_effort_bk),STRING("gp_effort_bk.output"));
  write(LIST<arr>(d_effort_bk),STRING("d_effort_bk.output"));
  write(LIST<arr>(dt_bk),STRING("dt_bk.output"));
  write(LIST<arr>(ARR(storage_index_)),STRING("storage_index.output"));
  storage_index_ = 0;
  return true;
}

bool TreeControllerClass::setCommand(tree_controller_pkg::SetCommand::Request &req, tree_controller_pkg::SetCommand::Response &resp) {
  for(uint i =0;i<q.d0;i++) {
    des_q(i) = req.q[i];
    des_qd(i) = req.qd[i];
    gp_effort(i) =  req.uGP[i];
  }

  return true;
}
} // namespace

PLUGINLIB_DECLARE_CLASS(tree_controller_pkg,TreeControllerPlugin, tree_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
