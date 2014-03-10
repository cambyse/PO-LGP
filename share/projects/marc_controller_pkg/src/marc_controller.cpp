#include "marc_controller_pkg/marc_controller.h"
#include <pluginlib/class_list_macros.h>
#include <Core/array-vector.h>

namespace marc_controller_ns {

/// Controller initialization in non-realtime
bool TreeControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh){
  std::string joint_name;
  if (!pr2_tree.init(robot)) {
    ROS_ERROR("Could not load robot tree");
    return false;
  }

  // Print joint state
  jnt_pos_.resize(pr2_tree.size());
  jnt_vel_.resize(pr2_tree.size());
  jnt_efforts_.resize(pr2_tree.size());
  ROS_INFO("Tree sizes: %d",pr2_tree.size());
  int i;
  pr2_tree.getPositions(jnt_pos_);
  for(i=0;i<pr2_tree.size();i++) {
    ROS_INFO("Joint Name %d: %s: %f", i, pr2_tree.getJoint(i)->joint_->name.c_str(), jnt_pos_(i));
  }

  pr2_jointIndex = {30,31,32,33,34,35,36};
  q = arr(pr2_jointIndex.d0);
  qd = arr(pr2_jointIndex.d0);       qd.setZero();
  q_ref = arr(pr2_jointIndex.d0);
  qdot_ref = arr(pr2_jointIndex.d0);   qdot_ref.setZero();

  // initialize joint state
  pr2_tree.getPositions(jnt_pos_);
  for (uint i =0;i<pr2_jointIndex.d0;i++) {
    q(i) = jnt_pos_(pr2_jointIndex(i));
    q_ref(i) = jnt_pos_(pr2_jointIndex(i));
  }

  // define torque and joint limits for the used joints
  for (uint i =0;i<pr2_jointIndex.d0;i++) {
    double low_tmp, high_tmp;
    pr2_tree.getJoint(pr2_jointIndex(i))->getLimits(low_tmp,high_tmp);
    lowerEffortLimits.append(low_tmp*0.7);
    upperEffortLimits.append(high_tmp*0.7);
    if (pr2_tree.getJoint(pr2_jointIndex(i))->joint_->type ==pr2_tree.getJoint(pr2_jointIndex(i))->joint_->CONTINUOUS) {
      lowerJointLimits.append(-1e5);
      upperJointLimits.append(1e5);
    } else {
      lowerJointLimits.append(pr2_tree.getJoint(pr2_jointIndex(i))->joint_->limits->lower);
      upperJointLimits.append(pr2_tree.getJoint(pr2_jointIndex(i))->joint_->limits->upper);
    }
  }
  cout << "lowEffortLimits: " << lowerEffortLimits << endl;
  cout << "highEffortLimits: " << upperEffortLimits << endl;
  cout << "lowJointLimits: " << lowerJointLimits << endl;
  cout << "highJointLimits: " << upperJointLimits << endl;

  jointState_publisher = nh.advertise<marc_controller_pkg::JointState>("jointState", 1);
  jointReference_subscriber = nh.subscribe("jointReference", 1, &TreeControllerClass::jointReferenceSubscriber, this);

  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting(){
  //                                       ROS_GAINS
  //                                          P    D    I
  // 30: r_shoulder_pan_joint,   150 60    2400   18  800
  // 31: r_shoulder_lift_joint,  150 60    1200   10  700
  // 32: r_upper_arm_roll_joint,  30  4    1000    6  600
  // 33: r_elbow_flex_joint,      30 10     700    4  450
  // 34: r_forearm_roll_joint,    10  2     300    6  300
  // 35: r_wrist_flex_joint,       6  2     300    4  300
  // 36: r_wrist_roll_joint,       6  2     300    4  300
  //  Kp = {150,150,30,30,10,6,6};
  //  Kd = {60,60,4,10,2,2,2};
  Kp = {150,150,80,50,40,80,20};
  Kd = {60,60,10,10,10,30,20};

  //[1000,1,1] [100,100,100] [10000,100,100]

  q_filt = 0.;
  qd_filt = 0.95;
}

/// Controller update loop in realtime
void TreeControllerClass::update() {
  //-- pull pos & vel
  pr2_tree.getPositions(jnt_pos_);
  pr2_tree.getVelocities(jnt_vel_);
  pr2_tree.getEfforts(jnt_efforts_);

  //-- convert KDL to ORS
  for (uint i =0;i<pr2_jointIndex.d0;i++) {
    q(i) = jnt_pos_(pr2_jointIndex(i));
    qd(i) = qd_filt*qd(i) + (1.-qd_filt)*jnt_vel_.qdot(pr2_jointIndex(i));
  }

  //-- publish joint state
  jointStateMsg.N = q.d0;
  jointStateMsg.q = VECTOR(q);
  jointStateMsg.qd = VECTOR(qd);
  jointState_publisher.publish(jointStateMsg);

  //-- PD on q_ref!
  u = (Kp % (q_ref - q)) + (Kd % (qdot_ref - qd));

  //-- command efforts to KDL
  for (uint i =0;i<pr2_jointIndex.d0;i++) {
    if (u(i)>upperEffortLimits(i) || u(i)<lowerEffortLimits(i)) {
      ROS_ERROR("SAFETY CHECK FAILED! Max u(%d): %f , Min u(%d): %f", i, upperEffortLimits(i),i,lowerEffortLimits(i));
      if(u(i)>upperEffortLimits(i)) u(i)=upperEffortLimits(i);
      if(u(i)<lowerEffortLimits(i)) u(i)=lowerEffortLimits(i);
    }
    pr2_tree.getJoint(pr2_jointIndex(i))->commanded_effort_ = u(i);
    pr2_tree.getJoint(pr2_jointIndex(i))->enforceLimits();
  }
}

/// Controller stopping in realtime
void TreeControllerClass::stopping() {}

void TreeControllerClass::jointReferenceSubscriber(const marc_controller_pkg::JointState::ConstPtr& msg){
  q_ref = ARRAY(msg->q);
  qdot_ref = ARRAY(msg->qd);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(marc_controller_pkg,TreeControllerPlugin, marc_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
