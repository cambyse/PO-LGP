#include "tree_controller_pkg/tree_controller.h"
#include <pluginlib/class_list_macros.h>


namespace tree_controller_ns {

/// Controller initialization in non-realtime
bool TreeControllerClass::init(pr2_mechanism_model::RobotState *robot,
                               ros::NodeHandle &n)
{
  std::string joint_name;
  if (!tree_.init(robot))
  {
    ROS_ERROR("Could not load robot tree");
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  ROS_INFO("Tree sizes: %d",tree_.size());
  int i;
  for(i=0;i<tree_.size();i++) {
    ROS_INFO("Joint Name %d: %s, Max Position: %f, Min Position: %f",i,tree_.getJoint(i)->joint_->name.c_str(),tree_.getJoint(i)->joint_statistics_.max_position_,tree_.getJoint(i)->joint_statistics_.min_position_);
  }

  jnt_pos_.resize(tree_.size());
  jnt_vel_.resize(tree_.size());
  jnt_efforts_.resize(tree_.size());

  tree_.getPositions(jnt_pos_);
  for(i=0;i<tree_.size();i++) {
    ROS_INFO("Joint Namea %d: %s: %f",i,tree_.getJoint(i)->joint_->name.c_str(), jnt_pos_(i));
  }

  // init Services
  setPosTargetSrv_ =  n.advertiseService("set_pos_target", &TreeControllerClass::setPosTarget, this);
  getPosTargetSrv_ =  n.advertiseService("get_pos_target", &TreeControllerClass::getPosTarget, this);
  setVecTargetSrv_ =  n.advertiseService("set_vec_target", &TreeControllerClass::setVecTarget, this);
  getVecTargetSrv_ =  n.advertiseService("get_vec_target", &TreeControllerClass::getVecTarget, this);
  getTaskStateSrv_ =   n.advertiseService("get_task_state", &TreeControllerClass::getTaskState, this);
  setJointGainsSrv_ =  n.advertiseService("set_joint_gains", &TreeControllerClass::setJointGains, this);
  getJointGainsSrv_ =  n.advertiseService("get_joint_gains", &TreeControllerClass::getJointGains, this);

  // init ORS
  world = new ors::KinematicWorld("scene");
  MP = new FeedbackMotionControl(*world,false);
  regularization = 1e-3;
  tau_control = 0.001;
  tau_plan = 0.02;
  MP->nullSpacePD.prec=0.;
  taskPos = MP->addPDTask("pos", tau_plan*5, 1, posTMT, "endeff");
  taskVec = MP->addPDTask("vec", tau_plan*5, 1, vecTMT, "endeff",ARR(0.,0.,1.));
  taskHome = MP->addPDTask("home", .02, 0.5, qItselfTMT);
  taskPos->setGains(1.,100.); taskPos->prec=1e0;
  taskVec->setGains(1.,100.); taskVec->prec=1e0;
  taskHome->setGains(0.,10.); taskHome->prec=1e-1;

  controlIdx = {30,31,32,33,34,35,36};
  q = arr(controlIdx.d0);
  qd = arr(controlIdx.d0);       qd.setZero();
  des_q = arr(controlIdx.d0);
  des_qd = arr(controlIdx.d0);   des_qd.setZero();

  tree_.getPositions(jnt_pos_);
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    des_q(i) = jnt_pos_(controlIdx(i));
  }

  world->setJointState(q,qd);
  taskPos->y_ref = ARRAY(world->getBodyByName("endeff")->X.pos);
  taskPos->v_ref = ARR(0.,0.,0.);
  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting()
{
  // 30: r_shoulder_pan_joint,   150 60
  // 31: r_shoulder_lift_joint,  150 60
  // 32: r_upper_arm_roll_joint,  30  4
  // 33: r_elbow_flex_joint,      30 10
  // 34: r_forearm_roll_joint,    10  2
  // 35: r_wrist_flex_joint,       6  2
  // 36: r_wrist_roll_joint,       6  2
  Kp = {150,150,30,30,10,6,6};
  Kd = { 60, 60, 4,10, 2,2,2};
}

/// Controller update loop in realtime
void TreeControllerClass::update()
{
  // pull pos & vel
  tree_.getPositions(jnt_pos_);
  tree_.getVelocities(jnt_vel_);
  tree_.getEfforts(jnt_efforts_);

  // Convert KDL to ORS
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    qd(i) = jnt_vel_.qdot(controlIdx(i));
  }

  // set current state
  MP->setState(q,qd);

  // OSC
  qdd = MP->operationalSpaceControl();
  des_q += tau_control*des_qd;
  des_qd += tau_control*qdd;

  p_effort = (Kp % (des_q - q));
  d_effort = (Kd % (des_qd - qd));
  //  i_effort = i_gain * integral;

  // Convert ORS to KDL
  for (uint i =0;i<controlIdx.d0;i++) {
//    tree_.getJoint(controlIdx(i))->commanded_effort_ = p_effort(i) + d_effort(i);
  }
}

/// Controller stopping in realtime
void TreeControllerClass::stopping()
{}

bool TreeControllerClass::setPosTarget(tree_controller_pkg::SetPosTarget::Request &req, tree_controller_pkg::SetPosTarget::Response &resp)
{
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    taskPos->y_ref(i) = req.pos[i];
    taskPos->v_ref(i) = req.vel[i];
  }
  return true;
}

bool TreeControllerClass::getPosTarget(tree_controller_pkg::GetPosTarget::Request &req, tree_controller_pkg::GetPosTarget::Response &resp)
{
  resp.pos.resize(taskPos->y_ref.d0);
  resp.vel.resize(taskPos->v_ref.d0);
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    resp.pos[i] = taskPos->y_ref(i);
    resp.vel[i] = taskPos->v_ref(i);
  }
  return true;
}

bool TreeControllerClass::setVecTarget(tree_controller_pkg::SetVecTarget::Request &req, tree_controller_pkg::SetVecTarget::Response &resp)
{
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    taskPos->y_ref(i) = req.pos[i];
    taskPos->v_ref(i) = req.vel[i];
  }
  return true;
}

bool TreeControllerClass::getVecTarget(tree_controller_pkg::GetVecTarget::Request &req, tree_controller_pkg::GetVecTarget::Response &resp)
{
  resp.pos.resize(taskVec->y_ref.d0);
  resp.vel.resize(taskVec->v_ref.d0);
  for(uint i =0;i<taskVec->y_ref.d0;i++) {
    resp.pos[i] = taskVec->y_ref(i);
    resp.vel[i] = taskVec->v_ref(i);
  }
  return true;
}

bool TreeControllerClass::getTaskState(tree_controller_pkg::GetTaskState::Request &req, tree_controller_pkg::GetTaskState::Response &resp)
{
  world->setJointState(q,qd);
  y = ARRAY(world->getBodyByName("endeff")->X.pos);
  yd = ARRAY(world->getBodyByName("endeff")->X.vel);

  resp.y.resize(y.d0);
  resp.yd.resize(yd.d0);
  for(uint i =0;i<y.d0;i++) {
    resp.y[i] = y(i);
    resp.yd[i] = yd(i);
  }
  return true;
}

bool TreeControllerClass::setJointGains(tree_controller_pkg::SetJointGains::Request &req, tree_controller_pkg::SetJointGains::Response &resp)
{
  for(uint i =0;i<Kp.d0;i++) {
    Kp(i) = req.pos_gains[i];
    Kd(i) = req.vel_gains[i];
  }
  return true;
}

bool TreeControllerClass::getJointGains(tree_controller_pkg::GetJointGains::Request &req, tree_controller_pkg::GetJointGains::Response &resp)
{
  resp.pos_gains.resize(Kp.d0);
  resp.vel_gains.resize(Kd.d0);
  for(uint i =0;i<Kp.d0;i++) {
    resp.pos_gains[i] = Kp(i);
    resp.vel_gains[i] = Kd(i);
  }
  return true;
}

} // namespace

/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(tree_controller_pkg,TreeControllerPlugin,
                        tree_controller_ns::TreeControllerClass,
                        pr2_controller_interface::Controller)

