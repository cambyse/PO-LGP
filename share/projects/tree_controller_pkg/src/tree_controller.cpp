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
  setPosTargetSrv_ =  n.advertiseService("set_pos_target", &TreeControllerClass::setPosTarget, this);
  getPosTargetSrv_ =  n.advertiseService("get_pos_target", &TreeControllerClass::getPosTarget, this);
  setVecTargetSrv_ =  n.advertiseService("set_vec_target", &TreeControllerClass::setVecTarget, this);
  getVecTargetSrv_ =  n.advertiseService("get_vec_target", &TreeControllerClass::getVecTarget, this);
  getTaskStateSrv_ =   n.advertiseService("get_task_state", &TreeControllerClass::getTaskState, this);
  setJointGainsSrv_ =  n.advertiseService("set_joint_gains", &TreeControllerClass::setJointGains, this);
  getJointGainsSrv_ =  n.advertiseService("get_joint_gains", &TreeControllerClass::getJointGains, this);
  getJointStateSrv_ =  n.advertiseService("get_joint_state", &TreeControllerClass::getJointState, this);
  getFilterGainsSrv_ = n.advertiseService("get_filter_gains", &TreeControllerClass::getFilterGains, this);
  setFilterGainsSrv_ = n.advertiseService("set_filter_gains", &TreeControllerClass::setFilterGains, this);
  getTaskGainsSrv_ = n.advertiseService("get_task_gains", &TreeControllerClass::getTaskGains, this);
  setTaskGainsSrv_ = n.advertiseService("set_task_gains", &TreeControllerClass::setTaskGains, this);
  startLoggingSrv_ = n.advertiseService("start_logging", &TreeControllerClass::startLogging, this);
  stopLoggingSrv_ = n.advertiseService("stop_logging", &TreeControllerClass::stopLogging, this);
  setNaturalGainsSrv_ = n.advertiseService("set_natural_gains", &TreeControllerClass::setNaturalGains, this);

  /// init ORS
  world = new ors::KinematicWorld("scene");
  MP = new FeedbackMotionControl(*world,false);
  tau_control = 0.001;

  /// Initialize PD Controller
  MP->nullSpacePD.active=false;
  taskPos = MP->addPDTask("pos", .1, 1, posTMT, "endeffR");
  taskVec = MP->addPDTask("vec", .1, 1, vecTMT, "endeffR",ARR(0.,0.,1.));
  taskHome = MP->addPDTask("home", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, 0.01*MP->H_rate_diag);

  double t_PD = 1.;
  double damp_PD = 0.9;
  taskPos->setGainsAsNatural(t_PD,damp_PD);
  taskVec->setGainsAsNatural(t_PD,damp_PD);
  taskHome->setGainsAsNatural(t_PD,damp_PD); taskHome->Pgain = 0.;
  taskPos->prec=1e5;
  taskVec->prec=1e3;
  taskHome->prec=1e2;


  controlIdx = {30,31,32,33,34,35,36};
  q = arr(controlIdx.d0);
  qd = arr(controlIdx.d0);       qd.setZero();
  des_q = arr(controlIdx.d0);
  des_qd = arr(controlIdx.d0);   des_qd.setZero();
  state = arr(3); stateVec = arr(3);
  integral = arr(controlIdx.d0); integral.setZero();
  i_claim = arr(controlIdx.d0); i_claim.setZero();
  u = arr(controlIdx.d0); u.setZero();
  measured_effort = arr(controlIdx.d0);

  /// Initialize Logging
  LOGGING = false;
  storage_index_ = 0;
  q_bk = arr(StoreLen,controlIdx.d0);
  qd_bk = arr(StoreLen,controlIdx.d0);
  des_q_bk = arr(StoreLen,controlIdx.d0);
  des_qd_bk = arr(StoreLen,controlIdx.d0);
  des_qdd_bk = arr(StoreLen,controlIdx.d0);
  u_bk = arr(StoreLen,controlIdx.d0);
  p_effort_bk = arr(StoreLen,controlIdx.d0);
  measured_effort_bk = arr(StoreLen,controlIdx.d0);
  d_effort_bk = arr(StoreLen,controlIdx.d0);
  i_effort_bk = arr(StoreLen,controlIdx.d0);
  a_effort_bk = arr(StoreLen,controlIdx.d0);
  dt_bk = arr(StoreLen);
  taskPos_y_bk = arr(StoreLen,3);
  taskPos_yRef_bk = arr(StoreLen,3);
  taskVec_y_bk = arr(StoreLen,3);
  taskVec_yRef_bk = arr(StoreLen,3);

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

  /// Initialize Task Space
  arr state,stateVec;
  world->setJointState(q,qd);
  world->kinematicsPos(state,NoArr,world->getBodyByName("endeffR")->index);
  world->kinematicsVec(stateVec,NoArr,world->getBodyByName("endeffR")->index);

  cout << "initial state: " << state << endl;
  cout << "initial stateVec: " << stateVec << endl;

  taskPos->y_ref = state;
  taskPos->v_ref = ARR(0.,0.,0.);
  taskVec->y_ref = stateVec;
  taskVec->v_ref = ARR(0.,0.,0.);

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
//  Ka = {0.05, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01};
  Ka = {0.15, 0.15, 0.15, 0.15, 0.1, 0.1, 0.1};
  Ki = {150,150,80,50,40,80,20}; Ki=Ki*0.1;

  integral.setZero();

  u_filt = 0.996;
  qd_filt = 0.9;

  /// Initialize joint state
  tree_.getPositions(jnt_pos_);
  for (uint i =0;i<controlIdx.d0;i++) {
    q(i) = jnt_pos_(controlIdx(i));
    des_q(i) = jnt_pos_(controlIdx(i));
  }

  /// Initialize Task Space
  arr state,stateVec;
  world->setJointState(q,qd);
  world->kinematicsPos(state,NoArr,world->getBodyByName("endeffR")->index);
  world->kinematicsVec(stateVec,NoArr,world->getBodyByName("endeffR")->index);

  cout << "initial state: " << state << endl;
  cout << "initial stateVec: " << stateVec << endl;

  taskPos->y_ref = state;
  taskPos->v_ref = ARR(0.,0.,0.);
  taskVec->y_ref = stateVec;
  taskVec->v_ref = ARR(0.,0.,0.);

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
    qd(i) = qd_filt*qd(i) + (1.-qd_filt)*jnt_vel_.qdot(controlIdx(i));
    measured_effort(i) = jnt_efforts_(controlIdx(i));
  }

  /// set current state
  MP->setState(q,qd);

  integral = integral + (des_q - q);

  /// OSC
  qdd = MP->operationalSpaceControl();
  des_q = q + tau_control*des_qd;
  des_qd = qd + tau_control*qdd;

  i_effort = Ki % integral;
  a_effort = Ka % qdd;

  /// Convert ORS to KDL
  for (uint i =0;i<controlIdx.d0;i++) {
    // Limit i_effort to i_claim values
    if (i_effort(i) > i_claim(i)) {
      i_effort(i) = i_claim(i);
    } else if (i_effort(i) < (-1.*i_claim(i))) {
      i_effort(i) = -1.*i_claim(i);
    }

    u(i) = u_filt*u(i)+(1.-u_filt)*(a_effort(i) + i_effort(i));
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
    des_qdd_bk[index] = qdd;
    u_bk[index] = u;
    i_effort_bk[index] = i_effort;
    a_effort_bk[index] = a_effort;
    measured_effort_bk[index] = measured_effort;
    dt_bk(index) = robot_->getTime().now().toSec();
    taskPos_y_bk[index] = taskPos->y;
    taskPos_yRef_bk[index] = taskPos->y_ref;
    taskVec_y_bk[index] = taskVec->y;
    taskVec_yRef_bk[index] = taskVec->y_ref;

    storage_index_++;
  }
}

/// Controller stopping in realtime
void TreeControllerClass::stopping()
{}

bool TreeControllerClass::setPosTarget(tree_controller_pkg::SetPosTarget::Request &req, tree_controller_pkg::SetPosTarget::Response &resp){
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    taskPos->y_ref(i) = req.pos[i];
    taskPos->v_ref(i) = req.vel[i];
  }
  return true;
}

bool TreeControllerClass::getPosTarget(tree_controller_pkg::GetPosTarget::Request &req, tree_controller_pkg::GetPosTarget::Response &resp){
  resp.pos.resize(taskPos->y_ref.d0); resp.vel.resize(taskPos->v_ref.d0);
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    resp.pos[i] = taskPos->y_ref(i);
    resp.vel[i] = taskPos->v_ref(i);
  }
  return true;
}

bool TreeControllerClass::setVecTarget(tree_controller_pkg::SetVecTarget::Request &req, tree_controller_pkg::SetVecTarget::Response &resp){
  for(uint i =0;i<taskPos->y_ref.d0;i++) {
    taskVec->y_ref(i) = req.pos[i];
    taskVec->v_ref(i) = req.vel[i];
  }
  return true;
}

bool TreeControllerClass::getVecTarget(tree_controller_pkg::GetVecTarget::Request &req, tree_controller_pkg::GetVecTarget::Response &resp){
  resp.pos.resize(taskVec->y_ref.d0);
  resp.vel.resize(taskVec->v_ref.d0);
  for(uint i =0;i<taskVec->y_ref.d0;i++) {
    resp.pos[i] = taskVec->y_ref(i);
    resp.vel[i] = taskVec->v_ref(i);
  }
  return true;
}

bool TreeControllerClass::getTaskState(tree_controller_pkg::GetTaskState::Request &req, tree_controller_pkg::GetTaskState::Response &resp){
  state = taskPos->y;
  stateVec = taskVec->y;
  resp.pos.resize(state.d0);
  resp.vec.resize(stateVec.d0);
  for(uint i =0;i<state.d0;i++) {
    resp.pos[i] = state(i);
    resp.vec[i] = stateVec(i);
  }
  return true;
}
bool TreeControllerClass::setJointGains(tree_controller_pkg::SetJointGains::Request &req, tree_controller_pkg::SetJointGains::Response &resp){
  for(uint i =0;i<Ki.d0;i++) {
    Ka(i) = req.acc_gains[i];
    Ki(i) = req.i_gains[i];
    i_claim(i) = req.i_claim[i];
  }
  integral.setZero();
  return true;
}
bool TreeControllerClass::getJointGains(tree_controller_pkg::GetJointGains::Request &req, tree_controller_pkg::GetJointGains::Response &resp){
  resp.acc_gains.resize(Ka.d0);
  resp.i_gains.resize(Ki.d0);
  resp.i_claim.resize(i_claim.d0);
  for(uint i =0;i<Ka.d0;i++) {
    resp.acc_gains[i] = Ka(i);
    resp.i_gains[i] = Ki(i);
    resp.i_claim[i] = i_claim(i);
  }
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
bool TreeControllerClass::getFilterGains(tree_controller_pkg::GetFilterGains::Request &req, tree_controller_pkg::GetFilterGains::Response &resp){
  resp.u_filt = u_filt;
  resp.qd_filt = qd_filt;
  return true;
}
bool TreeControllerClass::setFilterGains(tree_controller_pkg::SetFilterGains::Request &req, tree_controller_pkg::SetFilterGains::Response &resp){
  u_filt = req.u_filt;
  qd_filt = req.qd_filt;
  return true;
}
bool TreeControllerClass::getTaskGains(tree_controller_pkg::GetTaskGains::Request &req, tree_controller_pkg::GetTaskGains::Response &resp){
  resp.pos_gains[0] = taskPos->Pgain;     resp.vel_gains[0] = taskPos->Dgain;     resp.precision[0] = taskPos->prec;
  resp.pos_gains[1] = taskVec->Pgain;     resp.vel_gains[1] = taskVec->Dgain;     resp.precision[1] = taskVec->prec;
  resp.pos_gains[2] = taskHome->Pgain;    resp.vel_gains[2] = taskHome->Dgain;    resp.precision[2] = taskHome->prec;
  return true;
}
bool TreeControllerClass::setTaskGains(tree_controller_pkg::SetTaskGains::Request &req, tree_controller_pkg::SetTaskGains::Response &resp){
  taskPos->setGains(req.pos_gains[0],req.vel_gains[0]); taskPos->prec=req.precision[0];
  taskVec->setGains(req.pos_gains[1],req.vel_gains[1]); taskVec->prec=req.precision[1];
  taskHome->setGains(req.pos_gains[2],req.vel_gains[2]); taskHome->prec=req.precision[2];
  return true;
}
bool TreeControllerClass::setNaturalGains(tree_controller_pkg::SetNaturalGains::Request &req, tree_controller_pkg::SetNaturalGains::Response &resp){
  taskPos->setGainsAsNatural(req.decayTime,req.dampingRatio);
  taskVec->setGainsAsNatural(req.decayTime,req.dampingRatio);
  taskHome->setGainsAsNatural(req.decayTime,req.dampingRatio); taskHome->Pgain = 0.;
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
  write(LIST<arr>(des_q_bk),STRING("des_q_bk.output"));
  write(LIST<arr>(des_qd_bk),STRING("des_qd_bk.output"));
  write(LIST<arr>(des_qdd_bk),STRING("des_qdd_bk.output"));
  write(LIST<arr>(u_bk),STRING("u_bk.output"));
  write(LIST<arr>(measured_effort_bk),STRING("measured_effort_bk.output"));
  write(LIST<arr>(p_effort_bk),STRING("p_effort_bk.output"));
  write(LIST<arr>(d_effort_bk),STRING("d_effort_bk.output"));
  write(LIST<arr>(i_effort_bk),STRING("i_effort_bk.output"));
  write(LIST<arr>(a_effort_bk),STRING("a_effort_bk.output"));
  write(LIST<arr>(dt_bk),STRING("dt_bk.output"));
  write(LIST<arr>(ARR(storage_index_)),STRING("storage_index.output"));
  write(LIST<arr>(taskPos_y_bk),STRING("taskPos_y_bk.output"));
  write(LIST<arr>(taskPos_yRef_bk),STRING("taskPos_yRef_bk.output"));
  write(LIST<arr>(taskVec_y_bk),STRING("taskVec_y_bk.output"));
  write(LIST<arr>(taskVec_yRef_bk),STRING("taskVec_yRef_bk.output"));
  storage_index_ = 0;
  return true;
}
} // namespace

PLUGINLIB_DECLARE_CLASS(tree_controller_pkg,TreeControllerPlugin, tree_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
