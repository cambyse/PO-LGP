#include "marc_controller_pkg/marc_controller.h"
#include <pluginlib/class_list_macros.h>
#include <iomanip>
#include <geometry_msgs/Twist.h>

namespace marc_controller_ns {

double marginMap(double x, double lo, double hi, double marginRatio=.1){
  double m=(hi-lo)*marginRatio;
  double lom = lo + m;
  double him = hi - m;
  if(x>=lom && x<=him) return 0.;
  if(x<lom) return (x-lom)/m;
  if(x>him) return (x-him)/m;
  return 0.;
}

/// Controller initialization in non-realtime
bool TreeControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh){
  ROS_INFO("*** Starting TreeControllerClass3");

  //-- match ROS and ORS joint ids
  ROS_INFO("*** trying to load ORS model... (failure means that model.kvg was not found)");
  world <<FILE("model.kvg");
  ROS_INFO("%s",STRING("*** ORS model loaded: " <<world.q.N <<"joints -- saved in 'z.model.kvg'").p);
  world >>FILE("z.model.kvg");
  q.resize(world.q.N).setZero();
  qd.resize(world.q.N).setZero();
  Kp_base.resize(world.q.N).setZero();
  Kd_base.resize(world.q.N).setZero();
  Kp=Kd=ARR(1.);
  KiFTL.clear();
  KiFTR.clear();
  Ki.clear();
  limits.resize(world.q.N,5).setZero();
  J_ft_invL.clear();
  J_ft_invR.clear();
  fL_offset = zeros(6);
  fR_offset = zeros(6);

  //read out gain parameters from ors data structure
  { for_list(mlr::Joint, j, world.joints) if(j->qDim()>0){
    arr *info;
    info = j->ats.find<arr>("gains");  if(info){ Kp_base(j->qIndex)=info->elem(0); Kd_base(j->qIndex)=info->elem(1); }
    info = j->ats.find<arr>("limits");  if(info){ limits(j->qIndex,0)=info->elem(0); limits(j->qIndex,1)=info->elem(1); }
    info = j->ats.find<arr>("ctrl_limits");  if(info){ limits(j->qIndex,2)=info->elem(0); limits(j->qIndex,3)=info->elem(1); limits(j->qIndex,4)=info->elem(2); }
    } }

  //match joint names with ros joints
  ROS_joints.resize(world.q.N);
  ROS_joints = NULL;
  for_list(mlr::Joint, j, world.joints) if(j->qDim()>0){
    pr2_mechanism_model::JointState *pr2_joint = robot->getJointState(j->name.p);
    if(pr2_joint){
      ROS_joints(j->qIndex) = pr2_joint;
      q(j->qIndex) = pr2_joint->position_;
      ROS_INFO("%s",STRING("Joint '" <<j->name <<"' matched in pr2 '" <<pr2_joint->joint_->name.c_str()
                      <<"' \tq=" <<q(j->qIndex)
                      <<" \tgains=" <<Kp_base(j->qIndex) <<' '<<Kd_base(j->qIndex)
                      <<" \tlimits=" <<limits[j->qIndex]).p);
    }else{
      ROS_INFO("%s",STRING("Joint '" <<j->name <<"' not matched in pr2").p);
    }
  }

  j_worldTranslationRotation = world.getJointByName("worldTranslationRotation");
  ROS_INFO("%s",STRING("*** WorldTranslationRotation found?:" <<j).p);

  ROS_INFO("*** starting publisher and subscriber");

  jointState_publisher = nh.advertise<marc_controller_pkg::JointState>("jointState", 1);
  baseCommand_publisher = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  jointReference_subscriber = nh.subscribe("jointReference", 1, &TreeControllerClass::jointReference_subscriber_callback, this);
  l_ft_subscriber = nh.subscribe("/ft/l_gripper_motor", 1, &TreeControllerClass::l_ft_subscriber_callback, this);
  r_ft_subscriber = nh.subscribe("/ft/r_gripper_motor", 1, &TreeControllerClass::r_ft_subscriber_callback, this);

  ROS_INFO("*** TreeControllerClass Started");

  msgBlock=1;
  iterationsSinceLastMsg=0;

  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting(){
  //-- get current joint pos
  for (uint i=0;i<q.N;i++) if(ROS_joints(i)){
    q(i) = ROS_joints(i)->position_;
  }

  q_ref = q;
  qdot_ref = zeros(q.N);
  u_bias = zeros(q.N);
  fL_err = zeros(1);
  fR_err = zeros(1);
  int_error = zeros(q.N);
  q_filt = 0.;
  qd_filt = 0.97;
  fL_gamma = 0.;
  fR_gamma = 0.;
  fL_offset = zeros(6);
  fR_offset = zeros(6);
  velLimitRatio = effLimitRatio = 1.;
}

/// Controller update loop in realtime
void TreeControllerClass::update() {
  //-- get current joint pos
  for (uint i=0;i<q.N;i++) if(ROS_joints(i)){
    q(i) = ROS_joints(i)->position_; //jnt_pos_(ROS_qIndex(i));
    qd(i) = qd_filt*qd(i) + (1.-qd_filt)* ROS_joints(i)->velocity_; //jnt_vel_.qdot(ROS_qIndex(i));
  }

  mutex.lock(); //only inside here we use the msg values...

  // stop when no messages arrive anymore
  if(iterationsSinceLastMsg < 100){
    iterationsSinceLastMsg++;
  }
  if(iterationsSinceLastMsg == 100){
    q_ref = q;
    qdot_ref = zeros(q.N);
    u_bias = zeros(q.N);
    iterationsSinceLastMsg++;
  }

  //-- PD on q_ref
  if(q_ref.N!=q.N || qdot_ref.N!=qd.N || u_bias.N!=q.N
     || velLimitRatio<=0. || velLimitRatio>1.01
     || effLimitRatio<=0. || effLimitRatio>1.01){
    //cout <<'#' <<flush; //hashes indicate that q_ref has wrong size...
    if(!msgBlock){
      ROS_INFO("%s",STRING("*** q_ref, qdot_ref or u_bias have wrong dimension, or vel/eff limit ratios outside (0,1]"
                           <<q.N <<' ' <<q_ref.N <<' ' <<qdot_ref.N <<' ' <<u_bias.N <<' ' <<velLimitRatio <<' ' <<effLimitRatio).p);
      msgBlock=1000;
    }else{
      msgBlock--;
    }
  }else{
    if(msgBlock){
      ROS_INFO("%s",STRING("*** all is good: vel/eff ratios=" <<velLimitRatio <<' ' <<effLimitRatio).p);
      msgBlock=0;
    }

    u = zeros(q.N);
    if(Kp.N==1 && Kd.N==1){
      u += Kp_base % (Kp.scalar() * (q_ref - q));
      u += Kd_base % (Kd.scalar() * (qdot_ref - qd));
    }else if(Kp.d0 == q.N && Kp.d1 == q.N && Kd.d0 == q.N && Kd.d1 == q.N) {
      u += Kp*(q_ref - q);
      u += Kd*(qdot_ref - qd);
    }

    // add integral term
    if(Ki.N){
      if(Ki.N==1){
	int_error += Kp_base % (Ki.scalar() * 0.01 * (q_ref - q));
      }else if(Ki.d0 == q.N && Ki.d1 == q.N){
	int_error += Ki * (0.01 * (q_ref - q));
      }else{
	int_error = 0.;
      }
      for (uint i=0;i<q.N;i++) if(ROS_joints(i)){
        clip(int_error(i), -intLimitRatio*limits(i,4), intLimitRatio*limits(i,4));
      }
      u += int_error;
    }else{
      int_error = 0.;
    }

    u += u_bias;


    // torque PD for left ft sensor
      //TODO: How to allow multiple Tasks? Upper AND Lower bounds simultaneously?
    if (!KiFTL.N) {   // no contact or Ki gain -> don't use the integral term
      fL_err = fL_err*0.;              // reset integral error
    } else {
      calcFTintegral(fL_err,fL_ref,fL_obs,J_ft_invL,fL_gamma);
      u += KiFTL * fL_err;
    }
    // torque PD for right ft sensor
    if (!KiFTR.N) {   // no contact or Ki gain -> don't use the integral term
      fR_err = fR_err*0.;              // reset integral error
    } else {
      calcFTintegral(fR_err,fR_ref,fR_obs,J_ft_invR,fR_gamma);
      u += KiFTR * fR_err;
    }

    //-- command efforts to KDL
    for (uint i=0;i<q.N;i++) if(ROS_joints(i)){
        /*double velM = marginMap(qd(i), -velLimitRatio*limits(i,2), velLimitRatio*limits(i,2), .1);
          //clip(velM, -1., 1.)
      if(velM<0. && u(i)<0.) u(i)*=(1.+velM); //decrease effort close to velocity margin
      if(velM>0. && u(i)>0.) u(i)*=(1.-velM); //decrease effort close to velocity margin
        */
      clip(u(i), -effLimitRatio*limits(i,3), effLimitRatio*limits(i,3));
      ROS_joints(i)->commanded_effort_ = u(i);
      ROS_joints(i)->enforceLimits();
    }

    //-- command twist to base
    if(j_worldTranslationRotation && j_worldTranslationRotation->qDim()==3){
      geometry_msgs::Twist base_cmd;
      double phi = q_ref(j_worldTranslationRotation->qIndex+2);
      double vx  = qdot_ref(j_worldTranslationRotation->qIndex+0);
      double vy  = qdot_ref(j_worldTranslationRotation->qIndex+1);
      double co  = cos(phi), si = -sin(phi);
      base_cmd.linear.x = co*vx - si*vy;
      base_cmd.linear.y = si*vx + co*vy;
      base_cmd.angular.z = qdot_ref(j_worldTranslationRotation->qIndex+2);
      baseCommand_publisher.publish(base_cmd);
    }
  }

  mutex.unlock();

  //-- publish joint state
  jointStateMsg.q = conv_arr2stdvec(q);
  jointStateMsg.qdot = conv_arr2stdvec(qd);
  jointStateMsg.fL = conv_arr2stdvec(fL_obs);
  jointStateMsg.fR = conv_arr2stdvec(fR_obs);
  jointStateMsg.u_bias = conv_arr2stdvec(u);
  jointStateMsg.fL_err = conv_arr2stdvec(fL_err);
  jointStateMsg.fR_err = conv_arr2stdvec(fR_err);

  jointState_publisher.publish(jointStateMsg);
}

/// Controller stopping in realtime
void TreeControllerClass::stopping() {}

void TreeControllerClass::jointReference_subscriber_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
  mutex.lock();
  iterationsSinceLastMsg=0;
  q_ref = conv_stdvec2arr(msg->q);
  qdot_ref = conv_stdvec2arr(msg->qdot);
  u_bias = conv_stdvec2arr(msg->u_bias);
  fL_ref = conv_stdvec2arr(msg->fL);
  fR_ref = conv_stdvec2arr(msg->fR);
  J_ft_invL = conv_stdvec2arr(msg->J_ft_invL); if (J_ft_invL.N>0) J_ft_invL.reshape(fL_ref.d0,6);
  J_ft_invR = conv_stdvec2arr(msg->J_ft_invR); if (J_ft_invR.N>0) J_ft_invR.reshape(fR_ref.d0,6);
#define CP(x) x=conv_stdvec2arr(msg->x); if(x.N>q_ref.N) x.reshape(q_ref.N, q_ref.N); //this is needed to read matrices!!!
  CP(Kp);
  CP(Kd);
  CP(Ki);
  CP(KiFTR);
  CP(KiFTL);
#undef CP
  fR_offset = conv_stdvec2arr(msg->fR_offset);
  fL_offset = conv_stdvec2arr(msg->fL_offset);

  velLimitRatio = msg->velLimitRatio;
  effLimitRatio = msg->effLimitRatio;
  intLimitRatio = msg->intLimitRatio;
  fR_gamma = msg->fR_gamma;
  fL_gamma = msg->fL_gamma;
  qd_filt = msg->qd_filt;

  mutex.unlock();
}

void TreeControllerClass::l_ft_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  const geometry_msgs::Vector3 &f=msg->wrench.force;
  const geometry_msgs::Vector3 &t=msg->wrench.torque;
  fL_obs = ARR(f.x, f.y, f.z, t.x, t.y, t.z) - fL_offset;
}

void TreeControllerClass::r_ft_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  const geometry_msgs::Vector3 &f=msg->wrench.force;
  const geometry_msgs::Vector3 &t=msg->wrench.torque;
  fR_obs = ARR(f.x, f.y, f.z, t.x, t.y, t.z) - fR_offset;
}

void TreeControllerClass::calcFTintegral(arr& f_err, const arr& f_ref, const arr& f_obs, const arr& J_ft_inv, const double& f_gamma){
  // check if f_err has same dimension as f_ref, otherwise reset to zero
  if (f_err.N != f_ref.N) {
    f_err = zeros(f_ref.N);
  }

  f_err *= f_gamma;
  arr f_task = J_ft_inv*f_obs;

  for(uint i=0;i<f_task.N;i++) {
    if(f_ref(i) < 0) {
      if(f_task(i) < f_ref(i)) {
        f_err(i) += f_ref(i) - f_task(i);
      }
    } else {
      if(f_task(i) > f_ref(i)) {
        f_err(i) += f_ref(i) - f_task(i);
      }
    }
  }
}
} // namespace

PLUGINLIB_DECLARE_CLASS(marc_controller_pkg, TreeControllerPlugin, marc_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
