#include "marc_controller_pkg/marc_controller.h"
#include <pluginlib/class_list_macros.h>
#include <Core/array-vector.h>
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
  ROS_INFO("*** ORS model loaded");
  q.resize(world.q.N).setZero();
  qd.resize(world.q.N).setZero();
  Kp.resize(world.q.N).setZero();
  Kd.resize(world.q.N).setZero();
  Kq_gainFactor=Kd_gainFactor=ARR(1.);
  Kf_gainFactor=ARR(0.);
  limits.resize(world.q.N,4).setZero();
  //read out gain parameters from ors data structure
  { for_list(ors::Joint, j, world.joints) if(j->qDim()>0){
    arr *info;
    info = j->ats.getValue<arr>("gains");  if(info){ Kp(j->qIndex)=info->elem(0); Kd(j->qIndex)=info->elem(1); }
    info = j->ats.getValue<arr>("limits");  if(info){ limits(j->qIndex,0)=info->elem(0); limits(j->qIndex,1)=info->elem(1); }
    info = j->ats.getValue<arr>("ctrl_limits");  if(info){ limits(j->qIndex,2)=info->elem(0); limits(j->qIndex,3)=info->elem(1); }
    } }

  //match joint names with ros joints
  ROS_joints.resize(world.q.N);
  ROS_joints = NULL;
  for_list(ors::Joint, j, world.joints) if(j->qDim()>0){
    pr2_mechanism_model::JointState *pr2_joint = robot->getJointState(j->name.p);
    if(pr2_joint){
      ROS_joints(j->qIndex) = pr2_joint;
      q(j->qIndex) = pr2_joint->position_;
      ROS_INFO("%s",STRING("Joint '" <<j->name <<"' matched in pr2 '" <<pr2_joint->joint_->name.c_str()
		      <<"' \tq=" <<q(j->qIndex)
		      <<" \tgains=" <<Kp(j->qIndex) <<' '<<Kd(j->qIndex)
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
  forceSensor_subscriber = nh.subscribe("/ft/l_gripper_motor", 1, &TreeControllerClass::forceSensor_subscriber_callback, this);

  ROS_INFO("*** TreeControllerClass Started");

  msgBlock=1;

  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting(){
  q_ref = q;
  qdot_ref = zeros(q.N);
  u_bias = zeros(q.N);
  q_filt = 0.;
  qd_filt = 0.95;
  velLimitRatio = effLimitRatio = 1.;
}

/// Controller update loop in realtime
void TreeControllerClass::update() {
  //-- get current point pos
  for (uint i=0;i<q.N;i++) if(ROS_joints(i)){
      q(i) = ROS_joints(i)->position_; //jnt_pos_(ROS_qIndex(i));
      qd(i) = qd_filt*qd(i) + (1.-qd_filt)* ROS_joints(i)->velocity_; //jnt_vel_.qdot(ROS_qIndex(i));
  }

  //-- publish joint state
  jointStateMsg.q = VECTOR(q);
  jointStateMsg.qdot = VECTOR(qd);
  jointStateMsg.fL = VECTOR(fL_obs);
  jointState_publisher.publish(jointStateMsg);

  mutex.lock(); //only inside here we use the msg values...

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
    if(Kq_gainFactor.N==1 && Kd_gainFactor.N==1){
      u += Kq_gainFactor.scalar()*(Kp % (q_ref - q));
      u += Kd_gainFactor.scalar()*(Kd % (qdot_ref - qd));
    }else if(Kq_gainFactor.d0==q.N && Kq_gainFactor.d1==q.N && Kd_gainFactor.N==1){
      u += Kp % (Kq_gainFactor*(q_ref - q)); //matrix multiplication!
      u += Kd_gainFactor.scalar()*(Kd % (qdot_ref - qd));
    }
    u += u_bias;

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
      base_cmd.linear.x = qdot_ref(j_worldTranslationRotation->qIndex+0);
      base_cmd.linear.y = qdot_ref(j_worldTranslationRotation->qIndex+1);
      base_cmd.angular.z = qdot_ref(j_worldTranslationRotation->qIndex+2);
      baseCommand_publisher.publish(base_cmd);
    }
  }

  mutex.unlock();
}

/// Controller stopping in realtime
void TreeControllerClass::stopping() {}

void TreeControllerClass::jointReference_subscriber_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
  mutex.lock();
  q_ref = ARRAY(msg->q);
  qdot_ref = ARRAY(msg->qdot);
  //  fL_ref = ARRAY(msg->fL);
  //  fR_ref = ARRAY(msg->fR);
  u_bias = ARRAY(msg->u_bias);
#define CP(x) x=ARRAY(msg->x); if(x.N>q_ref.N) x.reshape(q_ref.N, q_ref.N);
  CP(Kq_gainFactor);
  CP(Kd_gainFactor);
  CP(Kf_gainFactor);
#undef CP
  velLimitRatio = msg->velLimitRatio;
  effLimitRatio = msg->effLimitRatio;
  mutex.unlock();
}

void TreeControllerClass::forceSensor_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
  const geometry_msgs::Vector3 &f=msg->wrench.force;
  const geometry_msgs::Vector3 &t=msg->wrench.torque;
  fL_obs = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(marc_controller_pkg, TreeControllerPlugin, marc_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
