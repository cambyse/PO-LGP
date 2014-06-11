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
  ROS_INFO("*** Starting TreeControllerClass");

  //-- setup pr2 tree and message buffers
  if(!pr2_tree.init(robot)) { ROS_ERROR("Could not load robot tree");  return false; }
  jnt_pos_.resize(pr2_tree.size());
  jnt_vel_.resize(pr2_tree.size());
  jnt_efforts_.resize(pr2_tree.size());
  pr2_tree.getPositions(jnt_pos_);

  //-- match ROS and ORS joint ids
  ROS_INFO("*** trying to load ORS model... (failure means that model.kvg was not found)");
  world <<FILE("model.kvg");
  ROS_INFO("*** ORS model loaded");
  ftL_shape = world.getShapeByName("endeffForceL");
  ftR_shape = world.getShapeByName("endeffForceR");
  ROS_qIndex.resize(world.q.N) = UINT_MAX;
  q.resize(world.q.N).setZero();
  qd.resize(world.q.N).setZero();
  Kp.resize(world.q.N).setZero();
  Kd.resize(world.q.N).setZero();
  Kp_gainFactor=Kd_gainFactor=1.;
  fL_gainFactor=fR_gainFactor=0.;
  limits.resize(world.q.N,4).setZero();
  for(uint i=0;i<(uint)pr2_tree.size();i++) {
//    ROS_INFO("Joint Name %d: %s: %f", i, pr2_tree.getJoint(i)->joint_->name.c_str(), jnt_pos_(i));
    ors::Joint *j = world.getJointByName(pr2_tree.getJoint(i)->joint_->name.c_str());
    if(j && j->qDim()>0 && ROS_qIndex(j->qIndex)==UINT_MAX){ //only overwrite the association if not associated before
      ROS_qIndex(j->qIndex) = i;
      q(j->qIndex) = jnt_pos_(i);
      arr *info;
      info = j->ats.getValue<arr>("gains");  if(info){ Kp(j->qIndex)=info->elem(0); Kd(j->qIndex)=info->elem(1); }
      info = j->ats.getValue<arr>("limits");  if(info){ limits(j->qIndex,0)=info->elem(0); limits(j->qIndex,1)=info->elem(1); }
      info = j->ats.getValue<arr>("ctrl_limits");  if(info){ limits(j->qIndex,2)=info->elem(0); limits(j->qIndex,3)=info->elem(1); }
    }
  }

  //-- output info on joints
  ROS_INFO("*** JOINTS");
  for_list(ors::Joint, j, world.joints) if(j->qDim()>0){
    uint i = j->qIndex;
    if(ROS_qIndex(i)!=UINT_MAX){
      ROS_INFO(STRING("  " <<i <<'\t' <<ROS_qIndex(i)
		      <<" \tgains=" <<Kp(i) <<' '<<Kd(i)
		      <<" \tlimits=" <<limits[i]
		      <<" \t" <<pr2_tree.getJoint(ROS_qIndex(i))->joint_->name.c_str()
		      <<" \t" <<j->name));
    }else{
      ROS_INFO(STRING("  " <<i <<" not matched " <<j->name));
    }
  }

  j_worldTranslationRotation = world.getJointByName("worldTranslationRotation");
  ROS_INFO(STRING("*** WorldTranslationRotation found?:" <<j));

  ROS_INFO("*** starting publisher and subscriber");

  jointState_publisher = nh.advertise<marc_controller_pkg::JointState>("jointState", 1);
  baseCommand_publisher = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
  jointReference_subscriber = nh.subscribe("jointReference", 1, &TreeControllerClass::jointReference_subscriber_callback, this);
  forceSensor_subscriber = nh.subscribe("/ft/l_gripper_motor", 1, &TreeControllerClass::forceSensor_subscriber_callback, this);

  ROS_INFO("*** TreeControllerClass Started");

  return true;
}

/// Controller startup in realtime
void TreeControllerClass::starting(){
  q_ref = q;
  qdot_ref = qd;
  q_filt = 0.;
  qd_filt = 0.95;
}

/// Controller update loop in realtime
void TreeControllerClass::update() {
  //-- pull pos & vel
  pr2_tree.getPositions(jnt_pos_);
  pr2_tree.getVelocities(jnt_vel_);
  pr2_tree.getEfforts(jnt_efforts_);

  //-- get current point pos
  for (uint i=0;i<q.N;i++) if(ROS_qIndex(i)!=UINT_MAX){
    q(i) = jnt_pos_(ROS_qIndex(i));
    qd(i) = qd_filt*qd(i) + (1.-qd_filt)*jnt_vel_.qdot(ROS_qIndex(i));
  }

  //-- publish joint state
  jointStateMsg.q = VECTOR(q);
  jointStateMsg.qdot = VECTOR(qd);
  jointStateMsg.fL = VECTOR(fL_obs);
  jointState_publisher.publish(jointStateMsg);

  //-- update ORS to compute Jacobians used in force controller
  world.setJointState(q, qd);
  world.kinematicsPos(y_fL, J_fL, ftL_shape->body, &ftL_shape->rel.pos);
  world.kinematicsPos(y_fR, J_fR, ftR_shape->body, &ftR_shape->rel.pos);

  //-- PD on q_ref
  if(q_ref.N!=q.N || qdot_ref.N!=qd.N){
    cout <<'#' <<flush; //hashes indicate that q_ref has wrong size...
  }else{
    u = Kp_gainFactor*(Kp % (q_ref - q)) + Kd_gainFactor*(Kd % (qdot_ref - qd));

    if(fL_ref.N==3){
      u += fL_gainFactor*(~J_fL * fL_ref);
    }
    if(fR_ref.N==3){
      u += fR_gainFactor*(~J_fR * fR_ref);
    }

    //-- command efforts to KDL
    for (uint i=0;i<q.N;i++) if(ROS_qIndex(i)!=UINT_MAX){
      double velM = marginMap(qd(i), -limits(i,2), limits(i,2), .1);
      if(velM<0. && u(i)<0.) u(i)*=(1.+velM); //decrease effort close to velocity margin
      if(velM>0. && u(i)>0.) u(i)*=(1.-velM); //decrease effort close to velocity margin
      clip(u(i), -limits(i,3), limits(i,3));
      pr2_tree.getJoint(ROS_qIndex(i))->commanded_effort_ = u(i);
      pr2_tree.getJoint(ROS_qIndex(i))->enforceLimits();
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
}

/// Controller stopping in realtime
void TreeControllerClass::stopping() {}

void TreeControllerClass::jointReference_subscriber_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
//  cout <<"subscriber callback" <<endl;
  q_ref = ARRAY(msg->q);
  qdot_ref = ARRAY(msg->qdot);
  fL_ref = ARRAY(msg->fL);
  fR_ref = ARRAY(msg->fR);
#define CP(x) x=msg->x;  if(x>1.) x=1.;  if(x<0.) x=0.;
  CP(Kp_gainFactor);
  CP(Kd_gainFactor);
  CP(fL_gainFactor);
  CP(fR_gainFactor);
#undef CP
}

void TreeControllerClass::forceSensor_subscriber_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
//  cout <<"subscriber callback" <<endl;
  const geometry_msgs::Vector3 &f=msg->wrench.force;
  //const geometry_msgs::Vector3 &t=msg->wrench.torque;
  fL_obs = ARR(f.x, f.y, f.z);
  //ef = ARR(t.x, t.y, t.z);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(marc_controller_pkg, TreeControllerPlugin, marc_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
