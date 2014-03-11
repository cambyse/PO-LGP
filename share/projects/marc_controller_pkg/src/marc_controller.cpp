#include "marc_controller_pkg/marc_controller.h"
#include <pluginlib/class_list_macros.h>
#include <Core/array-vector.h>
#include <iomanip>

namespace marc_controller_ns {

/// Controller initialization in non-realtime
bool TreeControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &nh){
  cout <<"*** Starting TreeControllerClass" <<endl;

  //-- setup pr2 tree and message buffers
  if(!pr2_tree.init(robot)) { ROS_ERROR("Could not load robot tree");  return false; }
  jnt_pos_.resize(pr2_tree.size());
  jnt_vel_.resize(pr2_tree.size());
  jnt_efforts_.resize(pr2_tree.size());
  ROS_INFO("Tree sizes: %d",pr2_tree.size());
  pr2_tree.getPositions(jnt_pos_);

  //-- match ROS and ORS joint ids
  world <<FILE("model.kvg");
  ROS_INFO("ORS model loaded");
  ROS_qIndex.resize(world.q.N) = UINT_MAX;
  q.resize(world.q.N).setZero();
  qd.resize(world.q.N).setZero();
  Kp.resize(world.q.N).setZero();
  Kd.resize(world.q.N).setZero();
  limits.resize(world.q.N,4).setZero();
  for(uint i=0;i<(uint)pr2_tree.size();i++) {
//    ROS_INFO("Joint Name %d: %s: %f", i, pr2_tree.getJoint(i)->joint_->name.c_str(), jnt_pos_(i));
    ors::Joint *j = world.getJointByName(pr2_tree.getJoint(i)->joint_->name.c_str());
    if(j){
//      cout <<"JOINT MATCH: ROS-id=" <<i <<" ORS-id=" <<j->qIndex <<endl;
      ROS_qIndex(j->qIndex) = i;
      q(j->qIndex) = jnt_pos_(i);
      arr *info;
      info = j->ats.getValue<arr>("gains");  if(info){ Kp(j->qIndex)=info->elem(0); Kd(j->qIndex)=info->elem(1); }
      info = j->ats.getValue<arr>("limits");  if(info){ limits(j->qIndex,0)=info->elem(0); limits(j->qIndex,1)=info->elem(1); }
      info = j->ats.getValue<arr>("ctrl_limits");  if(info){ limits(j->qIndex,2)=info->elem(0); limits(j->qIndex,3)=info->elem(1); }
    }
  }

  //-- output info on joints
  cout <<"*** JOINTS" <<endl;
  for_list(ors::Joint, j, world.joints){
    uint i = j->qIndex;
    if(ROS_qIndex(i)!=UINT_MAX){
      cout <<"  " <<i <<'\t' <<ROS_qIndex(i)
          <<" \tgains=" <<Kp(i) <<' '<<Kd(i)
         <<" \tlimits=" <<limits[i]
         <<" \t" <<pr2_tree.getJoint(ROS_qIndex(i))->joint_->name.c_str() <<endl;
    }else{
      cout <<"  " <<i <<" not matched " <<j->name <<endl;
    }
  }

  jointState_publisher = nh.advertise<marc_controller_pkg::JointState>("jointState", 1);
  jointReference_subscriber = nh.subscribe("jointReference", 1, &TreeControllerClass::jointReferenceSubscriber, this);

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

  //-- convert KDL to ORS
  for (uint i =0;i<q.N;i++) if(ROS_qIndex(i)!=UINT_MAX){
    q(i) = jnt_pos_(ROS_qIndex(i));
    qd(i) = qd_filt*qd(i) + (1.-qd_filt)*jnt_vel_.qdot(ROS_qIndex(i));
  }

  //-- publish joint state
  jointStateMsg.N = q.d0;
  jointStateMsg.q = VECTOR(q);
  jointStateMsg.qd = VECTOR(qd);
  jointState_publisher.publish(jointStateMsg);

  //-- PD on q_ref!
  if(q_ref.N!=q.N || qdot_ref.N!=qd.N) cout <<'#' <<flush; //hashes indicate that q_ref has wrong size...
  else{
    u = (Kp % (q_ref - q)) + (Kd % (qdot_ref - qd));

    //-- command efforts to KDL
    for (uint i=0;i<q.N;i++) if(ROS_qIndex(i)!=UINT_MAX){
      if(u(i)<-limits(i,3)) u(i)=-limits(i,3);
      if(u(i)> limits(i,3)) u(i)= limits(i,3);
      pr2_tree.getJoint(ROS_qIndex(i))->commanded_effort_ = u(i);
      pr2_tree.getJoint(ROS_qIndex(i))->enforceLimits();
    }
  }
}

/// Controller stopping in realtime
void TreeControllerClass::stopping() {}

void TreeControllerClass::jointReferenceSubscriber(const marc_controller_pkg::JointState::ConstPtr& msg){
//  cout <<"subscriber callback" <<endl;
  q_ref = ARRAY(msg->q);
  qdot_ref = ARRAY(msg->qd);
}

} // namespace

PLUGINLIB_DECLARE_CLASS(marc_controller_pkg,TreeControllerPlugin, marc_controller_ns::TreeControllerClass, pr2_controller_interface::Controller)
