#include "roscom.h"
#include <ros/ros.h>
#include <Core/array-vector.h>
#include <marc_controller_pkg/JointState.h>

void rosCheckInit(){
  ros::init(MT::argc, MT::argv, "pr2_module");
}

bool rosOk(){
  return ros::ok();
}

struct sRosCom{
  ros::Subscriber sub_jointState;
  ros::Publisher pub_jointReference;
  RosCom *base;
  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg);
};

RosCom::RosCom():Module("RosCom"){
  s = new sRosCom;
  s->base=this;
  rosCheckInit();
}

void sRosCom::joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
//  cout <<"** joinstState_callback" <<endl;
  base->q_obs.set() = ARRAY(msg->q);
  base->qdot_obs.set() = ARRAY(msg->qd);
}

void RosCom::publishJointReference(){
  marc_controller_pkg::JointState jointRef;
  jointRef.N = 0;
  jointRef.q = VECTOR(q_ref.get()());
  jointRef.qd = VECTOR(qdot_ref.get()());
  s->pub_jointReference.publish(jointRef);
}

void RosCom::open(){
  ros::NodeHandle nh;
  s->sub_jointState = nh.subscribe("/tree_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
  s->pub_jointReference = nh.advertise<marc_controller_pkg::JointState>("/tree_rt_controller/jointReference", 1);
}

void RosCom::step(){
  ros::spinOnce();
}

void RosCom::close(){}

