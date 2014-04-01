#include "roscom.h"
#include <ros/ros.h>
#include <Core/array-vector.h>
#include <pr2/marc_controller_pkg/msg_gen/cpp/include/marc_controller_pkg/JointState.h>

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
  CtrlMsg m(ARRAY(msg->q), ARRAY(msg->qdot), ARRAY(msg->fL), ARRAY(msg->fR));
  base->ctrl_obs.set() = m;
}

void RosCom::publishJointReference(){
  CtrlMsg m = ctrl_ref.get();
  marc_controller_pkg::JointState jointRef;
  jointRef.q = VECTOR(m.q);
  jointRef.qdot= VECTOR(m.qdot);
  jointRef.fL = VECTOR(m.fL);
  jointRef.fR = VECTOR(m.fR);
  jointRef.Kp_gainFactor = m.Kp_gainFactor;
  jointRef.Kd_gainFactor = m.Kd_gainFactor;
  jointRef.fL_gainFactor = m.fL_gainFactor;
  jointRef.fR_gainFactor = m.fR_gainFactor;
  s->pub_jointReference.publish(jointRef);
}

void RosCom::open(){
  ros::NodeHandle nh;
  s->sub_jointState = nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
  s->pub_jointReference = nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
}

void RosCom::step(){
  ros::spinOnce();
}

void RosCom::close(){}

