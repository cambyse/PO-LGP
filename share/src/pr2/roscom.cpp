#include "roscom.h"

#ifdef MT_ROS
#include <ros/ros.h>
#include <Core/array-vector.h>
#include <ros_msg/JointState.h>

//===========================================================================

void rosCheckInit(){
  ros::init(MT::argc, MT::argv, "pr2_module");
}

bool rosOk(){
  return ros::ok();
}

//===========================================================================

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

//===========================================================================

struct sRosCom_Spinner{
};

void RosCom_Spinner::open(){
  ros::init(MT::argc, MT::argv, "RosCom_Spinner_module");
}
void RosCom_Spinner::step(){
  ros::spinOnce();
}
void RosCom_Spinner::close(){}

//===========================================================================

struct sRosCom_ControllerSync{
  ros::Subscriber sub_jointState;
  ros::Publisher pub_jointReference;
  RosCom_ControllerSync *base;
  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
  //  cout <<"** joinstState_callback" <<endl;
    CtrlMsg m(ARRAY(msg->q), ARRAY(msg->qdot), ARRAY(msg->fL), ARRAY(msg->fR));
    base->ctrl_obs.set() = m;
  }
};

void RosCom_ControllerSync::open(){
  s = new sRosCom_ControllerSync;
  s->base=this;
  rosCheckInit();
  ros::NodeHandle nh;
  s->sub_jointState = nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
  s->pub_jointReference = nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
}

void RosCom_ControllerSync::step(){
  JointControllerRefsMsg m = ctrl_refs.get();
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

void RosCom_ControllerSync::close(){
  delete s;
}

//===========================================================================

#else //MT_ROS
#endif

REGISTER_MODULE(RosCom_Spinner)
REGISTER_MODULE(RosCom_ControllerSync)
REGISTER_MODULE(RosCom_KinectSync)
REGISTER_MODULE(RosCom_HeadCamsSync)
REGISTER_MODULE(RosCom_ArmCamsSync)
