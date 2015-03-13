#include "roscom.h"

#ifdef MT_ROS

#include <ros/ros.h>
#include <Core/array-vector.h>
#include <ros_msg/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/WrenchStamped.h>
#ifdef MLR_AR_TRACK
#  include <ar_track_alvar/AlvarMarkers.h>
#endif

//===========================================================================

void rosCheckInit(){
  ros::init(MT::argc, MT::argv, "pr2_module", ros::init_options::NoSigintHandler);
}

bool rosOk(){
  return ros::ok();
}

//===========================================================================

struct sRosCom_Spinner{
};

void RosCom_Spinner::open(){
  rosCheckInit();
}

void RosCom_Spinner::step(){
  ros::spinOnce();
}

void RosCom_Spinner::close(){}

//===========================================================================

struct sRosCom_ControllerSync{
  RosCom_ControllerSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_jointState;
  ros::Publisher pub_jointReference;
  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
    //  cout <<"** joinstState_callback" <<endl;
    CtrlMsg m(ARRAY(msg->q), ARRAY(msg->qdot), ARRAY(msg->fL), ARRAY(msg->fR), ARRAY(msg->u_bias), ARRAY(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
    base->ctrl_obs.set() = m;
  }
};

void RosCom_ControllerSync::open(){
  rosCheckInit();
  s = new sRosCom_ControllerSync;
  s->base=this;
  s->sub_jointState = s->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom_ControllerSync::joinstState_callback, s);
  s->pub_jointReference = s->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
  //  s->sub_jointState = s->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom::joinstState_callback, s);
  //  s->pub_jointReference = s->nh.advertise<marc_controller_pkg::JointState>("/marc_rt_controller/jointReference", 1);
}

void RosCom_ControllerSync::step(){
  CtrlMsg m = ctrl_ref.get();
  if(!m.q.N) return;
  marc_controller_pkg::JointState jointRef;
  jointRef.q = VECTOR(m.q);
  jointRef.qdot= VECTOR(m.qdot);
  jointRef.fL = VECTOR(m.fL);
  jointRef.u_bias = VECTOR(m.u_bias);
  jointRef.Kp = VECTOR(m.Kp);
  jointRef.Kd = VECTOR(m.Kd);
  jointRef.Ki = VECTOR(m.Ki);
  jointRef.J_ft_inv = VECTOR(m.J_ft_inv);
  jointRef.velLimitRatio = m.velLimitRatio;
  jointRef.effLimitRatio = m.effLimitRatio;
  jointRef.gamma = m.gamma;
  s->pub_jointReference.publish(jointRef);
}

void RosCom_ControllerSync::close(){
  s->nh.shutdown();
  delete s;
}

//===========================================================================

struct sRosCom_KinectSync{
  RosCom_KinectSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_rgb;
  ros::Subscriber sub_depth;
  void cb_rgb(const sensor_msgs::Image::ConstPtr& msg){
    //  cout <<"** sRosCom_KinectSync callback" <<endl;
    base->kinect_rgb.set() = ARRAY(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_depth(const sensor_msgs::Image::ConstPtr& msg){
    //  cout <<"** sRosCom_KinectSync callback" <<endl;
    byteA data = ARRAY(msg->data);
    uint16A ref((const uint16_t*)data.p, data.N/2);
    ref.reshape(msg->height, msg->width);
    base->kinect_depth.set() = ref;
  }
};

void RosCom_KinectSync::open(){
  rosCheckInit();
  s = new sRosCom_KinectSync;
  s->base = this;
  s->sub_rgb = s->nh.subscribe("/kinect_head/rgb/image_color", 1, &sRosCom_KinectSync::cb_rgb, s);
  s->sub_depth = s->nh.subscribe("/kinect_head/depth_registered/image_raw", 1, &sRosCom_KinectSync::cb_depth, s);
}

void RosCom_KinectSync::step(){
}

void RosCom_KinectSync::close(){
  s->nh.shutdown();
}

//===========================================================================

struct sRosCom_CamsSync{
  RosCom_CamsSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const sensor_msgs::Image::ConstPtr& msg){
    base->rgb_leftEye.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_right(const sensor_msgs::Image::ConstPtr& msg){
    base->rgb_rightEye.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
};

void RosCom_CamsSync::open(){
  rosCheckInit();
  s = new sRosCom_CamsSync;
  s->base = this;
  s->sub_left  = s->nh.subscribe("/wide_stereo/left/image_rect_color", 1, &sRosCom_CamsSync::cb_left, s);
  s->sub_right = s->nh.subscribe("/wide_stereo/right/image_rect_color", 1, &sRosCom_CamsSync::cb_right, s);
}

void RosCom_CamsSync::step(){
}

void RosCom_CamsSync::close(){
  s->nh.shutdown();
}

//===========================================================================

struct sRosCom_ArmCamsSync{
  RosCom_ArmCamsSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const sensor_msgs::Image::ConstPtr& msg){
    base->rgb_leftArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
  void cb_right(const sensor_msgs::Image::ConstPtr& msg){
    base->rgb_rightArm.set() = ARRAY<byte>(msg->data).reshape(msg->height, msg->width, 3);
  }
};

void RosCom_ArmCamsSync::open(){
  rosCheckInit();
  s = new sRosCom_ArmCamsSync;
  s->base = this;
  s->sub_left  = s->nh.subscribe("/l_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_left, s);
  s->sub_right = s->nh.subscribe("/r_forearm_cam/image_rect_color", 1, &sRosCom_ArmCamsSync::cb_right, s);
}

void RosCom_ArmCamsSync::step(){
}

void RosCom_ArmCamsSync::close(){
  s->nh.shutdown();
}

//===========================================================================

struct sRosCom_ForceSensorSync{
  RosCom_ForceSensorSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_left;
  ros::Subscriber sub_right;
  void cb_left(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    const geometry_msgs::Vector3 &f=msg->wrench.force;
    const geometry_msgs::Vector3 &t=msg->wrench.torque;
    base->wrenchL.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
  }
  void cb_right(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    const geometry_msgs::Vector3 &f=msg->wrench.force;
    const geometry_msgs::Vector3 &t=msg->wrench.torque;
    base->wrenchR.set() = ARR(f.x, f.y, f.z, t.x, t.y, t.z);
  }

};

void RosCom_ForceSensorSync::open(){
  rosCheckInit();
  s = new sRosCom_ForceSensorSync;
  s->base = this;
  s->sub_left  = s->nh.subscribe("/ft_sensor/ft_compensated", 1, &sRosCom_ForceSensorSync::cb_left, s);  // /ft/l_gripper_motor
//  s->sub_right = s->nh.subscribe("/ft_sensor/r_ft_compensated", 1, &sRosCom_ForceSensorSync::cb_right, s); // /ft/r_gripper_motor
}

void RosCom_ForceSensorSync::step(){
}

void RosCom_ForceSensorSync::close(){
  s->nh.shutdown();
}

//===========================================================================

#ifdef MLR_AR_TRACK

struct sRosCom_ARMarkerSync{
  RosCom_ARMarkerSync *base;
  ros::NodeHandle nh;
  ros::Subscriber ar_marker;
  void cb_sync(const ar_track_alvar::AlvarMarkers::ConstPtr& msg){
    uint N = 20;
    arr marker_pose;
    if (marker_pose.N==0){
      marker_pose = zeros(N,7);
    }else{
      marker_pose = base->marker_pose.get();
    }
    for (uint i = 0; i<msg->markers.size();i++) {
      const ar_track_alvar::AlvarMarker m = msg->markers.at(i);
      marker_pose[m.id] = ARR(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z, m.pose.pose.orientation.w);
    }
    base->marker_pose.set() = marker_pose;
  }
};

void RosCom_ARMarkerSync::open(){
  rosCheckInit();
  s = new sRosCom_ARMarkerSync;
  s->base = this;
  s->ar_marker  = s->nh.subscribe("/ar_pose_marker", 1, &sRosCom_ARMarkerSync::cb_sync, s);
}

void RosCom_ARMarkerSync::step(){
}

void RosCom_ARMarkerSync::close(){
  s->nh.shutdown();
}

#else

void RosCom_ARMarkerSync::open(){
  MT_MSG("compiler flags do not enable ARMarkers")
}

void RosCom_ARMarkerSync::step(){
}

void RosCom_ARMarkerSync::close(){
  MT_MSG("compiler flags do not enable disabled ARMarkers")
}

#endif

//===========================================================================

#else //MT_ROS

void RosCom_Spinner::open(){ NICO }
void RosCom_Spinner::step(){ NICO }
void RosCom_Spinner::close(){ NICO }

void RosCom_ControllerSync::open(){ NICO }
void RosCom_ControllerSync::step(){ NICO }
void RosCom_ControllerSync::close(){ NICO }

void RosCom_ForceSensorSync::open(){ NICO }
void RosCom_ForceSensorSync::step(){ NICO }
void RosCom_ForceSensorSync::close(){ NICO }

#endif

//REGISTER_MODULE(RosCom_Spinner)
//REGISTER_MODULE(RosCom_ControllerSync)
//REGISTER_MODULE(RosCom_KinectSync)
//REGISTER_MODULE(RosCom_HeadCamsSync)
//REGISTER_MODULE(RosCom_ArmCamsSync)
