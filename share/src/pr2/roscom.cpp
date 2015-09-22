#include "roscom.h"

#ifdef MT_ROS
#include <ros/ros.h>
#include <Core/array-vector.h>
#include <ros_msg/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <Core/geo.h>
#include <tf/tf.h>


//===========================================================================
// HELPERS
void rosCheckInit(const char* module_name){
// TODO make static variables to singleton
  static Mutex mutex;
  static bool inited = false;

  mutex.lock();
  if(!inited) {
    ros::init(MT::argc, MT::argv, module_name, ros::init_options::NoSigintHandler);
    inited = true;
  }
  mutex.unlock();
}

ors::Transformation ros_cvrt(const tf::Transform &trans){
  ors::Transformation X;
  tf::Quaternion q = trans.getRotation();
  tf::Vector3 t = trans.getOrigin();
  X.rot.set(q.w(), q.x(), q.y(), q.z());
  X.pos.set(t.x(), t.y(), t.z());
  return X;
}

timespec cvrt(const ros::Time& time){
  return {time.sec, time.nsec};
}

bool rosOk(){
  return ros::ok();
}

//===========================================================================
// RosCom_Spinner
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
// CosCom_ControllerSync
struct sRosCom_ControllerSync{
  RosCom_ControllerSync *base;
  ros::NodeHandle nh;
  ros::Subscriber sub_jointState;
//  ros::Subscriber sub_odom;
  ros::Publisher pub_jointReference;

  void joinstState_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
    //  cout <<"** joinstState_callback" <<endl;
    CtrlMsg m(ARRAY(msg->q), ARRAY(msg->qdot), ARRAY(msg->fL), ARRAY(msg->fR), ARRAY(msg->u_bias), ARRAY(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
    base->ctrl_obs.set() = m;
  }
//  void odom_callback(const marc_controller_pkg::JointState::ConstPtr& msg){
//    //  cout <<"** joinstState_callback" <<endl;
//    CtrlMsg m(ARRAY(msg->q), ARRAY(msg->qdot), ARRAY(msg->fL), ARRAY(msg->fR), ARRAY(msg->u_bias), ARRAY(msg->J_ft_inv), msg->velLimitRatio, msg->effLimitRatio, msg->gamma);
//    base->ctrl_obs.set() = m;
//  }
};

void RosCom_ControllerSync::open(){
  rosCheckInit();
  s = new sRosCom_ControllerSync;
  s->base=this;
  s->sub_jointState = s->nh.subscribe("/marc_rt_controller/jointState", 1, &sRosCom_ControllerSync::joinstState_callback, s);
//  s->sub_odom = s->nh.subscribe("/robot_pose_ekf/odom_combined", 1, &sRosCom_ControllerSync::joinstState_callback, s);
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
  jointRef.KiFT = VECTOR(m.KiFT);
  jointRef.J_ft_inv = VECTOR(m.J_ft_inv);
  jointRef.velLimitRatio = m.velLimitRatio;
  jointRef.effLimitRatio = m.effLimitRatio;
  jointRef.intLimitRatio = m.intLimitRatio;
  jointRef.gamma = m.gamma;
  s->pub_jointReference.publish(jointRef);
}

void RosCom_ControllerSync::close(){
  s->nh.shutdown();
  delete s;
}

//===========================================================================
// Helper function so sync ors with the real PR2
void initialSyncJointStateWithROS(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  //-- wait for first q observation!
  cout << "** Waiting for ROS message of joints for initial configuration.." << endl
       << "   If nothing is happening: is the controller running?" << endl;

  for (uint trials = 0; trials < 20; trials++) {
    ctrl_obs.var->waitForNextRevision();
    cout << "REMOTE joint dimension=" << ctrl_obs.get()->q.N << endl;
    cout << "LOCAL  joint dimension=" << world.q.N << endl;

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      cout << "** Updating world state" << endl;
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
    cout << "retrying..." << endl;
  }
  HALT("sync'ing real PR2 with simulated failed");
}

void syncJointStateWitROS(ors::KinematicWorld& world,
    Access_typed<CtrlMsg>& ctrl_obs, bool useRos) {

  if (not useRos) { return; }

  for (uint trials = 0; trials < 2; trials++) {
    ctrl_obs.var->waitForNextRevision();

    if (ctrl_obs.get()->q.N == world.q.N and ctrl_obs.get()->qdot.N == world.q.N) {
      // set current state
      world.setJointState(ctrl_obs.get()->q, ctrl_obs.get()->qdot);
      return;
    }
  }
  HALT("sync'ing real PR2 with simulated failed");
}

//===========================================================================
// RosCom_KinectSync
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
// RosCom_CamsSync
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
// RosCom_ArmCamsSync
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
// RosCom_ForceSensorSync
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
// RosCom_SoftHandSync
struct sRosCom_SoftHandSync{
  RosCom_SoftHandSync *base;
  ros::NodeHandle nh;
  ros::Publisher pub_shReference;
};

void RosCom_SoftHandSync::open(){
  rosCheckInit();
  s = new sRosCom_SoftHandSync;
  s->base=this;
  s->pub_shReference = s->nh.advertise<std_msgs::String>("/softhand/grasp_ref", 1);
}

void RosCom_SoftHandSync::step(){
  SoftHandMsg shm = sh_ref.get();
  std_msgs::String refs;
  refs.data = shm.soft_hand_cmd.p;
  s->pub_shReference.publish(refs);
}

void RosCom_SoftHandSync::close(){
  s->nh.shutdown();
  delete s;
}


//===========================================================================
#else // MT_ROS no defined

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
