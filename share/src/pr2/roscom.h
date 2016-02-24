#pragma once

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

//===========================================================================

#include <Core/module.h>
#include <Core/array.h>
#include <Geo/geo.h>
#include <Ors/ors.h>
#include <ros_msg/JointState.h>


//===========================================================================
//
// utils
//

//-- a basic message type for communication with the PR2 controller
struct CtrlMsg{
  arr q, qdot, fL, fR, J_ft_invL, J_ft_invR;
  arr Kp, Kd, Ki,u_bias, KiFTL, KiFTR, fL_offset, fR_offset, fL_err, fR_err;
  double velLimitRatio, effLimitRatio, intLimitRatio, fL_gamma, fR_gamma, qd_filt;
  CtrlMsg():Kp(ARR(1.)), Kd(ARR(1.)), Ki(ARR(0.)), u_bias(ARR(0.)),fL_offset(zeros(6)),fR_offset(zeros(6)), velLimitRatio(1.), effLimitRatio(1.), intLimitRatio(0.1),
  fL_gamma(0.),fR_gamma(0.),qd_filt(0.97) {}
  CtrlMsg(const arr& q, const arr& qdot,
          const arr& fL, const arr& fR, const arr& u_bias, const arr& fL_err, const arr& fR_err)
    :q(q), qdot(qdot), fL(fL), fR(fR), u_bias(u_bias), fL_err(fL_err), fR_err(fR_err){}
};

void rosCheckInit(const char* module_name="pr2_module");
bool rosOk();

//-- ROS <--> MLR
std_msgs::String    conv_string2string(const mlr::String&);
mlr::String         conv_string2string(const std_msgs::String&);
ors::Transformation conv_transform2transformation(const tf::Transform&);
ors::Transformation conv_pose2transformation(const geometry_msgs::Pose&);
ors::Vector         conv_point2vector(const geometry_msgs::Point& p);
ors::Quaternion     conv_quaternion2quaternion(const geometry_msgs::Quaternion& q);
void                conv_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped &pose);
arr                 conv_pose2transXYPhi(const geometry_msgs::PoseWithCovarianceStamped &pose);
double              conv_time2double(const ros::Time& time);
timespec            conv_time2timespec(const ros::Time&);
arr                 conv_wrench2arr(const geometry_msgs::WrenchStamped& msg);
byteA               conv_image2byteA(const sensor_msgs::Image& msg);
uint16A             conv_image2uint16A(const sensor_msgs::Image& msg);
arr                 conv_points2arr(const std::vector<geometry_msgs::Point>& pts);
arr                 conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts);
CtrlMsg             conv_JointState2CtrlMsg(const marc_controller_pkg::JointState& msg);
ors::KinematicWorld conv_MarkerArray2KinematicWorld(const visualization_msgs::MarkerArray& markers);

//-- MLR -> ROS
std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts);
marc_controller_pkg::JointState   conv_CtrlMsg2JointState(const CtrlMsg& ctrl);

//-- get transformations
ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener);
ors::Transformation ros_getTransform(const std::string& from, const std_msgs::Header& to, tf::TransformListener& listener);


//===========================================================================
//
// subscribing a message directly into an Access
//

template<class msg_type>
struct Subscriber {
  Access_typed<msg_type>& access;
  ros::NodeHandle* nh;
  ros::Subscriber sub;
  Subscriber(const char* topic_name, Access_typed<msg_type>& _access)
    : access(_access) {
    nh = new ros::NodeHandle;
    sub  = nh->subscribe( topic_name, 1, &Subscriber::callback, this);
  }
  ~Subscriber(){
    nh->shutdown();
    delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) { access.set() = *msg; }
};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConv {
  Access_typed<var_type>& access;
  Access_typed<ors::Transformation> *frame;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  tf::TransformListener listener;
  SubscriberConv(const char* topic_name, Access_typed<var_type>& _access, Access_typed<ors::Transformation> *_frame=NULL)
    : access(_access), frame(_frame) {
    nh = new ros::NodeHandle;
    cout <<"subscibing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> ..." <<std::flush;
    sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
    cout <<"done" <<endl;
  }
  ~SubscriberConv(){
    nh->shutdown();
    delete nh;
  }
  void callback(const typename msg_type::ConstPtr& msg) {
    double time=conv_time2double(msg->header.stamp);
    access.set( time ) = conv(*msg);
    if(frame){
      frame->set( time ) = ros_getTransform("/base_link", msg->header.frame_id, listener);
    }
  }
};



//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

//template<class msg_type, class var_type, var_type conv(const msg_type&)>
//struct SubscriberConvThreaded : Thread {
//  Access_typed<var_type>& access;
//  Access_typed<ors::Transformation> *frame;
//  ros::NodeHandle *nh;
//  ros::Subscriber sub;
//  tf::TransformListener listener;
//  SubscriberConvThreaded(const char* topic_name, Access_typed<var_type>& _access, Access_typed<ors::Transformation> *_frame=NULL)
//    : Thread(STRING("Subscriber_"<<_access.name <<"->" <<_topic_name), .05),
//      access(_access), frame(_frame) {
//  }
//  ~SubscriberConvThreaded(){}
//  void open(){
//    nh = new ros::NodeHandle;
//    cout <<"subscibing to topic '" <<topic_name <<"' <" <<typeid(var_type).name() <<"> ..." <<std::flush;
//    sub = nh->subscribe(topic_name, 1, &SubscriberConv::callback, this);
//    cout <<"done" <<endl;
//  }
//  void close(){
//    delete nh;
//  }
//  void step(){}
//  void callback(const typename msg_type::ConstPtr& msg) {
//    double time=conv_time2double(msg->header.stamp);
//    access.set( time ) = conv(*msg);
//    if(frame){
//      frame->set( time ) = ros_getTransform("/base_link", msg->header.frame_id, listener);
//    }
//  }
//};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, var_type conv(const msg_type&)>
struct SubscriberConvNoHeader {
  Access_typed<var_type>& access;
  ros::NodeHandle nh;
  ros::Subscriber sub;
  SubscriberConvNoHeader(const char* topic_name, Access_typed<var_type>& _access)
    : access(_access) {
    sub = nh.subscribe( topic_name, 1, &SubscriberConvNoHeader::callback, this);
  }
  ~SubscriberConvNoHeader(){}
  void callback(const typename msg_type::ConstPtr& msg) {
    access.set() = conv(*msg);
  }
};


//===========================================================================
//
// subscribing a message into an MLR-type-Access via a conv_* function
//

template<class msg_type, class var_type, msg_type conv(const var_type&)>
struct PublisherConv : Module{
  Access_typed<var_type> access;
  ros::NodeHandle *nh;
  ros::Publisher pub;
  const char* topic_name;

  PublisherConv(const char* _topic_name, Access_typed<var_type>& _access)
      : Module(STRING("Publisher_"<<_access.name <<"->" <<_topic_name)),
        access(this, _access, true),
        topic_name(_topic_name){
//    listenTo(*access.var);
  }
  void open(){
    nh = new ros::NodeHandle;
    pub = nh->advertise<msg_type>(topic_name, 1);
  }
  void step(){
    pub.publish(conv(access.get()));
  }
  void close(){
    // nh->shutdown(); //why does this throw an error???
    delete nh;
  }
};

//===========================================================================
//
// maybe useful for expensive conversion: a subscriber module
//

//template<class msg_type>
//struct SubscriberModule : Module {
//  Access_typed<msg_type> access;
//  ros::NodeHandle* nh;
//  ros::Subscriber sub;
//  const char* topic_name;
//  Subscriber(const char* topic_name, const char* var_name, ModuleL& S=NoModuleL)
//    : Module(STRING("Subscriber_"<<topic_name <<"->" <<var_name), S, loopWithBeat, .01),
//      access(this, var_name),
//      topic_name(topic_name) {}
//  void open() {
//    nh = new ros::NodeHandle;
//    sub  = nh->subscribe( topic_name, 1, &Subscriber<msg_type>::callback, this);
//  }
//  void step() {}
//  void close() {
//    nh->shutdown();
//    delete nh;
//  }
//  void callback(const typename msg_type::ConstPtr& msg) { access.set() = *msg; }
//};

//===========================================================================
//
// OLD
//


//===========================================================================
//
// variable declarations
//


//-- a basic message type for communication with the soft hand controller
struct SoftHandMsg{
  mlr::String soft_hand_cmd;
  SoftHandMsg(){}
  SoftHandMsg(const mlr::String soft_hand_cmd)
    :soft_hand_cmd(soft_hand_cmd){}
};
//inline void operator<<(ostream& os, const CtrlMsg& m){ os<<"BLA"; }
//inline void operator>>(istream& os, CtrlMsg& m){  }


//===========================================================================
//
// modules
//
/* TODO: allow modules to set default loopWithBeat, listenFirst, etc
options. (In their constructor?)
*/
//===========================================================================
/// This module only calls ros:spinOnce() in step() and loops full speed -- to sync the process with the ros server
//BEGIN_MODULE(RosCom_Spinner)
//END_MODULE()

struct RosCom_Spinner:Module{
  RosCom_Spinner():Module("RosCom_Spinner", .001){}
  void open(){ rosCheckInit(); }
  void step(){ ros::spinOnce(); }
  void close(){}
};

//===========================================================================
/// This module syncs the controller state and refs with the real time hardware controller (marc_controller_...)
//struct RosCom_ControllerSync:Module{
//  struct sRosCom_ControllerSync *s;
//  ACCESSnew(CtrlMsg, ctrl_ref)
//  ACCESSnew(CtrlMsg, ctrl_obs)
//  RosCom_ControllerSync():Module("RosCom_ControllerSync", listenFirst){}
//  void open();
//  void step();
//  void close();
//};

// Helper function so sync ors with the real PR2
/**
 * This starts the initial sync of the world with ctrl_obs from the robot.
 *
 * This is verbose (helps debugging) and retries to connect to the robot multiple times.
 *
 * If useRos==false then nothing happens.
 */
void initialSyncJointStateWithROS(ors::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

/**
 * Sync the world with ctrl_obs from the robot.
 *
 * If useRos==false then nothing happens.
 */
void syncJointStateWitROS(ors::KinematicWorld& world, Access_typed<CtrlMsg>& ctrl_obs, bool useRos);

//===========================================================================

struct PerceptionObjects2Ors : Module{
  Access_typed<visualization_msgs::MarkerArray> perceptionObjects;
  Access_typed<ors::KinematicWorld> modelWorld;
  PerceptionObjects2Ors()
    : Module("PerceptionObjects2Ors"),
    perceptionObjects(this, "perceptionObjects", true),
    modelWorld(this, "modelWorld"){}
  void open(){}
  void step();
  void close(){}
};
