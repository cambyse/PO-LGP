#include "rosutil.h"


bool rosOk(){
  return ros::ok();
}

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


ors::Transformation cvrt_pose2transformation(const tf::Transform &trans){
  ors::Transformation X;
  X.setZero();
  tf::Quaternion q = trans.getRotation();
  tf::Vector3 t = trans.getOrigin();
  X.rot.set(q.w(), q.x(), q.y(), q.z());
  X.pos.set(t.x(), t.y(), t.z());
  return X;
}

ors::Transformation cvrt_pose2transformation(const geometry_msgs::Pose &pose){
  ors::Transformation X;
  X.setZero();
  auto& q = pose.orientation;
  auto& t = pose.position;
  X.rot.set(q.w, q.x, q.y, q.z);
  X.pos.set(t.x, t.y, t.z);
  return X;
}

void cvrt_pose2transXYPhi(arr& q, uint qIndex, const geometry_msgs::PoseWithCovarianceStamped& pose){
  auto& _quat=pose.pose.pose.orientation;
  auto& _pos=pose.pose.pose.position;
  ors::Quaternion quat(_quat.w, _quat.x, _quat.y, _quat.z);
  ors::Vector pos(_pos.x, _pos.y, _pos.z);

  double angle;
  ors::Vector rotvec;
  quat.getRad(angle, rotvec);
  q(qIndex+0) = pos(0);
  q(qIndex+1) = pos(1);
  q(qIndex+2) = MT::sign(rotvec(2)) * angle;
}

timespec cvrt_time2timespec(const ros::Time& time){
  return {time.sec, time.nsec};
}

double cvrt_time2double(const ros::Time& time){
  return (double)(time.sec) + 1e-9d*(double)(time.nsec);
}

ors::Transformation ros_getTransform(const std::string& from, const std::string& to, tf::TransformListener& listener){
  ors::Transformation X;
  X.setZero();
  try{
    tf::StampedTransform transform;
    listener.lookupTransform(from, to, ros::Time(0), transform);
    X = cvrt_pose2transformation(transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return X;
}


arr conv_points2arr(const std::vector<geometry_msgs::Point>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].x;
    P(i,1) = pts[i].y;
    P(i,2) = pts[i].z;
  }
  return P;
}

arr conv_colors2arr(const std::vector<std_msgs::ColorRGBA>& pts){
  uint n=pts.size();
  arr P(n,3);
  for(uint i=0;i<n;i++){
    P(i,0) = pts[i].r;
    P(i,1) = pts[i].g;
    P(i,2) = pts[i].b;
  }
  return P;
}


std::vector<geometry_msgs::Point> conv_arr2points(const arr& pts){
  uint n=pts.d0;
  std::vector<geometry_msgs::Point> P(n);
  for(uint i=0;i<n;i++){
    P[i].x = pts(i, 0);
    P[i].y = pts(i, 1);
    P[i].z = pts(i, 2);
  }
  return P;
}


