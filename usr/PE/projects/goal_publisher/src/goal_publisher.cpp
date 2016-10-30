#include <ros/ros.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <goal_publisher/GetGoal.h>

class GoalTracker{
private:
  double x,y,z;
  double q_x,q_y,q_z,q_w;
  double filter;
  uint id;
  ros::ServiceServer getGoalSrv_;
public:

  GoalTracker(ros::NodeHandle &nh) {
    ar_track_alvar::AlvarMarkersConstPtr msg = ros::topic::waitForMessage<ar_track_alvar::AlvarMarkers>("/ar_pose_marker",ros::Duration(3.));

    while (msg->markers.size() < 1) {
      msg = ros::topic::waitForMessage<ar_track_alvar::AlvarMarkers>("/ar_pose_marker",ros::Duration(3.));
    }
    x = msg->markers.at(0).pose.pose.position.x;
    y = msg->markers.at(0).pose.pose.position.y;
    z = msg->markers.at(0).pose.pose.position.z;

    filter = 0.1;
    id = msg->markers.at(0).id;
    getGoalSrv_ = nh.advertiseService("get_goal", &GoalTracker::getGoal, this);

  }

  void syncGoal(const ar_track_alvar::AlvarMarkers::ConstPtr& msg) {

    for (uint i = 0;i<msg->markers.size(); i++) {
      ROS_INFO("Marker ID: %d",msg->markers.at(i).id);
      std::cout<<msg->markers.at(i).id<<std::endl;
      if (msg->markers.at(i).id == id) {
        x = filter*x + (1-filter)*msg->markers.at(i).pose.pose.position.x;
        y = filter*y + (1-filter)*msg->markers.at(i).pose.pose.position.y;
        z = filter*z + (1-filter)*msg->markers.at(i).pose.pose.position.z;

        q_x = msg->markers.at(i).pose.pose.orientation.x;
        q_y = msg->markers.at(i).pose.pose.orientation.y;
        q_z = msg->markers.at(i).pose.pose.orientation.z;
        q_w = msg->markers.at(i).pose.pose.orientation.w;


        ROS_INFO("x: %f",x);
        ROS_INFO("y: %f",y);
        ROS_INFO("z: %f",z);
        ROS_INFO("q_x: %f",q_x);
        ROS_INFO("q_y: %f",q_y);
        ROS_INFO("q_z: %f",q_z);
        ROS_INFO("q_z: %f",q_w);
      }
    }
  }

  bool getGoal(goal_publisher::GetGoal::Request& req,  goal_publisher::GetGoal::Response& resp) {
    resp.x = x;
    resp.y = y;
    resp.z = z;
    resp.q_x = q_x;
    resp.q_y = q_y;
    resp.q_z = q_z;
    resp.q_w = q_w;
    return true;
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_publisher");
  ros::NodeHandle nh;
  GoalTracker goalTracker(nh);
  ros::Subscriber sub = nh.subscribe<ar_track_alvar::AlvarMarkers>("/ar_pose_marker",1,&GoalTracker::syncGoal,&goalTracker);
  ros::spin();
  return 0;
}
