#include <ros/ros.h>
#include <ros_msg/Literal.h>
#include <std_msgs/String.h>
#include <Core/util.h>

struct DummyServer {
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;

  DummyServer() {
    sub = n.subscribe("/RelationalMachine/ActionCommand", 10, callback);
    pub = n.advertise<RelationalMachine::Literal>("/RelationalMachine/ActionResponse", 10);
  }

  void callback(const RelationalMachine::Literal::ConstPtr &msg) {
    String literal;
    literal << "Received Action Command:";
    for(const auto &lit: msg->literal)
      literal << " " << lit;

    ROS_INFO("%s", (const char *)literal);
  }

  void run() {
    ros::spin();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "DummyServer");

  DummyServer ds;
  ds.run();

  return 0;
}
