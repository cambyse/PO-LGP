#include <ros/ros.h>
#include <ros_msg/Literal.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hello_world_node");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<RelationalMachine::Literal>("/RelationalMachine/ActionCommand", 1000);

  sleep(1); //IMPORTANT :-(

  RelationalMachine::Literal actionCommand;
  actionCommand.literal={"hello", "world"};
  actionCommand.parameters="x=5";

  ROS_INFO("%s", actionCommand.parameters.c_str());

  pub.publish(actionCommand);


  ros::spin();
  ros::shutdown();

  return 0;
}
