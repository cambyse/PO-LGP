#include <ros/ros.h>
#include <ros_msg/Literal.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hello_world_node");

  ros::NodeHandle nh("ThirdHand");

  ros::Publisher pub = nh.advertise<RelationalMachine::Literal>("ActionCommand", 10, true);

  RelationalMachine::Literal actionCommand;
  actionCommand.literal={"hello", "world"};
  actionCommand.parameters="x=5";

  ROS_INFO("%s", actionCommand.parameters.c_str());

  pub.publish(actionCommand);

  ros::Rate loop_rate(100);
  for(uint i=0;i<10;i++){
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
