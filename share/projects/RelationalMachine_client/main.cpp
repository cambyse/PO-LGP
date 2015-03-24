#include <ros/ros.h>
#include <std_msgs/String.h>

struct DummyServerNode {
  ros::NodeHandle nh;
  ros::Subscriber sub_state;
  ros::Subscriber sub_command;
  ros::Subscriber sub_symbols;
  ros::Publisher pub_effect;

  DummyServerNode() {
    sub_state = nh.subscribe("/RelationalMachine/RelationalState", 10, &DummyServerNode::cb_newState, this);
    sub_command = nh.subscribe("/RelationalMachine/RelationalCommand", 10, &DummyServerNode::cb_newCommand, this);
    sub_symbols = nh.subscribe("/RelationalMachine/RelationalSymbols", 10, &DummyServerNode::cb_newSymbols, this);
    pub_effect = nh.advertise<std_msgs::String>("/RelationalMachine/RelationalEffect", 10, true);
  }

  void cb_newState(const std_msgs::String::ConstPtr& msg);
  void cb_newCommand(const std_msgs::String::ConstPtr& msg);
  void cb_newSymbols(const std_msgs::String::ConstPtr& msg);
};

void DummyServerNode::cb_newState(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new state: %s", msg->data.c_str());
}

void DummyServerNode::cb_newCommand(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new command: %s", msg->data.c_str());
}

void DummyServerNode::cb_newSymbols(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new symbols: %s", msg->data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "DummyServerNode");

  DummyServerNode DSnode;
  ros::spin();

  return 0;
}
