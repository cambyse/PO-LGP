#include <ros/ros.h>
#include <std_msgs/String.h>


struct DummyActionExecutionNode {
  ros::NodeHandle nh;
  ros::Subscriber sub_state;
  ros::Subscriber sub_command;
  ros::Subscriber sub_symbols;
  ros::Publisher pub_effect;

  DummyActionExecutionNode(): nh("ThirdHand/RelationalMachine") {
    sub_state   = nh.subscribe("RelationalState", 10, &DummyActionExecutionNode::cb_newState, this);
    sub_command = nh.subscribe("RelationalCommand", 10, &DummyActionExecutionNode::cp_processCommand, this);
    sub_symbols = nh.subscribe("RelationalSymbols", 10, &DummyActionExecutionNode::cb_newSymbols, this);
    pub_effect  = nh.advertise<std_msgs::String>("RelationalEffect", 10, true);
    sleep(1);
  }

  void cp_processCommand(const std_msgs::String::ConstPtr& msg); ///< this is the most important routine -- listening to commands
  void cb_newState(const std_msgs::String::ConstPtr& msg);       ///< just for info
  void cb_newSymbols(const std_msgs::String::ConstPtr& msg);     ///< just for info
};

void DummyActionExecutionNode::cp_processCommand(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new command: %s", msg->data.c_str());

  // here you would trigger a true activity -- we just wait
  sleep(1);

  // at some time the activity has an effect -- you send this back to the RelationalMachine (usually not from this callback)
  static int i=0;
  std_msgs::String effect;
  i++;
  switch(i){
    case 1: effect.data = "(alignHand conv) (positionHand conv)";  break;
    case 2: effect.data = "(lowerHand conv)";  break;
    case 3: effect.data = "(controlForce timeout)";  break;
    case 4: effect.data = "(homing conv)";  break;
    case 5: effect.data = "(undefined)";  break;
  }
  ROS_INFO("Sending new effect: %s", effect.data.c_str());
  pub_effect.publish(effect);

}

void DummyActionExecutionNode::cb_newState(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new state: %s", msg->data.c_str());
}

void DummyActionExecutionNode::cb_newSymbols(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new symbols: %s", msg->data.c_str());
}

//===========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "DummyActionExecutionNode");

  DummyActionExecutionNode AEnode;
  ros::spin();

  return 0;
}
