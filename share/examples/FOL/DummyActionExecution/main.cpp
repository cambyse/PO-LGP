#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pr2/roscom.h>


struct DummyActionExecutionNode {

  ros::NodeHandle nh;
  ros::Subscriber sub_state;
  ros::Subscriber sub_symbols;

  DummyActionExecutionNode(){
    sub_state   = nh.subscribe("/RelationalMachine/state", 10, &DummyActionExecutionNode::cb_newState, this);
    sub_symbols = nh.subscribe("/RelationalMachine/symbols", 10, &DummyActionExecutionNode::cb_newSymbols, this);
  }

  void cb_newState(const std_msgs::String::ConstPtr& msg);       ///< just for info
  void cb_newSymbols(const std_msgs::String::ConstPtr& msg);     ///< just for info
};

void DummyActionExecutionNode::cb_newState(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new state: %s", msg->data.c_str());
}

void DummyActionExecutionNode::cb_newSymbols(const std_msgs::String::ConstPtr &msg) {
  ROS_INFO("Received new symbols: %s", msg->data.c_str());
}

//===========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "DummyActionExecutionNode");

  RosCom_Spinner spinner;

  ros::NodeHandle nh;
  ros::Publisher pub_effect = nh.advertise<std_msgs::String>("/RelationalMachine/effect", 1);

  threadOpenModules(true);

  mlr::wait(1);

  pub_effect.publish(conv_string2string("(alignHand conv) (positionHand conv)"));
//  effects.set() = "(alignHand conv) (positionHand conv)";
//  mlr::wait(1);
//  effects.set() = "(lowerHand conv)";
  //    case 3: effect.data = "(controlForce timeout)";  break;
  //    case 4: effect.data = "(homing conv)";  break;
  //    case 5: effect.data = "(undefined)";  break;

  threadCloseModules();
  cout <<"BYE BYE" <<endl;

  return 0;
}
