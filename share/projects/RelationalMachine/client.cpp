#include <ros/ros.h>
#include <RelationalMachine/ActionCommandAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<RelationalMachine::ActionCommandAction> Client;

int main(int argc, char** argv) {
  ros::init(argc, argv, "ThirdHandActionClient");

  Client client("topic", true);
  client.waitForServer();

  RelationalMachine::ActionCommandGoal goal;
  goal.literal.push_back("hello");
  goal.literal.push_back("world");
  // goal.literal={"hello", "world"};
  goal.parameters="x=5";

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  return 0;
}
