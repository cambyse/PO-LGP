#include <ros/ros.h>
#include <RelationalMachine/ActionCommandAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<RelationalMachine::ActionCommandAction> Server;

void execute(const RelationalMachine::ActionCommandGoalConstPtr &goal, Server *as) {
  // Perform action
  std::cout << "Performing Action:";
  for(const auto &lit: goal->literal)
    std::cout << " " << lit;
  std::cout << std::endl;
  as->setSucceeded();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ThirdHandActionServer");
  ros::NodeHandle n;
  Server server(n, "topic", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
