#include <Core/util.h>
#include <RosCom/roscom.h>
#include <RosCom/spinner.h>
#include <baxter_core_msgs/JointCommand.h>

// =================================================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  RosInit rosinit("my_node");
  RosCom_Spinner spinner;
  threadOpenModules(true);

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<baxter_core_msgs::JointCommand>(
                                       "robot/limb/left/joint_command", 1);

  baxter_core_msgs::JointCommand msg;
  msg.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  msg.command = { 0, 0, 0, 0, 0, 0, 0 };
  msg.names = { "left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2" };

  for ( uint i = 0; i < 1000; i++ )
  {
    pub.publish(msg);
    mlr::wait(0.05);
  }

  threadCloseModules();
  std::cout << "Bye bye" << std::endl;

  return 0;
}
