#include <ros/ros.h>
#include <Core/module.h>
#include <std_msgs/Float32MultiArray.h>
#include <Hardware/G4/G4.h>
#include <Hardware/G4/module_G4Display.h>
#include <RosCom/roscom.h>


int main(int argc, char **argv)
{
  mlr::initCmdLine(argc, argv);
  ros::init(argc, argv, "g4_data_publisher", ros::init_options::NoSigintHandler);

  Access_typed<floatA> dataAccess(NULL, "g4_poses");

  G4Poller g4poller;
  G4Display g4disp;
  PublisherConv<std_msgs::Float32MultiArray, floatA, &conv_floatA2Float32Array>  pub_floats("/g4_data", dataAccess);

  threadOpenModules(true);
  moduleShutdown().waitForValueGreaterThan(0);
  threadCloseModules();
  return 0;
}
