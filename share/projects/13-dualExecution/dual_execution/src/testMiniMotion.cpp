#define USE_SL

#include <ros/ros.h>

#ifdef USE_SL
#include <sl_controller_interface/controller_interface.h>
#include <sl_controller_interface/joint_trajectory_client.h>
#include <sl_controller_interface/switch_controller_stack_client.h>
#include <sl_controller_interface/data_collector_client.h>
//#include <boost/thread.hpp>
#endif

#include <Motion/motion.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Core/thread.h>
#include "execution.h"
#include "getTraj.h"



void moveArm(const arr& x0){
  std::string robot_part("RIGHT_ARM");
  // arm position control
  std::string stack_name("RightArmJointInverseDynamicsControl");
  std::string controller_name("RightArmJointTrajectoryGenerator");

  // First, bring the left arm into a useful configuration ///////////////
  int part = sl_controller_interface::RobotInfo::getRobotPartId(robot_part);
  ROS_INFO("part id %d\n", part);
  // Bring up the Joint Controller Client
  sl_controller_interface::JointTrajectoryClient joint_client(controller_name,part);
  // Switch to the appropriate Joint Inverse Dynamics Controller
  sl_controller_interface::SwitchControllerStackClient scs;
  ROS_INFO("Switching to %s",stack_name.c_str());
  scs.switchControllerStack(stack_name);

  // Print the joint names that will be controlled
  std::vector<std::string> joint_names = joint_client.getJointNames();
  printf("joint names\n");
  for(int i=0; i<(int)joint_names.size(); ++i)
    printf("%s \n", joint_names[i].c_str());

  ros::Duration(2.0).sleep();

  std::vector<double> desired_joint_positions;
  desired_joint_positions.resize(joint_names.size(),0.0);
  CHECK_EQ(x0.N , joint_names.size(), "wrong #joints");

  std::vector<double> desired_velocities = desired_joint_positions;
  std::vector<double> desired_accelerations = desired_joint_positions;

  //get initial position
  //robot_monitor.getJointPositions(sl_controller_interface::RobotInfo::getRobotPartId(robot_part), joint_pos);
  // useful hard-coded joint configuration for left arm
  for(uint i=0;i<x0.N;i++) desired_joint_positions[i] = x0(i);

  joint_client.moveTo(desired_joint_positions, desired_velocities, desired_accelerations, 5.0, true);
}

void moveHand(){
    std::string robot_part("RIGHT_HAND");
    std::string stack_name("RightHandJointPDControl");
    std::string controller_name("RightHandJointTrajectoryGenerator");

    int part = sl_controller_interface::RobotInfo::getRobotPartId(robot_part);
    printf("part id %d\n", part);
    sl_controller_interface::JointTrajectoryClient joint_client(controller_name,part);

    sl_controller_interface::SwitchControllerStackClient scs;
    ROS_INFO("Switching to %s",stack_name.c_str());
    scs.switchControllerStack(stack_name);

    std::vector<std::string> joint_names = joint_client.getJointNames();
    printf("joint names\n");
    for(int i=0; i<(int)joint_names.size(); ++i)
      printf("%s \n", joint_names[i].c_str());

    std::vector<double> desired_joint_positions;
    desired_joint_positions.resize(joint_names.size(),0.0);

    ros::Duration(1.0).sleep();

    //-- our starting pose
    desired_joint_positions[0] = 0.;
    desired_joint_positions[1] = 2.;
    desired_joint_positions[2] = 1.;
    desired_joint_positions[3] = 2.;
    joint_client.moveTo(desired_joint_positions, 3.);
}



int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ors::KinematicWorld world(MT::getParameter<MT::String>("orsFile"));

  arr x, y, ori, dual;
  getTrajectory(x, y, ori, dual, world);

//  arr x2 = reverseTrajectory(x);
//  x.append(x2);
//  for(uint i=0;i<2;i++)
//    displayTrajectory(x, 1, world, "planned trajectory");

//  world.getBodyByName("table")->X.pos.z += .1;
//  world.setJointState(x[0]);

  ors::Quaternion rotZ;
  rotZ.setDeg(180,0,0,1);
  arr baseOrg={0, 0.2, 1.305};
  for(uint t=0;t<y.d0;t++){
    y[t]() -= baseOrg;
    y(t,0) *= -1.;
    y(t,1) *= -1.;

    ors::Quaternion q;
    q.set(&ori(t,0));
    q = rotZ*q;
    ori[t]() = ARRAY(q);
  }

  cout <<"initial cfg: " <<"q=" <<x[0] <<endl <<"y=" <<y[0] <<endl;

  //-- launch ROS
  ros::init(argc, argv, "TestJointTrajectoryGenerator");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  sl_controller_interface::init();


  moveArm(x[0]);
  moveHand();

  dualExecution(x, y, ori, dual, world, 0.1);

  return 0;
}




