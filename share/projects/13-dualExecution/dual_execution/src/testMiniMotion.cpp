//#define USE_SL

#include <ros/ros.h>

#ifdef USE_SL
#include <sl_controller_interface/controller_interface.h>
#include <sl_controller_interface/joint_trajectory_client.h>
#include <sl_controller_interface/switch_controller_stack_client.h>
//#include <boost/thread.hpp>
#endif

#include <Motion/motion.h>
#include <Motion/taskMap_default.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>

arr getSimpleTrajectory(ors::Graph& G){
  MotionProblem P(&G, NULL, false);
  P.loadTransitionParameters();

  //-- setup the motion problem
  TaskCost *c;
  c = P.addTaskMap("position",
                   new DefaultTaskMap(posTMT, G, "endeff", NoVector));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly,
                          ARRAY(P.ors->getShapeByName("miniTarget")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  MotionProblemFunction MF(P);
  arr x = P.getInitialization();

  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, useAdaptiveDamping=false, damping=1e-3, maxStep=.5));
  P.costReport();
  return x;
}



void testMarcs(){
  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  arr x = getSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);

  for(uint i=0;i<1;i++)
    displayTrajectory(x, 1, G, gl,"planned trajectory");
}

#ifdef USE_SL
void testLudos(){
  std::string robot_part("LEFT_ARM");
  std::string stack_name("LeftArmJointInverseDynamicsControl");
  std::string controller_name("LeftArmJointTrajectoryGenerator");

  ros::init(MT::argc, MT::argv, "TestJointTrajectoryGenerator");

  ros::AsyncSpinner spinner(4);
  spinner.start();

  sl_controller_interface::init();

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

  ros::Duration(2.0).sleep();

  ROS_INFO("moving to pos 0");
  joint_client.moveTo(desired_joint_positions);

  desired_joint_positions[0] = 0.5;
  desired_joint_positions[3] = 1.5;

  ROS_INFO("moving to pos 1");
  joint_client.moveTo(desired_joint_positions);

  ROS_INFO("moving to pos 2");
  desired_joint_positions.clear();
  desired_joint_positions.resize(joint_names.size(), 0.0);
  joint_client.moveTo(desired_joint_positions,2.0);
}
#endif

int main(int argc, char** argv){

  MT::initCmdLine(argc,argv);
  //testMarcs();
  //testLudos();


  OpenGL gl;
  ors::Graph G;
  init(G, gl, MT::getParameter<MT::String>("orsFile"));

  arr x = getSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);

  for(uint i=0;i<1;i++)
    displayTrajectory(x, 1, G, gl,"planned trajectory");

  std::string robot_part("RIGHT_ARM");
  std::string stack_name("RightArmJointInverseDynamicsControl");
  std::string controller_name("RightArmJointTrajectoryGenerator");

  ros::init(argc, argv, "TestJointTrajectoryGenerator");

  ros::AsyncSpinner spinner(4);
  spinner.start();

#if USE_SL
  sl_controller_interface::init();

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

  CHECK(joint_names.size()==7, "");

  ros::Duration(1.0).sleep();


  for(uint i=0;i<7;i++) desired_joint_positions[i] = 0.0;
  desired_joint_positions[1] = -1.0;
  desired_joint_positions[3] = 1.5;
  joint_client.moveTo(desired_joint_positions, 10.);

  ROS_INFO("moving to x[0]");
  for(uint i=0;i<7;i++) desired_joint_positions[i] = x(0,i);
  joint_client.moveTo(desired_joint_positions, 10.);

  for(uint t=0;t<x.d0;t++){
    for(uint i=0;i<7;i++) desired_joint_positions[i] = x(t,i);
    joint_client.moveTo(desired_joint_positions, 0.01);
  }
#endif

  return 0;
}
