#define USE_SL

#include <ros/ros.h>

#ifdef USE_SL
#include <sl_controller_interface/controller_interface.h>
#include <sl_controller_interface/joint_trajectory_client.h>
#include <sl_controller_interface/switch_controller_stack_client.h>
#include <sl_controller_interface/data_collector_client.h>
//#include <boost/thread.hpp>
#endif

#include <KOMO/komo.h>
#include <Kin/taskMaps.h>
#include <Gui/opengl.h>
#include <Optim/optimization.h>
#include <Core/thread.h>
#include "execution.h"

arr getSimpleTrajectory(mlr::KinematicWorld& G){
  MotionProblem P(G, false);
  P.loadTransitionParameters();

  //-- setup the motion problem
  Task *c;
  c = P.addTask("position",
                   new TaskMap_Default(posTMT, G, "endeff", NoVector));
  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getShapeByName("miniTarget")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, {0.,0.,0.}, 1e1);

  MotionProblemFunction MF(P);
  arr x = P.getInitialization();

  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=100, damping=1e-3, maxStep=.5));
  P.costReport();
  return x;
}

arr getKindOfSimpleTrajectory(mlr::KinematicWorld& G){
  MotionProblem P(G, false);
  P.loadTransitionParameters();
  arr x = P.getInitialization();

  //-- setup the motion problem
  Task *c;
  c = P.addTask("position",
                   new TaskMap_Default(posTMT, G, "endeff", NoVector));
  c->setCostSpecs(P.T, P.T,
                          conv_vec2arr(P.world.getShapeByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, {0.,0.,0.}, 1e1);

  //c = P.addTask("collisionConstraints", new CollisionConstraint());
  c = P.addTask("planeConstraint", new PlaneConstraint(G, "endeff", ARR(0,0,-1,.7)));

  MotionProblemFunction MF(P);
  Convert ConstrainedP(MF);
  LagrangianProblem LagrangianP(ConstrainedP);
  LagrangianP.mu = 10.;

  for(uint k=0;k<5;k++){
    optNewton(x, LagrangianP, OPT(verbose=2, stopIters=100, damping=1e-3, stopTolerance=1e-4, maxStep=.5));
//    optNewton(x, UCP, OPT(verbose=2, stopIters=100, damping=1e-3, maxStep=1.));
    P.costReport();
//    displayTrajectory(x, 1, G, gl,"planned trajectory");
    LagrangianP.augmentedLagrangian_LambdaUpdate(x, .9);
    P.dualMatrix = LagrangianP.lambda;
    LagrangianP.mu *= 2.;
  }
  P.costReport();
  return x;
}

#ifdef USE_SL
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
#endif

int main(int argc, char** argv){

  mlr::initCmdLine(argc,argv);
  //testMarcs();
  //testLudos();


  OpenGL gl;
  mlr::KinematicWorld G(mlr::getParameter<mlr::String>("orsFile"));

  arr x = getKindOfSimpleTrajectory(G);
  arr x2 = reverseTrajectory(x);
  x.append(x2);
  arr v,a;
  getVel(v, x, .1);
  getAcc(a, x, .1);


//  for(uint i=0;i<1;i++)
//    displayTrajectory(x, 1, G, gl,"planned trajectory");


  ros::init(argc, argv, "TestJointTrajectoryGenerator");

  ros::AsyncSpinner spinner(4);
  spinner.start();

#ifdef USE_SL
  sl_controller_interface::init();


  moveHand();


  std::string robot_part("RIGHT_ARM");
  std::string stack_name("RightArmJointInverseDynamicsControl");
  std::string controller_name("RightArmJointTrajectoryGenerator");

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

  CHECK_EQ(joint_names.size(),7, "");

  ros::Duration(1.0).sleep();

    //-- Jeannette's zero position
//  for(uint i=0;i<7;i++) desired_joint_positions[i] = 0.0;
//  desired_joint_positions[1] = -1.0;
//  desired_joint_positions[3] = 1.5;
//  joint_client.moveTo(desired_joint_positions, 10.);

  //-- our starting pose
  ROS_INFO("moving to x[0]");
  for(uint i=0;i<7;i++) desired_joint_positions[i] = x(0,i);
  joint_client.moveTo(desired_joint_positions, 3.);

  sl_controller_interface::DataCollectorClient data_collect;
  data_collect.collectData();

  Metronome tic("myticcer",.1);
  ROS_INFO("sending trajectory");
  for(uint t=0;t<x.d0;t++){
    std::vector<double> desired_joint_velocities(7);
    std::vector<double> desired_joint_accelerations(7);
    for(uint i=0;i<7;i++) desired_joint_positions[i] = x(t,i);
    for(uint i=0;i<7;i++) desired_joint_velocities[i] = v(t,i);
    for(uint i=0;i<7;i++) desired_joint_accelerations[i] = a(t,i);


    sl_controller_msgs::Trajectory traj;
    /*
    joint_client.moveTo(desired_joint_positions,
        desired_joint_velocities,
        desired_joint_accelerations,
        0.095,
        true); // false);
        */
    traj.preempt = false;
    traj.dimension_names = joint_names;

    double time_step = 0.1;
    sl_controller_msgs::TrajectoryPoint point;
    point.time_from_start = ros::Duration(time_step);
    point.positions = desired_joint_positions;
    point.velocities = desired_joint_velocities;
    point.accelerations = desired_joint_accelerations;

    traj.points.push_back(point);
    traj.poke = false;
    traj.keep_trajectory_goal = true;

    joint_client.sendCommand(traj, false);

    /*
    // tic.waitForTic();
    cout <<"tic " <<t <<" time:" <<mlr::realTime() <<endl;
    joint_client.moveTo(desired_joint_positions, desired_joint_velocities, desired_joint_accelerations, 0.095, true); // false);
    //cout <<"tic " <<t <<" time:" <<mlr::realTime() <<endl;
    */
    // joint_client.moveTo(desired_joint_positions, 0.1);
  }
#endif

  return 0;
}
