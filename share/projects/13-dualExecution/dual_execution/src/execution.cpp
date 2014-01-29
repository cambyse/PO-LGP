#include <sl_controller_interface/controller_interface.h>
#include <sl_controller_interface/joint_trajectory_client.h>
#include <sl_controller_interface/switch_controller_stack_client.h>
#include <sl_controller_interface/data_collector_client.h>
#include <sl_controller_interface/cartesian_trajectory_client.h>
#include <sl_controller_interface/controller_interface.h>
#include <sl_controller_interface/robot_info.h>
#include <sl_controller_interface/robot_monitor.h>

#include <sl_controller_interface/gain_client.h>
#include <sl_controller_interface/joint_trajectory_client.h>
#include <sl_controller_interface/cartesian_trajectory_client.h>
#include <sl_controller_interface/switch_controller_stack_client.h>
#include <sl_barrett_controller_interface/force_torque_sensor_client.h>


#include <Motion/motion.h>
#include <Core/thread.h>

#include "execution.h"

void dualExecution(const arr& x, const arr& y, const arr& ori, const arr &dual, ors::KinematicWorld &world, double tau){
#if 0
  std::string end_effector_name("R_HAND");
  std::string stack_name_cart("RightArmCartesianControl");
  std::string controller_name_cart("RightArmCartesianTrajectoryGenerator");

  int end_effector_id = sl_controller_interface::RobotInfo::getEndeffectorId(end_effector_name);
  ROS_INFO("Endeff ID %d", end_effector_id);

  std::string end_eff_name = sl_controller_interface::RobotInfo::getEndeffectorLink(end_effector_id);
  ROS_INFO("Endeff Name %s", end_eff_name.c_str());

  sl_controller_interface::RobotMonitor robot_monitor;

  sl_controller_interface::SwitchControllerStackClient scs;
  ROS_INFO("Switching to %s", stack_name_cart.c_str());
  scs.switchControllerStack(stack_name_cart);

  // to turn on an off the cartesian force and position control gains
  sl_controller_interface::GainClient gain_client(end_effector_id);
  // to calibrate the force torque sensor
  sl_controller_interface::ForceTorqueSensorClient ft_client(end_effector_id);
  // this waiting has to be here so that the above clients all get properly initialised
  ros::Duration(2.0).sleep();

  // use known weight of hand to zero the F/T measurements
  ft_client.calibrate();

  // switch to the Cartesian Force Control task ////////////////////////////
  sl_controller_interface::CartesianTrajectoryClient cart_client(controller_name_cart,end_effector_id);
  ROS_INFO("Switching to %s", stack_name_cart.c_str());
  scs.switchControllerStack(stack_name_cart);

  // enable the force and position gains by fading them in
   if(!gain_client.enablePositionForceControlGains())
    ROS_ERROR("Couldn't enable position and force control gains\n");

  // enable the force and position gains for selective dimensions
  // 'p' in the string stands for using position control in this cartesian dimension
  // 'f' in the string stands for using position control in this cartesian dimension
//  if(!gain_client.setArmPositionForceControlGains("ppfppp"))
//    ROS_ERROR("Couldn't enable position and force control gains\n");

  std::vector<double> force;
  force.resize(6,0.0);
  force[2] = 1.0;

  // get the current endeffector pose
  geometry_msgs::Pose pose;
  robot_monitor.getEndeffPose(pose, end_effector_id);
  std::cout << "Endeffector pose " << pose << std::endl;

  for(uint t=0;t<x.d0;t++){
    //-- get cartesian pose
    pose.position.x = y(t,0);
    pose.position.y = y(t,1);
    pose.position.z = y(t,2);
    pose.orientation.w = ori(t,0);
    pose.orientation.x = ori(t,1);
    pose.orientation.y = ori(t,2);
    pose.orientation.z = ori(t,3);

    tf::Transform pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    cart_client.moveTo(pose_tf, tau, true);
//    cart_client.moveToWithForce(pose_tf, force, tau, true);
    ros::Duration(0.05).sleep();
  }

return;
#else

  arr y_vel, y_acc, ori_vel, ori_acc;
  getVel(y_vel, y, tau);
  getAcc(y_acc, y, tau);
  getVel(ori_vel, ori, tau);
  getAcc(ori_acc, ori, tau);
  write(LIST<arr>(y, ori, y_vel, ori_vel, y_acc, ori_acc), "trajectory.dat");


  std::string end_effector_name("R_HAND");
  std::string stack_name_cart("RightArmCartesianControl");
  std::string controller_name_cart("RightArmCartesianTrajectoryGenerator");


  int end_effector_id = sl_controller_interface::RobotInfo::getEndeffectorId(end_effector_name);
  ROS_INFO("Endeff ID %d", end_effector_id);

  std::string end_eff_name = sl_controller_interface::RobotInfo::getEndeffectorLink(end_effector_id);
  ROS_INFO("Endeff Name %s", end_eff_name.c_str());



  // to turn on an off the cartesian force and position control gains
  sl_controller_interface::GainClient gain_client(end_effector_id);
  // to calibrate the force torque sensor
  sl_controller_interface::ForceTorqueSensorClient ft_client(end_effector_id);
  // this waiting has to be here so that the above clients all get properly initialised
  sl_controller_interface::RobotMonitor robot_monitor;
  ros::Duration(2.0).sleep();

  // use known weight of hand to zero the F/T measurements
  ft_client.calibrate();

  // switch to the Cartesian Force Control task ////////////////////////////
  sl_controller_interface::CartesianTrajectoryClient cart_client(controller_name_cart,end_effector_id);
  sl_controller_interface::SwitchControllerStackClient scs;
  ROS_INFO("Switching to %s", stack_name_cart.c_str());
  scs.switchControllerStack(stack_name_cart);


  if(!gain_client.setArmPositionForceControlGains("ppfppp"))
    ROS_ERROR("Couldn't enable position and force control gains\n");

  geometry_msgs::Pose pose;
  robot_monitor.getEndeffPose(pose, end_effector_id);
  std::cout << "Endeffector pose " << pose << std::endl;


#if 0
  Metronome tic("myticcer",tau);
  ROS_INFO("sending trajectory");
  for(uint t=0;t<x.d0;t++){

    //-- get cartesian pose
    pose.position.x = y(t,0);
    pose.position.y = y(t,1);
    pose.position.z = y(t,2);
    pose.orientation.w = ori(t,0);
    pose.orientation.x = ori(t,1);
    pose.orientation.y = ori(t,2);
    pose.orientation.z = ori(t,3);

    //-- get cartesian vels and accs
    std::vector<double> vel(7),acc(7);
    for(uint i=0;i<3;i++) vel[i] = y_vel(t,i);
    for(uint i=0;i<4;i++) vel[3+i] = ori_vel(t,i);
    for(uint i=0;i<3;i++) acc[i] = y_acc(t,i);
    for(uint i=0;i<4;i++) acc[3+i] = ori_acc(t,i);

    //-- get force
    std::vector<double> force;
    force.resize(6,0.0);
    if(dual(t)>0.){ //steer towards constraint
      force[2] = 1.;
    }

    //move and send force at the same time
    tf::Transform pose_tf;
    tf::poseMsgToTF(pose, pose_tf);

    cout <<"TIME = " <<MT::realTime() <<endl;
//    tic.waitForTic();
//    cart_client.moveToWithForce(pose_tf, force, tau, true);
    cart_client.moveToWithForce(pose_tf, vel, acc, force, tau, false);

    ros::Duration(0.05).sleep();
  }
#else
  sl_controller_msgs::Trajectory traj;
  for(uint t=0; t<x.d0; t++) {

    traj.preempt = false;
    traj.dimension_names = cart_client.getDimensionNames();
//    traj.dimension_names.resize(3);

    std::vector<std::string>::iterator it = traj.dimension_names.end();
    traj.dimension_names.insert(it, cart_client.getForceDimensionNames().begin(), cart_client.getForceDimensionNames().end());


    sl_controller_msgs::TrajectoryPoint point;
    point.time_from_start = ros::Duration(tau*t);

    double force=0.;
    if(dual(t)>0.) force = 10.;

    std::vector<double> zeros(3);
    point.positions.push_back(y(t,0));
    point.positions.push_back(y(t,1));
    point.positions.push_back(y(t,2));
    point.positions.push_back(ori(t,0));
    point.positions.push_back(ori(t,1));
    point.positions.push_back(ori(t,2));
    point.positions.push_back(ori(t,3));
    point.positions.push_back(0.);
    point.positions.push_back(0.);
    point.positions.push_back(force);
    point.positions.insert(point.positions.end(), zeros.begin(), zeros.end());

    zeros.resize(6);
    point.velocities.push_back(y_vel(t,0));
    point.velocities.push_back(y_vel(t,1));
    point.velocities.push_back(y_vel(t,2));
    point.velocities.push_back(ori_vel(t,0));
    point.velocities.push_back(ori_vel(t,1));
    point.velocities.push_back(ori_vel(t,2));
    point.velocities.push_back(ori_vel(t,3));
    point.velocities.insert(point.velocities.end(), zeros.begin(), zeros.end());

    point.accelerations.push_back(y_acc(t,0));
    point.accelerations.push_back(y_acc(t,1));
    point.accelerations.push_back(y_acc(t,2));
    point.accelerations.push_back(ori_acc(t,0));
    point.accelerations.push_back(ori_acc(t,1));
    point.accelerations.push_back(ori_acc(t,2));
    point.accelerations.push_back(ori_acc(t,3));
    point.accelerations.insert(point.accelerations.end(), zeros.begin(), zeros.end());

    traj.points.push_back(point);
    traj.poke = false;
    traj.keep_trajectory_goal = true;
  }

  ROS_INFO("sending FULL trajectory");
  cart_client.sendCommand(traj, true);

#endif
#endif
}


