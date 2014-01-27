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

void dualExecution(const arr& x, const arr& y, const arr& dual, ors::KinematicWorld& world){
  arr v,a;
  getVel(v, x, .1);
  getAcc(a, x, .1);


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

  Metronome tic("myticcer",.1);
  ROS_INFO("sending trajectory");
  for(uint t=0;t<x.d0;t++){

    //-- get cartesian pose
    pose.position.x = y(t,0);
    pose.position.y = y(t,1);
    pose.position.z = y(t,2);

    //-- get force
    std::vector<double> force;
    force.resize(6,0.0);
    if(dual(t)>0.){ //steer towards constraint
      force[2] = 1.;
    }

    //move and send force at the same time
    tf::Transform pose_tf;
    tf::poseMsgToTF(pose, pose_tf);
    cart_client.moveToWithForce(pose_tf, force, 0.1, false);

    ros::Duration(0.1).sleep();
  }
}


