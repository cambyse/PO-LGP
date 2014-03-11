#ifndef AMEX_CONTROLLER_h
#define AMEX_CONTROLLER_h

#include <ros/ros.h>
#include <ros/service_client.h>
#include <Motion/mobject.h>
#include <Motion/adaptiveMotionExecution.h>
#include <tree_controller_pkg/SetJointGains.h>
#include <tree_controller_pkg/SetPosTarget.h>
#include <tree_controller_pkg/SetVecTarget.h>
#include <tree_controller_pkg/GetJointState.h>
#include <tree_controller_pkg/SetTaskGains.h>
#include <tree_controller_pkg/SetNaturalGains.h>
#include <goal_publisher/GetGoal.h>
#include <Ors/ors.h>


struct AmexController {
  uint NUM_JOINTS;                // joint dimension

  ros::NodeHandle nh;
  ros::Timer timerRunAmex;

  ros::ServiceClient setPosTargetClient;
  ros::ServiceClient setVecTargetClient;
  ros::ServiceClient getJointStateClient;
  ros::ServiceClient setJointGainsClient;
  ros::ServiceClient getGoalClient;
  ros::ServiceClient setTaskGainsClient;
  ros::ServiceClient setNaturalGainsClient;
  goal_publisher::GetGoal getGoalSrv;
  tree_controller_pkg::SetPosTarget setPosTargetSrv;
  tree_controller_pkg::SetVecTarget setVecTargetSrv;
  tree_controller_pkg::GetJointState getJointStateSrv;
  tree_controller_pkg::SetJointGains setJointGainsSrv;
  tree_controller_pkg::SetTaskGains setTaskGainsSrv;
  tree_controller_pkg::SetNaturalGains setNaturalGainsSrv;

  arr q, qd;                      // Joint position, velocity
  arr state;                      // Task position
  arr refPlan;                    // Reference trajectory
  double TRef;                    // Final time of reference trajectory
  arr q0;                         // Initial position in joint space
  arr x0;                         // Initial position in task space
  double dtAmex;                  // Time step between sending trajectories
  arr refFrame;                   // Reference Coordinate Frame
  bool useGoalPub;                // Use Goal from /get_goal topic

  ors::KinematicWorld world;      //
  AdaptiveMotionExecution* amex;  // Motion adaptation method
  MObject* goalMO;                // Target goal state
  arr acc_gains, vel_gains;       // PD Gains for joints
  arr i_gains, i_claim;

  // BOOKKEEPING VARS
  String folder;
  arr q_bk;                       // joint angles at each time step
  arr x_bk;                       // task variables at each time step
  arr goal_bk;                    // goal position at each time step
  arr ct_bk;                      // computational time at each time step
  arr s_bk;                       // phase variable at each time step

  AmexController(ros::NodeHandle &_nh, ors::KinematicWorld &_world, arr &_refPlan, arr &_q0, double _TRef, bool _useGoalPub);
  void initRosServices();
  void initController();
  void iterate();
  void startController();

  void runAmex(double dtReal);
};
#endif
