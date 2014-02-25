#ifndef AMEX_CONTROLLER_h
#define AMEX_CONTROLLER_h

#include <ros/ros.h>
#include <ros/service_client.h>
#include <Motion/feedbackControl.h>
#include <Motion/mobject.h>
#include <Motion/adaptiveMotionExecution.h>
#include <tree_controller_pkg/SetJointGains.h>
#include <tree_controller_pkg/SetPosTarget.h>
#include <tree_controller_pkg/SetVecTarget.h>


struct AmexController {
private:
  ros::NodeHandle nh;
  MObject* goalMO;
  FeedbackMotionControl *FMC;
  ros::Timer timerRunAmex;

  ros::ServiceClient setPosTargetClient;
  ros::ServiceClient setVecTargetClient;
  ros::ServiceClient getJointStateClient;
  ros::ServiceClient setJointGainsClient;


  arr p_gains, d_gains;

  arr q, qd, qdd;             // Joint position, velocity, acceleration
  arr state;                  // Task position
  arr tauRef;                 // Reference trajectory
  double TRef;                // Final time of reference trajectory
  arr q0;                     // Initial position joint space
  arr x0;                     // Initial position task space
  double dtAmex;              // Time step between sending trajectories
  double regularization ;     // Regularization parameter for PD control
  arr refFrame;               // Reference Coordinate Frame
  bool useGoalPub;            // Use Goal from /get_goal topic

  // BOOKKEEPING VARS
  String folder;

  arr q_bk;    // joint angles at each time step
  arr x_bk;    // task variables at each time step
  arr goal_bk; // goal position at each time step
  arr ct_bk;   // computational time at each time step
  arr s_bk;    // phase variable at each time step

  ors::KinematicWorld world;
  AdaptiveMotionExecution* pfc;
  PDtask *taskPos, *taskVec, *taskHome, *taskCol;

public:

  AmexController(ros::NodeHandle &_nh);
  void iterate();

};
#endif
