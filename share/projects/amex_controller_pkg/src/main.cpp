#include <ros/ros.h>
#include <Core/array.h>
#include "amex_controller.h"
#include "traj_optimizer.h"
#include <goal_publisher/GetGoal.h>
#include <tree_controller_pkg/GetJointState.h>
#include <tree_controller_pkg/StartLogging.h>
#include <tree_controller_pkg/StopLogging.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>

int main(int argc, char** argv)
{
  /// Init ros
  ros::init(argc, argv, "amex_controller");
  ros::NodeHandle nh;

  /// Init ors
  MT::initCmdLine(argc,argv);
  ors::KinematicWorld world;
  world.init(STRING("scene"));
  makeConvexHulls(world.shapes);

  nh.setParam("tree_rt_controller/type","tree_controller_pkg/TreeControllerPlugin");

  /// Stop R_ARM controller and start tree_controller
  ros::ServiceClient loadControllerClient = nh.serviceClient<pr2_mechanism_msgs::LoadController>("/pr2_controller_manager/load_controller");
  pr2_mechanism_msgs::LoadController loadControllerSrv;
  loadControllerClient.waitForExistence();
  loadControllerSrv.request.name = "tree_rt_controller";
  if (!loadControllerClient.call(loadControllerSrv)) {
    ROS_ERROR("TREE CONTROLLER COULD NOT BE LOADED");
    return 0;
  }



  /// Check if tree_controller is loaded
  ros::ServiceClient getInitJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state");
  if (!getInitJointStateClient.waitForExistence(ros::Duration(3.))) {
    ROS_ERROR("TREE CONTROLLER NOT FOUND");
    ROS_ERROR("EXECUTION IS STOPPED");
    return 0;
  }

  /// Get init position from robot
  tree_controller_pkg::GetJointState getInitJointStateSrv;
  getInitJointStateClient.call(getInitJointStateSrv);
  arr q0(getInitJointStateSrv.response.q.size());
  for(uint i=0;i<q0.d0;i++) q0(i) = getInitJointStateSrv.response.q[i];



  /// Get goal from AR tag or scene file
  bool useARtag;
  arr goal;
  ros::ServiceClient get_init_goal_client = nh.serviceClient<goal_publisher::GetGoal>("/get_goal");
  goal_publisher::GetGoal initCamGoal;
  if (get_init_goal_client.waitForExistence(ros::Duration(1.0))){
    // Use AR Tag
    useARtag = true;
    get_init_goal_client.call(initCamGoal);
    arr refFrame = ARRAY(world.getBodyByName("torso_lift_link")->X.pos);
    goal = refFrame + ARR(initCamGoal.response.x,initCamGoal.response.y,initCamGoal.response.z);
  } else {
    // Use goal from scene file
    useARtag = false;
    goal = ARRAY(world.getBodyByName("goalRef")->X.pos);
  }

  /// Create reference plan with optimizer
  TrajOptimizer to(world);
  to.optimizeTrajectory(goal,q0);



  ros::ServiceClient switchControllerClient = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("/pr2_controller_manager/switch_controller");
  pr2_mechanism_msgs::SwitchController switchControllerSrv;
  switchControllerSrv.request.start_controllers.resize(1);
  switchControllerSrv.request.stop_controllers.resize(1);
  switchControllerSrv.request.start_controllers[0] = STRING("tree_rt_controller");
  switchControllerSrv.request.stop_controllers[0] = STRING("r_arm_controller");
  switchControllerSrv.request.strictness = switchControllerSrv.request.BEST_EFFORT;
  if (!switchControllerClient.call(switchControllerSrv)) {
    ROS_ERROR("TREE CONTROLLER COULD NOT BE STARTED");
    return 0;
  }

  /// Start Logging
  ros::ServiceClient startLoggingClient = nh.serviceClient<tree_controller_pkg::StartLogging>("/tree_rt_controller/start_logging");
  tree_controller_pkg::StartLogging startLoggingSrv;
  startLoggingClient.call(startLoggingSrv);


  /// Start adaptive motion execution
  AmexController amex(nh, world, to.refPlan, q0, to.TRef, useARtag);
  amex.initRosServices();
  amex.initController();
  amex.startController();


  /// Stop Logging
  ros::ServiceClient stopLoggingClient = nh.serviceClient<tree_controller_pkg::StopLogging>("/tree_rt_controller/stop_logging");
  tree_controller_pkg::StopLogging stopLoggingSrv;
  stopLoggingClient.call(stopLoggingSrv);
  return 0;
}
