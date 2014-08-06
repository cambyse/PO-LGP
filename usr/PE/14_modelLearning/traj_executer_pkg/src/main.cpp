#include <ros/ros.h>
#include <Core/array.h>
#include <Core/array-vector.h>
#include "traj_optimizer.h"
#include <tree_controller_pkg/GetJointState.h>
#include <tree_controller_pkg/StartLogging.h>
#include <tree_controller_pkg/StopLogging.h>
#include <tree_controller_pkg/SetCommand.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include "gp_control.h"

int main(int argc, char** argv)
{
  /// Init ros
  ros::init(argc, argv, "traj_executer");
  ros::NodeHandle nh;

  /// Init ors
  MT::initCmdLine(argc,argv);
  ors::KinematicWorld world;
  world.init(STRING("scene"));
  makeConvexHulls(world.shapes);

  /// Check if tree_controller is loaded
  ros::ServiceClient getInitJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state");
  if (!getInitJointStateClient.waitForExistence(ros::Duration(3.))) {
    ROS_ERROR("TREE CONTROLLER NOT FOUND");
    ROS_ERROR("EXECUTION IS STOPPED");
    return 0;
  }

  /// Start Logging
  ros::ServiceClient startLoggingClient = nh.serviceClient<tree_controller_pkg::StopLogging>("/tree_rt_controller/start_logging");
  tree_controller_pkg::StartLogging startLoggingSrv;
  startLoggingClient.call(startLoggingSrv);

  /// Get init position from robot
  tree_controller_pkg::GetJointState getInitJointStateSrv;
  getInitJointStateClient.call(getInitJointStateSrv);
  arr q0 = ARRAY(getInitJointStateSrv.response.q);
  cout << "g0: " << q0 << endl;

  /// Start adaptive motion execution
  ros::ServiceClient setCommandClient = nh.serviceClient<tree_controller_pkg::SetCommand>("/tree_rt_controller/set_command");
  tree_controller_pkg::SetCommand setCommandSrv;


  double dt = 0.01;
  TrajOptimizer* to = new TrajOptimizer(world);
  arr x,xd,xdd,goal;
  to->sampleGoal(goal,q0);
  cout << "goal: " << goal << endl;
  to->optimizeTrajectory(goal,q0,x);

  getVel(xd,x,dt);
  getAcc(xdd,x,dt);

  GPControl* gp = new GPControl();

  arr q,qd,qdd,u;
  for (uint t =0;t<x.d0;t++) {
    ros::Rate loop_rate(1./dt);

    q = x[t];
    qd = xd[t];
    qdd = xdd[t];

    arr state;
    state.append(q);
    state.append(qd);
    state.append(qdd);
    gp->predict(state,u);

    setCommandSrv.request.q = VECTOR(q);
    setCommandSrv.request.qd = VECTOR(qd);
    setCommandSrv.request.uGP = VECTOR(u);

    setCommandClient.call(setCommandSrv);

    loop_rate.sleep();
  }

  /// Stop Logging
  ros::ServiceClient stopLoggingClient = nh.serviceClient<tree_controller_pkg::StopLogging>("/tree_rt_controller/stop_logging");
  tree_controller_pkg::StopLogging stopLoggingSrv;
//  stopLoggingClient.call(stopLoggingSrv);
  return 0;
}
