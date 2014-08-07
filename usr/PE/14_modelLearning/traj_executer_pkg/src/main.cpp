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
  srand(time(NULL));

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
  /// Stop Logging

  ros::ServiceClient stopLoggingClient = nh.serviceClient<tree_controller_pkg::StopLogging>("/tree_rt_controller/stop_logging");
  tree_controller_pkg::StopLogging stopLoggingSrv;
  stopLoggingClient.call(stopLoggingSrv);

  /// Get init position from robot
  tree_controller_pkg::GetJointState getInitJointStateSrv;
  getInitJointStateClient.call(getInitJointStateSrv);
  arr q0 = ARRAY(getInitJointStateSrv.response.q);

  ros::ServiceClient setCommandClient = nh.serviceClient<tree_controller_pkg::SetCommand>("/tree_rt_controller/set_command");
  tree_controller_pkg::SetCommand setCommandSrv;


  double dt = 0.01;
  TrajOptimizer* to = new TrajOptimizer(world);
  arr x,xd,xdd,goal;
  to->sampleGoal(goal,q0);
  cout << "q0: " << q0 << endl;
  cout << "goal: " << goal << endl;
  to->optimizeTrajectory(goal,q0,x);

  getVel(xd,x,dt);
  getAcc(xdd,x,dt);


  cout <<"Duration: " <<  x.d0*dt << endl;
  /// Start trajectory execution
  cout << "Start Trajectory [Enter]" << endl;
  MT::wait();

  /// Start Logging
  ros::ServiceClient startLoggingClient = nh.serviceClient<tree_controller_pkg::StopLogging>("/tree_rt_controller/start_logging");
  tree_controller_pkg::StartLogging startLoggingSrv;
  startLoggingClient.call(startLoggingSrv);

  ros::Rate loop_rate(1./dt);

  cout << "Send Initial Command" << endl;
  arr u;
  GPControl* gp = new GPControl();
  for (uint t =0;t<200;t++) {
    arr state;
    state.append(q0);
    state.append(q0*0.);
    state.append(q0*0.);

    gp->predict(state,u);

    setCommandSrv.request.q = VECTOR(q0);
    setCommandSrv.request.qd = VECTOR(q0*0.);
    setCommandSrv.request.uGP = VECTOR(u);
    setCommandClient.call(setCommandSrv);
    loop_rate.sleep();

  }


  cout << "Send Trajectory Commands" << endl;
  arr q,qd,qdd;

  MT::timerStart(true);

  for (uint t =0;t<x.d0;t++) {
    q = x[t];
    qd = xd[t];
    qdd = xdd[t];

    arr state;
    state.append(q);
    state.append(qd);
    state.append(qdd);

    gp->predict(state,u);
//    u = q*0.;
    double timeOff = MT::timerRead(true);

    setCommandSrv.request.q = VECTOR(q);
    setCommandSrv.request.qd = VECTOR(qd);
    setCommandSrv.request.uGP = VECTOR(u);
    setCommandClient.call(setCommandSrv);
    cout << timeOff << endl;
    cout << "max(ARR(0.01-timeOff,0.001)): " << max(ARR(0.01-timeOff,0.001)) << endl;
//    MT::wait(max(ARR(0.01-timeOff,0.001)));
    ros::Rate loop_rate2(1./(max(ARR(0.01-timeOff,0.001))));
    loop_rate2.sleep();
    cout << t*dt << endl;
    cout << "sleep time: " << MT::timerRead(true) << endl;
  }

  cout << "Trajectory finished!" << endl;

  // send zero torques
  /*
  setCommandSrv.request.uGP = VECTOR(u*0.);
  setCommandClient.call(setCommandSrv);*/

  // stop logging
  stopLoggingClient.call(stopLoggingSrv);

  return 0;
}
