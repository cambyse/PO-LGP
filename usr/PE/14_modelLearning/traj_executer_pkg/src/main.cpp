#include <ros/ros.h>
#include <Core/array.h>
#include <Core/array-vector.h>
#include <Algo/spline.h>
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


  arr x,xd,xdd,goal;
  TrajOptimizer* to = new TrajOptimizer(world);

  to->sampleGoal(goal,q0);

  // 30: r_shoulder_pan_joint,
  // 31: r_shoulder_lift_joint,
  // 32: r_upper_arm_roll_joint,
  // 33: r_elbow_flex_joint,
  // 34: r_forearm_roll_joint,
  // 35: r_wrist_flex_joint,
  // 36: r_wrist_roll_joint,
//  double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, _q0(4)-M_PI, -2. ,_q0(6)-M_PI};
//  double qUpperLimit[7] = {0.714602, 0.6,     0.8,   0., _q0(4)+M_PI,  0., _q0(6)+M_PI};
//  goal = ARR(-0.0167554, 0.246322, 0.27938, -0.166749, -0.237116, -0.0891157, 5.3637 );

  to->optimizeTrajectory(goal,q0,x);

  cout << "q0: " << q0 << endl;
  cout << "goal: " << goal << endl;

  double dt = to->dt;
  getVel(xd,x,dt);
  getAcc(xdd,x,dt);

  /// create spline
  MT::Spline s(x.d0,x);
  MT::Spline sd(xd.d0,xd);
  MT::Spline sdd(xdd.d0,xdd);

  double dur = (x.d0-1)*dt;
  cout <<"Duration: " << dur << endl;

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
  double t = 0.;
  while ( t < dur ) {
    q = s.eval( t/dur );
    qd = sd.eval( t/dur );
    qdd = sdd.eval( t/dur );

    arr state;
    state.append(q);
    state.append(qd);
    state.append(qdd);

    gp->predict(state,u);
    setCommandSrv.request.q = VECTOR(q);
    setCommandSrv.request.qd = VECTOR(qd);
    setCommandSrv.request.uGP = VECTOR(u*0.);
    setCommandClient.call(setCommandSrv);

    double timeDiff = MT::timerRead(true);
    t = t + timeDiff;

  }

  cout << "Trajectory finished!" << endl;

  // stop logging
  stopLoggingClient.call(stopLoggingSrv);

  return 0;
}
