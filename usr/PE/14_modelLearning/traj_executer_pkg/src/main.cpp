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
  ros::ServiceClient getInitJointStateClient = nh.serviceClient<tree_controller_pkg::GetJointState>("/tree_rt_controller/get_joint_state",true);
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

  ros::ServiceClient setCommandClient = nh.serviceClient<tree_controller_pkg::SetCommand>("/tree_rt_controller/set_command",true);
  tree_controller_pkg::SetCommand setCommandSrv;


  arr x,xd,xdd,goal;
  TrajOptimizer* to = new TrajOptimizer(world);

  // 30: r_shoulder_pan_joint,
  // 31: r_shoulder_lift_joint,
  // 32: r_upper_arm_roll_joint,
  // 33: r_elbow_flex_joint,
  // 34: r_forearm_roll_joint,
  // 35: r_wrist_flex_joint,
  // 36: r_wrist_roll_joint,
  // move to hand specified goal
  //  double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, _q0(4)-M_PI, -2. ,_q0(6)-M_PI};
  //  double qUpperLimit[7] = {0.714602, 0.6,     0.8,   0., _q0(4)+M_PI,  0., _q0(6)+M_PI};
//
  uint type = MT::getParameter<uint>("type");
  switch (type){
    case 0:

//      goal = ARR(0.0902771, 0.453495, 0.310329, -1.0775, -0.357554, -1.03557, 6.47796 ); // circle
//      goal = ARR(0.0656538, 0.472021, 0.321393, -1.22169 ,-0.353447, -0.814975, 6.48104); // eight
      goal = ARR(0.215881, 0.257827, 0.544928, -0.869321, -0.336787, -1.25407, 6.47457); // star

      to->optimizeTrajectory(goal,q0,x);
      break;
    case 1:
      // sample random goal and move there
      to->sampleGoal(goal,q0);
      to->optimizeTrajectory(goal,q0,x);
      break;
    case 2:
      // move a benchmark motion
      to->optimizeBenchmarkMotion(BM_TYPE::STAR,q0,x);
      break;
  }

  cout << "q0: " << q0 << endl;

  double dt = to->dt;
  getVel(xd,x,dt);
  getAcc(xdd,x,dt);

  /// create spline
  MT::Spline s(x.d0,x);
  MT::Spline sd(xd.d0,xd);
  MT::Spline sdd(xdd.d0,xdd);

  double dur = (x.d0-1)*dt;
  cout <<"Duration: " << dur << endl;
  cout <<"dt: " << dt << endl;

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

  arr traj_act, y_act, q_act;
  arr traj_ref, y_ref;
  arr pred_time;

  tree_controller_pkg::GetJointState getJointStateSrv;

  MT::timerStart(true);
  double t = 0.;
  while ( t < dur ) {
    // get actual joint state
    getInitJointStateClient.call(getJointStateSrv);
    q_act = ARRAY(getJointStateSrv.response.q);
    world.setJointState(q_act);
    world.kinematicsPos(y_act,NoArr,world.getBodyByName("endeffR"));
    traj_act.append(~y_act);

    // send next command
    q = s.eval( t/dur );
    qd = sd.eval( t/dur );
    qdd = sdd.eval( t/dur );

    // get reference joint state
    world.setJointState(q);
    world.kinematicsPos(y_ref,NoArr,world.getBodyByName("endeffR"));
    traj_ref.append(~y_ref);

    arr state;
    state.append(q);
    state.append(qd);
    state.append(qdd);

    double pt = MT::timerRead(false);
    gp->predict(state,u);
    pt = MT::timerRead(false) - pt;
    pred_time.append(pt);

    setCommandSrv.request.q = VECTOR(q);
    setCommandSrv.request.qd = VECTOR(qd);
    setCommandSrv.request.uGP = VECTOR(u);
    setCommandClient.call(setCommandSrv);

    double timeDiff = MT::timerRead(true);
    t = t + timeDiff;
//    cout << t << endl;
  }

  cout << "Trajectory finished!" << endl;

  // stop logging
  stopLoggingClient.call(stopLoggingSrv);

  write(LIST<arr>(traj_ref),"traj_ref");
  write(LIST<arr>(traj_act),"traj_act");
  write(LIST<arr>(pred_time),"pred_time");

  return 0;
}
