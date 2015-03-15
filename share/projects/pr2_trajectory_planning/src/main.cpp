#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "sensor_msgs/JointState.h"

#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>
#include <Motion/taskMaps.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <ctime>

#include <devTools/logging.h>
SET_LOG(main, DEBUG);

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class RobotArm {
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while (!traj_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm() {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal) {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  pr2_controllers_msgs::JointTrajectoryGoal trajectoryToPR2Msg(const arr& trajectory) {
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.points.resize(trajectory.d0);

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

    goal.trajectory.points[0].time_from_start = ros::Duration(0.1);

    for(uint idx = 0; idx < trajectory.d0; ++idx) {
      goal.trajectory.points[idx].positions.resize(trajectory.d1);
      goal.trajectory.points[idx].velocities.resize(trajectory.d1);
      goal.trajectory.points[idx].time_from_start = ros::Duration(idx * .01);

      // joints
      for(uint p = 0; p < trajectory.d1; ++p) {
        goal.trajectory.points[idx].positions[p] = trajectory(idx, p);
        goal.trajectory.points[idx].velocities[p] = 0.0;
      }
    }
    return goal; 
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState() {
    return traj_client_->getState();
  }

};


arr create_endpose(ors::KinematicWorld& G) {
  MotionProblem P(&G);

  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  Task *c = P.addTask("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);

  c = P.addTask("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.world.getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  arr x = P.x0;
  keyframeOptimizer(x, P, false, 2);

  return x;
}

arr create_rrt_trajectory(ors::KinematicWorld& G, arr& target) {
  double stepsize = MT::getParameter<double>("rrt_stepsize", .005);

  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  Task *c = P.addTask("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);
  arr q = { 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003 };
  planner.joint_max = q + ones(q.N,q.N);
  planner.joint_min = q - ones(q.N,q.N);
  std::cout << "Planner initialized" <<std::endl;
  
  return planner.getTrajectoryTo(target);
}

arr optimize_trajectory(ors::KinematicWorld& G, arr& init_trajectory) {
  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();
  P.T = init_trajectory.d0-1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  Task *c = P.addTask("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e1);

  c = P.addTask("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.world.getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  MotionProblemFunction MF(P);
  arr x = init_trajectory;
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=40, damping=1e-0, maxStep=1.));
  return x;
}

void show_trajectory(ors::KinematicWorld& G, OpenGL& gl, arr& trajectory, const char* title) {
  arr start;
  G.getJointState(start);
  displayTrajectory(trajectory, trajectory.d0, G, gl, title);
  gl.watch();
  G.setJointState(start);
}


arr joint_state;
void callback(const sensor_msgs::JointState::ConstPtr& state) {

  cout << "bla" << endl;
  joint_state.resize(7);
  std::vector<std::string> names = {
    "l_shoulder_pan_joint",
    "l_shoulder_lift_joint",
    "l_upper_arm_roll_joint",
    "l_forearm_roll_joint",
    "l_elbow_flex_joint",
    "l_wrist_flex_joint",
    "l_wrist_roll_joint"
  };

  for (uint i = 0; i < state->name.size(); i++) {
    auto iter = std::find(names.begin(), names.end(), state->name[i]);
    if (iter != names.end()) {
      joint_state(std::distance(names.begin(), iter)) = state->position[i];
    }
  }
}


int main(int argc, char** argv) {
  // Init the ROS node
  ros::init(argc, argv, "robot_dri");
  ros::NodeHandle nh;
  ros::Subscriber joint_sub = nh.subscribe("/joint_states", 100, callback);
  while (joint_state.N == 0)
    ros::spinOnce();
  cout << "after wait...proceding" << endl;

  // MLR
  MT::initCmdLine(argc,argv);
  int seed = time(NULL);

  rnd.seed(seed);

  ors::KinematicWorld G("world.ors");
  makeConvexHulls(G.shapes);

  OpenGL gl;
  bindOrsToOpenGL(G, gl);

  std::cout << "q = " << joint_state << std::endl;
  G.setJointState(joint_state);
  
  arr start = G.getJointState();
  std::cout << "q = " << start << std::endl;

  arr target = create_endpose(G);
  G.setJointState(start);

  std::cout << "target = " << target << std::endl;

  arr rrt_trajectory = create_rrt_trajectory(G, target);
  show_trajectory(G, gl, rrt_trajectory, "RRT");

  arr opt_trajectory = optimize_trajectory(G, rrt_trajectory);
  DEBUG_VAR(main, opt_trajectory);
  MT::wait();
  show_trajectory(G, gl, opt_trajectory, "optimized");

  std::cout << "Should I run the trajectory on the /real/ PR2? (y/N)" << std::endl;
  char answer;
  std::cin >> answer;

  if (answer != 'y') return 0;

  RobotArm arm;
  // Start the trajectory
  // auto goal = arm.trajectoryToPR2Msg(opt_trajectory);

  // for (uint i = 0; i < goal.trajectory.joint_names.size(); i++) {
  //   std::cout << goal.trajectory.joint_names[i] << " " << opt_trajectory(opt_trajectory.d0-1, i) << std::endl;
  // }
  // arm.startTrajectory(arm.trajectoryToPR2Msg(zeros(7)));
  arm.startTrajectory(arm.trajectoryToPR2Msg(opt_trajectory));
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok()) {
    usleep(50000);
  }
  return 0;
}

