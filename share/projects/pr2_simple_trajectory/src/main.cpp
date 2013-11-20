#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_default.h>
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

  /**
   * Generates a simple trajectory with two waypoints, used as an example.
   *
   * Note that this trajectory contains two waypoints, joined together as
   * a single trajectory.  Alternatively, each of these waypoints could be in
   * its own trajectory a trajectory can have one or more waypoints depending
   * on the desired application.
   */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory() {
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = 0.0;
    goal.trajectory.points[ind].positions[1] = 0.0;
    goal.trajectory.points[ind].positions[2] = 0.0;
    goal.trajectory.points[ind].positions[3] = 0.0;
    goal.trajectory.points[ind].positions[4] = 0.0;
    goal.trajectory.points[ind].positions[5] = 0.0;
    goal.trajectory.points[ind].positions[6] = 0.0;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(7);
    goal.trajectory.points[ind].positions[0] = -0.3;
    goal.trajectory.points[ind].positions[1] = 0.2;
    goal.trajectory.points[ind].positions[2] = -0.1;
    goal.trajectory.points[ind].positions[3] = -1.2;
    goal.trajectory.points[ind].positions[4] = 1.5;
    goal.trajectory.points[ind].positions[5] = -0.3;
    goal.trajectory.points[ind].positions[6] = 0.5;
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  pr2_controllers_msgs::JointTrajectoryGoal trajectoryToPR2Msg(const arr& trajectory) {
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.points.resize(trajectory.d1);

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("l_wrist_roll_joint");

    for(uint idx = 0; idx < trajectory.d1; ++idx) {
      goal.trajectory.points[idx].positions.resize(trajectory.d0);
      for(uint p = 0; p < trajectory.d0; ++p) {
        goal.trajectory.points[idx].positions[p] = trajectory(idx, p);  
      }  
    }
    return goal; 
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState() {
    return traj_client_->getState();
  }

};


arr create_endpose(ors::Graph& G) {
  MotionProblem P(&G);

  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  arr x = P.x0;
  keyframeOptimizer(x, P, false, 2);

  return x;
}

arr create_rrt_trajectory(ors::Graph& G, arr& target) {
  double stepsize = MT::getParameter<double>("rrt_stepsize", .005);

  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);
  arr q = { 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003 };
  planner.joint_max = q + ones(q.N, 1);
  planner.joint_min = q - ones(q.N, 1);
  std::cout << "Planner initialized" <<std::endl;
  
  return planner.getTrajectoryTo(target);
}

arr optimize_trajectory(ors::Graph& G, arr& init_trajectory) {
  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();
  P.T = init_trajectory.d0-1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e1);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  MotionProblemFunction MF(P);
  arr x = init_trajectory;
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=40, useAdaptiveDamping=false, damping=1e-0, maxStep=1.));
  return x;
}

void show_trajectory(ors::Graph& G, OpenGL& gl, arr& trajectory, const char* title) {
  arr start;
  G.getJointState(start);
  displayTrajectory(trajectory, trajectory.d0, G, gl, title);
  gl.watch();
  G.setJointState(start);
}

int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  int seed = time(NULL);

  rnd.seed(seed);

  ors::Graph G("world.ors");
  makeConvexHulls(G.shapes);

  OpenGL gl;
  bindOrsToOpenGL(G, gl);

  arr start = G.getJointState();
  std::cout << "q = " << start << std::endl;

  arr target = create_endpose(G);
  G.setJointState(start);

  std::cout << "target = " << target << std::endl;

  arr rrt_trajectory = create_rrt_trajectory(G, target);
  show_trajectory(G, gl, rrt_trajectory, "RRT");

  arr opt_trajectory = optimize_trajectory(G, rrt_trajectory);
  show_trajectory(G, gl, opt_trajectory, "optimized");

  std::cout << "Should I run the trajectory on the /real/ PR2? (y/N)" << std::endl;
  char answer;
  std::cin >> answer;

  if (answer != 'y') return 0;

  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.trajectoryToPR2Msg(opt_trajectory));
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok()) {
    usleep(50000);
  }
  return 0;
}

