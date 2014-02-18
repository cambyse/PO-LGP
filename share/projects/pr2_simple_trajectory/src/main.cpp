#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <Ors/ors.h>
#include <gtest/gtest.h>
#include <Gui/opengl.h>
#include <ctime>

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
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

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
    goal.trajectory.points[ind].positions[5] = -0.1;
    goal.trajectory.points[ind].positions[6] = 0.0;
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

  pr2_controllers_msgs::JointTrajectoryGoal OrsConfigToPR2Msg(const arr& q_last, const arr& q, const ors::KinematicWorld& G) {
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.points.resize(2);


    // First, the joint names, which apply to all waypoints
    uint COUNT=0;
    for(ors::Joint* j:G.joints) if(!j->mimic){
      goal.trajectory.joint_names.push_back(j->name.p);
//      cout <<j->name <<endl;
      COUNT++;
    }
    CHECK(q.N==COUNT,"DAMMIT");
    CHECK(q_last.N==COUNT,"DAMMIT");

    goal.trajectory.points[0].positions.resize(q_last.N);
    goal.trajectory.points[0].velocities.resize(q_last.N);
    for(uint i=0; i<q_last.N; i++) {
      goal.trajectory.points[0].positions[i] = 0.0; //q_last(i);
      goal.trajectory.points[0].velocities[i] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    goal.trajectory.points[1].positions.resize(q.N);
    goal.trajectory.points[1].velocities.resize(q.N);
    for(uint i=0; i<q.N; i++) {
      goal.trajectory.points[1].positions[i] = q(i);
      goal.trajectory.points[1].velocities[i] = 0.0;
    }
    goal.trajectory.points[1].time_from_start = ros::Duration(2.0);
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState() {
    return traj_client_->getState();
  }

};

void circle(bool FIRE=false){
  ors::KinematicWorld G;
  OpenGL gl;
  init(G, gl, "pr2_model/pr2_left.ors");
  makeConvexHulls(G.shapes);

  RobotArm *arm=NULL;
  if(FIRE){
    ros::init(MT::argc, MT::argv, "robot_driver");
    arm = new RobotArm;
    if(!ros::ok()) cout <<"INIT not ok" <<endl;
  }

  arr q,W,q_last;
  uint n = G.getJointStateDimension();
  G.getJointState(q);
  q_last=q;
  W.setDiag(10.,n);
  W(0,0) = 1e3;

  uint endeff = G.getBodyByName("l_wrist_roll_link")->index;
//  for(ors::Joint* j: G.joints) cout <<j->name <<endl;

  gl.watch();

  arr y_target,y,J;
  for(uint i=0;i<100;i++){
    q_last=q;

    G.kinematicsPos(y, endeff);
    G.jacobianPos  (J, endeff);
    y_target = ARR(0.5, .2, 1.1);
    //y_target += .2 * ARR(0, cos((double)i/20), sin((double)i/20));
    cout <<i <<" current eff pos = " <<y <<"  current error = " <<length(y_target-y) <<endl;;
    double prec=1e-0;
    q += inverse(~J*J + W/prec)*~J*(y_target - y);

    G.setJointState(q);
    gl.update();

    if(FIRE){
      arm->startTrajectory(arm->armExtensionTrajectory());
//      arm->startTrajectory(arm->OrsConfigToPR2Msg(q_last, q, G));
      if(!ros::ok()) cout <<"not ok" <<endl;
      while (!arm->getState().isDone() && ros::ok()) {
        usleep(50000);
      }
//      MT::wait(5.);
      break;
    }
  }
  gl.watch();

  if(FIRE){
    delete arm;
  }
}

void base(){
  // Init the ROS node
  ros::init(MT::argc, MT::argv, "robot_driver");

  RobotArm arm;
  // Start the trajectory
  arm.startTrajectory(arm.armExtensionTrajectory());
  // Wait for trajectory completion
  while (!arm.getState().isDone() && ros::ok()) {
    usleep(50000);
  }
}

int main(int argc, char** argv) {
  circle(true);
//  base();
}
