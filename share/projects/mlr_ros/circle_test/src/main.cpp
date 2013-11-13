#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Core/array.h>
#include <Core/keyValueGraph.h>
#include <Ors/roboticsCourse.h>
#include <Gui/opengl.h>


typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

struct sSimulator {
  ors::Graph G;
  OpenGL gl;
  SwiftInterface swift;
  double margin;
  double dynamicNoise;
  bool gravity;
  
  //state
  arr q,qdot,qddot;

#ifdef MT_ODE
  OdeInterface ode;
#endif
  sSimulator(){ margin=.1; dynamicNoise=0.; gravity=true; } //default margin = 10cm
};



class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
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
    for (size_t j = 0; j < 7; ++j)
    {
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
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

void circle(){
  Simulator S("pr2_model/pr2_left.ors");
  arr q,W;
  uint n = S.getJointDimension();
  cout <<"n=" <<n <<endl;
  S.getJointAngles(q);
  W.setDiag(10.,n);
  W(0,0) = 1e3;

  //our goal variable
  pr2_controllers_msgs::JointTrajectoryGoal goal;

  // First, the joint names, which apply to all waypoints
  //goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");

  S.watch();
  ors::Graph& G=S.s->G;
  for(uint i=0;i<G.joints.N;i++){
       goal.trajectory.joint_names.push_back(G.joints(i)->name.p);
       cout <<G.joints(i)->name <<endl;
  }
    //pr2_controllers_msgs::JointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    //goal.trajectory.joint_names.push_back("l_wrist_roll_link");

  arr y_target,y,J;
  for(uint i=0;i<1000;i++){
    S.kinematicsPos(y,"l_wrist_roll_link");
    S.jacobianPos  (J,"l_wrist_roll_link");
#if 1
    y_target = ARR(0.5, .2, 1.1); 
    y_target += .2 * ARR(0, cos((double)i/20), sin((double)i/20)); 
#else
    y_target = y;
    y_target(2) += 0.1;
#endif
    cout <<i <<" current eff pos = " <<y <<"  current error = " <<norm(y_target-y) <<endl;;
    double prec=1e-0;
    q += inverse(~J*J + W/prec)*~J*(y_target - y); 
    S.setJointAngles(q);
    
    for(int i=0; i < q.N; ++i) {
      goal.trajectory.points[0].positions.resize(q.N);
      goal.trajectory.points[0].positions[i] = q(i);
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  }
  S.watch();
}


 
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "robot_driver");

  KeyValueGraph G;
  G.sortByDotOrder();

  RobotArm arm;
  // Start the trajectory
  arm.circle();
  // Wait for trajectory completion
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
