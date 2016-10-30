#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>


typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;



class RobotArm
{
private:
  TrajClient* traj_client_;

public:
  RobotArm()
  {
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action",true);

    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  ~RobotArm()
  {
    delete traj_client_;
  }

  void startTrajectory(control_msgs::JointTrajectoryGoal goal)
  {
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);

  }



  control_msgs::JointTrajectoryGoal armExtensionTrajectory()
  {
    ROS_INFO("START INIT MOTION");
    control_msgs::JointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");


//joint_names: ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
//desired: 
//  positions: [-0.34531430888997544, 0.3297327434661644, -0.12439335122114858, -0.24174029484874937, -9.008380810497305e-07, -0.08683316527546325, -1.9259652495051682e-06]
//    double q0[7] = {-0.1243,-0.345,0.3297,0.,-0.2417,-0.0868,0.};

//    joint_names: ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']

    double q0[7] = {-0.7788, 0.7109, 0.02537, -0.15256, 0.0680, -0.0764, -0.16178};

    //double q0[7] = {-10.,-20.,20.,0.,-20.,-5.,0.};
    goal.trajectory.points.resize(1);

    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[0].positions[j] = q0[j];//*M_PI/180.;
      goal.trajectory.points[0].velocities[j] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(10.0);


    //we are done; return the goal
    return goal;
  }

  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc,argv,"robot_arm_control");

  ros::NodeHandle nh;

  RobotArm arm;
  arm.startTrajectory(arm.armExtensionTrajectory());
  while(!arm.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }
}
