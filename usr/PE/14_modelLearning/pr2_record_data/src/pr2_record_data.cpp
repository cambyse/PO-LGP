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

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
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

    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);

//    double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -2.3213, -3.14, -2.18 ,-3.14};
//    double qUpperLimit[7] = {0.714602, 1.3963, 0.8, 0., 3.14, 0., 3.14};

    double qLowerLimit[7] = {-2.2854, -0.5236, -3.9, -1.7, -3.14, -2. ,-3.14};
    double qUpperLimit[7] = {0.714602, 0.6, 0.8, 0., 3.14, 0., 3.14};



    // randomly sample for each joint a goal position
    for (uint i = 0; i< goal.trajectory.joint_names.size();i++){
      std::cout <<fRand(qLowerLimit[i],qUpperLimit[i]) << std::endl;
      goal.trajectory.points[0].positions[i] = fRand(qLowerLimit[i],qUpperLimit[i]);
      goal.trajectory.points[0].velocities[i] = 0.0;
    }

    goal.trajectory.points[0].time_from_start = ros::Duration(fRand(4.,7.));

    // return the goal
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
  srand(2);
  RobotArm arm;
  while(ros::ok()){
    arm.startTrajectory(arm.armExtensionTrajectory());
    while(!arm.getState().isDone() && ros::ok())
    {
      usleep(50000);
    }
  }
}
