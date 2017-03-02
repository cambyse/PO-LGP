#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <Kin/kin.h>
#include <Perception/percept.h>
#include <Control/TaskControlThread.h>

#include "../interface/myBaxter.h"
#include <unordered_map>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <Perception/filter.h>
#include <Perception/percept.h>
#include <RosCom/publishDatabase.h>

#include <std_msgs/UInt8.h>

struct SubscribeJointPosition{
  ACCESSname(std_msgs::UInt8, get_joint_position)

  Subscriber<std_msgs::UInt8> sub;

  SubscribeJointPosition()
    : sub("/lockbox/joint_position", get_joint_position) {
  }
  ~SubscribeJointPosition(){
  }
};

struct SubscribeTestJoint{
  ACCESSname(std_msgs::UInt8, test_joint)

  Subscriber<std_msgs::UInt8> sub;

  SubscribeTestJoint()
    : sub("/lockbox/test_joint", test_joint) {
  }
  ~SubscribeTestJoint(){
  }

};

struct Lockbox:Module{
  Access_typed<std_msgs::UInt8> test_joint;
  Access_typed<std_msgs::UInt8> get_joint_position;
  Access_typed<Percepts> percepts_filtered;

  SubscribeAlvar alvar_subscriber;
  SubscribeTestJoint test_joint_sub;
  SubscribeJointPosition joint_position_sub;

  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;

  ros::NodeHandle* nh;
  ros::Publisher joint_position_publisher;
  ros::Publisher test_joint_publisher;

  Lockbox(MyBaxter* baxter);
  ~Lockbox();

  void open(){}
  void step();
  void close(){}

  //////// Johannes functions
  bool testJoint(const uint joint);
  bool moveJoint(const uint joint);
  double getJointPosition(const uint joint);

  // New methods
  void initializeJoints();
//  void syncronizeJoints(); // Synchronize inputs with the simulated box, move the simulated box.
//  void moveJoint(const uint joint, const double position); // Position should be 0-1, relative to limits

  void moveHome(const bool stopAllOtherTasks = false);
  void fixJoint(const uint joint, const bool fix);
  void grip(const bool toGrip);

  void update();
  bool updatedJointPose(const uint joint_num, arr& new_q);

  std::unordered_map<uint, mlr::String> joint_to_ors_joint;
  std::unordered_map<uint, mlr::String> joint_to_handle;
  std::unordered_map<uint, mlr::String> joint_name;

  mlr::Array<uint> locked_joints;
  std::unordered_map<uint, double> joint_positions;

  bool queryContinue();

  MyBaxter* myBaxter;

//  bool update = true;
  bool usingRos = false;

  mlr::KinematicWorld lockbox_world;

  CtrlTaskL joint_fixed_tasks;
  CtrlTask* grip_task;
  arr q0;
  bool readyToTest = false;

  int test_joint_revision = -1, joint_position_revision = -1;
};
