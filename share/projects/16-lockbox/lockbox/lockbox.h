#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <RosCom/filterObject.h>
#include <Control/TaskControllerModule.h>

#include "../interface/myBaxter.h"
#include <unordered_map>

#include <RosCom/subscribeAlvarMarkers.h>
#include <RosCom/subscribeTabletop.h>
#include <RosCom/perceptionCollection.h>
#include <RosCom/perceptionFilter.h>
#include <RosCom/filterObject.h>
#include <RosCom/publishDatabase.h>

struct Lockbox:Module{

  Access_typed<FilterObjects> object_database;

  SubscribeAlvar alvar_subscriber;
  Collector data_collector;
  Filter myFilter;
  PublishDatabase myPublisher;

  Lockbox(MyBaxter* baxter);
  ~Lockbox();

  void open(){}
  void step(){}
  void close(){}

  // New methods
  void initializeJoints();
//  void syncronizeJoints(); // Synchronize inputs with the simulated box, move the simulated box.
  void moveJoint(const uint joint, const double position); // Position should be 0-1, relative to limits
  bool testJoint(const uint joint);

  void moveHome(const bool stopAllOtherTasks = false);
  void fixJoint(const uint joint, const bool fix);

  void update();
  bool updatedJointPose(const uint joint_num, arr& new_q);

  std::unordered_map<uint, mlr::String> joint_to_ors_joint;
  std::unordered_map<uint, mlr::String> joint_to_handle;
  std::unordered_map<uint, mlr::String> joint_name;

  mlr::Array<uint> locked_joints;

  bool queryContinue();

  MyBaxter* myBaxter;

//  bool update = true;
  bool usingRos = false;

  ors::KinematicWorld lockbox_world;

  CtrlTaskL joint_fixed_tasks;
  arr q0;
};
