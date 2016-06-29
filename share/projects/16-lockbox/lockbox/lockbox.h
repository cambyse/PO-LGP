#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <RosCom/filterObject.h>
#include <Control/TaskControllerModule.h>

#include "../interface/myBaxter.h"
#include <unordered_map>

struct Lockbox:Module{

  ACCESSname(FilterObjects, object_database)

  Lockbox(MyBaxter* baxter) : Module("lockbox", -1){
    threadOpenModules(true);
    myBaxter = baxter;
  }
  ~Lockbox(){
    threadCloseModules();
  }

  void open();
  void step();
  void close();

  // New methods
  void initializeJoints();
//  void syncronizeJoints(); // Synchronize inputs with the simulated box, move the simulated box.
  void moveJoint(const uint joint, const double position); // Position should be 0-1, relative to limits

  void fixJoint(const uint joint, const bool fix);

  std::unordered_map<uint, mlr::String> joint_to_ors_joint;
  std::unordered_map<uint, mlr::String> joint_to_handle;


  MyBaxter* myBaxter;

  bool update = true;
  bool simulate = false;
};
