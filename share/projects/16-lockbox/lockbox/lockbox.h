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

  bool getAbsoluteJointTransform(const uint joint, ors::Transformation& tf);
  bool getOriginalJointTransform(const uint joint, ors::Transformation& orig);

  bool getJointTransform(const uint joint, ors::Transformation& diff);
  bool getJointPosition(const uint joint, double& position);

  void open();
  void step();
  void close();

  bool moveToJointStub(const uint joint, const ors::Transformation& offset, const double speed = 1, const double prec = 10);
  bool moveToAlvar(const uint joint, const ors::Transformation& offset, const double speed = 1, const double prec = 10);
  bool moveRelative(const ors::Vector& offset, const double speed = 1, const double prec = 10);
  bool moveAbsolute(const ors::Vector& offset, const double speed = 1, const double prec = 10);

  bool calculateStubOffset(const uint joint, const ors::Transformation& alvar_tf, ors::Transformation& newStub);
  void moveJointToPosition(const uint joint, const double position);

  //  std::unordered_map<uint, ors::Transformation> offsets = {

  std::unordered_map<uint, arr> offsets = {
    std::make_pair(1, ARR(-0.05, -0.15, 0.01)),
    std::make_pair(2, ARR(-0.04, 0.1, 0.17)),
    std::make_pair(3, ARR(-0.06, 0.11, 0.11)),
    std::make_pair(4, ARR(-0.06, 0.13, 0.0)),
    std::make_pair(5, ARR(-0.06, 0.15, -0.02))
  };


  std::unordered_map<uint, arr> end_offsets = {
    std::make_pair(1, ARR(-0.16, 0.34, 0.01)),
    std::make_pair(2, ARR(0, 0.0, 0.20)),
    std::make_pair(3, ARR(0, -0.02, -0.16)),
    std::make_pair(4, ARR(0, -0.1, 0)),
    std::make_pair(5, ARR(-0.1, 0, -0.05))
  };

  std::unordered_map<uint, ors::Quaternion> end_orientations;


//private:
  ors::Transformation fixed_alvar;
  std::unordered_map<uint, ors::Transformation> joint_origins;
  std::unordered_map<uint, ors::Transformation> joint_tfs;
  std::unordered_map<uint, ors::Transformation> joint_positions;

  void readJointPositions();
  MyBaxter* myBaxter;


  bool update = true;
};
