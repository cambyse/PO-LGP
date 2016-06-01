#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <RosCom/filterObject.h>

#include <unordered_map>


struct Lockbox:Module{

  ACCESSname(FilterObjects, object_database)

  Lockbox() : Module("lockbox", -1){
    threadOpenModules(true);
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

  //  std::unordered_map<uint, ors::Transformation> offsets = {

  std::unordered_map<uint, arr> offsets = {
    std::make_pair(1, ARR(-0.04, -0.06, 0)),
    std::make_pair(2, ARR(-0.04, 0.02, 0.16)),
    std::make_pair(3, ARR(-0.04, 0.1, 0.1)),
    std::make_pair(4, ARR(-0.07, 0.12, 0)),
    std::make_pair(5, ARR(-0.07, 0.1, 0.0))
  };


private:
  ors::Transformation fixed_alvar;
  std::unordered_map<uint, ors::Transformation> joint_origins;
  std::unordered_map<uint, ors::Transformation> joint_positions;
  void readJointPositions();
};
