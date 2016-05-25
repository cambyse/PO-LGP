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

  bool getAbsoluteJointTransform(const uint joint, ors::Transformation& diff);

  bool getJointTransform(const uint joint, ors::Transformation& diff);
  bool getJointPosition(const uint joint, double& position);

  void open();
  void step();
  void close();


  std::unordered_map<uint, arr> offsets = {
    std::make_pair(1, ARR(0., 0, 0.05)),
    std::make_pair(2, ARR(0., 0, 0)),
    std::make_pair(3, ARR(0., 0, 0)),
    std::make_pair(4, ARR(0., 0, 0)),
    std::make_pair(5, ARR(-0.1, 0.05, 0.05))
};

private:
  ors::Transformation fixed_alvar;
  std::unordered_map<uint, ors::Transformation> joint_origins;
  std::unordered_map<uint, ors::Transformation> joint_positions;
  void readJointPositions();
};
