#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <Ors/ors.h>
#include <RosCom/filterObject.h>

//#include <Control/TaskControllerModule.h>
//#include <RosCom/baxter.h>
//#include <RosCom/spinner.h>
//#include <Ors/orsviewer.h>
//#include <RosCom/subscribeAlvarMarkers.h>
//#include <RosCom/perceptionCollection.h>
//#include <RosCom/perceptionFilter.h>
//#include <RosCom/publishDatabase.h>
#include <unordered_map>


struct Lockbox{
  Lockbox(){}
  ~Lockbox(){}

  ACCESSname(FilterObjects, object_database)

//  TaskControllerModule tcm;

//  RosInit rosInit;
//  SubscribeAlvar alvar_subscriber;
//  Collector data_collector;
//  Filter myFilter;
//  PublishDatabase myPublisher;

//  OrsViewer view;
//  OrsPoseViewer ctrlView;
//  RosCom_Spinner spinner; //the spinner MUST come last: otherwise, during closing of all, it is closed before others that need messages

  std::unordered_map<uint, ors::Transformation> joint_origins;
  std::unordered_map<uint, ors::Transformation> joint_positions;

  bool getJointTransform(const uint joint, ors::Transformation& diff);
  bool getJointPosition(const uint joint, double position);

  ors::Transformation fixed_alvar;

  void readJointPositions();
};

