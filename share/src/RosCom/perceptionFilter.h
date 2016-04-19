#pragma once

#include <Core/module.h>
#include <Core/array.h>
#include <RosCom/roscom.h>

#include <Algo/hungarian.h>
#include <RosCom/filterObject.h>

struct Filter : Module{
  ACCESSname(FilterObjects, perceptual_inputs)
  ACCESSname(FilterObjects, object_database)

  double relevance_decay_factor = 0.9;
  double relevance_threshold = 0.25;
  double distance_threshold = 0.5;

  uint maxId = 0;

  Hungarian *ha;
  arr costs;

  Filter();

  ros::NodeHandle* nh;
  ros::Publisher tabletop_pub;
  ros::Publisher alvar_pub;

  virtual void open();
  virtual void step();
  virtual void close();

  arr createCostMatrix(const FilterObjects& perceptualInputs, const FilterObjects& objectDatabase);
  FilterObjects assign(const FilterObjects& perceps, const FilterObjects& database);
};

//void filterStep(const FilterObjects& perceptualInputs, FilterObjects& objectDatabase);
