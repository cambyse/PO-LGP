#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>
#include <RosCom/roscom.h>

struct RelationalMachineModule : Module{
  ACCESSlisten(mlr::String, effects)
  ACCESS(ActivityL, A)
  ACCESS(mlr::String, state)
  ACCESS(StringA, symbols)
  ACCESS(RelationalMachine, RM)

  Log _log;

  RelationalMachineModule();
  ~RelationalMachineModule();

  void open();
  void step();
  void close();
};

