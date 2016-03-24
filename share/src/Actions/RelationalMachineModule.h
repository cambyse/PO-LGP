#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>
#include <pr2/roscom.h>

struct RelationalMachineModule : Module{
  ACCESSlisten(mlr::String, effects)
  ACCESSnew(ActivityL, A)
  ACCESSnew(mlr::String, state)
  ACCESSnew(StringA, symbols)
  ACCESSnew(RelationalMachine, RM)

  Log _log;

  RelationalMachineModule();
  ~RelationalMachineModule();

  void open();
  void step();
  void close();
};

