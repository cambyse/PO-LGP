#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>

struct RelationalMachineModule : Module{
  ACCESSnew(MT::String, effects)
  ACCESSnew(ActivityL, A)
  ACCESSnew(MT::String, state)
  ACCESSnew(RelationalMachine, RM)

  Log _log;

  RelationalMachineModule();
  ~RelationalMachineModule();

  /// @name module implementations
  void open();
  void step();
  void close();
};
