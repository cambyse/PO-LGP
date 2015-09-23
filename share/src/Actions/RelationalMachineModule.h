#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>

struct RelationalMachineModule : Module{
  ACCESS(MT::String, effects)
  ACCESS(ActivityL, A)
  ACCESS(MT::String, state)
  ACCESS(RelationalMachine, RM)

  Log _log;

  RelationalMachineModule(ModuleL& S=NoModuleL);
  ~RelationalMachineModule();

  /// @name module implementations
  void open();
  void step();
  void close();
};
