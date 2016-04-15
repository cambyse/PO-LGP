#pragma once

#include <Core/module.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>

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

  //'scripting' interfaces
  ConditionVariable stopWaiting;
  void newSymbol(const char* symbol);
  void setFact(const char* fact);
  void waitForCondition(const char* query);
  void runScript(const char* filename);
};


