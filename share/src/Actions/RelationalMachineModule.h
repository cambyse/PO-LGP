#pragma once

#include <Core/thread.h>
#include <FOL/relationalMachine.h>
#include <Actions/activity.h>

struct RelationalMachineModule : Thread {
  ACCESSlisten(mlr::String, effects)
  ACCESS(ActivityL, A)
  ACCESS(mlr::String, state)
  ACCESS(StringA, symbols)
  ACCESS(RelationalMachine, RM)

  mlr::LogObject _log;

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


