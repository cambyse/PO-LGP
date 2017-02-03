#pragma once

#include <Core/thread.h>

enum ActStatus { AS_preStart=-1, AS_running, AS_done, AS_stalled, AS_converged };

struct Act;
typedef mlr::Array<Act*> ActL;

struct Act{
  double startTime;
  ConditionVariable status;

  Act();
  virtual ~Act(){}

  virtual void start() = 0;
  virtual void stop() = 0;
  virtual ActStatus getStatus(){ return (ActStatus)status.getValue(); }


  double time();
};

