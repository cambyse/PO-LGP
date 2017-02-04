#pragma once

#include <Core/thread.h>

enum ActStatus { AS_preStart=-1, AS_running, AS_done, AS_stalled, AS_converged };

struct Act;
typedef mlr::Array<Act*> ActL;

struct Act{
  struct Roopi *roopi;
  double startTime;
  ConditionVariable status;

  Act(Roopi *r);
  virtual ~Act(){}

  virtual ActStatus getStatus(){ return (ActStatus)status.getValue(); }

  double time();
};

