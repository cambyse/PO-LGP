#pragma once

#include <Core/thread.h>

enum ActStatus { AS_init=-1, AS_running, AS_done, AS_converged, AS_stalled };

struct Roopi;
struct Act;
typedef mlr::Array<Act*> ActL;

struct Act : ConditionVariable{
  Roopi& roopi;
  double startTime;

  Act(Roopi *r);
  virtual ~Act();

  double time();
};

