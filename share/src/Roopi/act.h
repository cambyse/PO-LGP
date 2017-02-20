#pragma once

#include <Core/thread.h>

enum ActStatus { AS_create=-1, AS_running, AS_done, AS_stalled, AS_converged };

struct Roopi;
struct Act;
typedef mlr::Array<Act*> ActL;

struct Act : ConditionVariable{
  Roopi& roopi;
  double startTime;

  Act(Roopi *r);
//  Act(Act&& a); //move constructor
  virtual ~Act();

//  virtual ActStatus getStatus(){ return (ActStatus)getStatus(); }

  double time();
};

