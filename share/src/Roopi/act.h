#pragma once

#include <Core/thread.h>

enum ActStatus { AS_init=-1, AS_running, AS_done, AS_converged, AS_stalled, AS_true, AS_false, AS_kill };

struct Roopi;
struct Act;
typedef mlr::Array<Act*> ActL;

struct Act : Signaler{
  Roopi& roopi;
  double startTime;

  Act(Roopi *r);
  virtual ~Act();

  double time();
  virtual void write(ostream& os);

  typedef std::shared_ptr<Act> Ptr;
};

