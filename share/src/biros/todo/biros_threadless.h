#ifndef THREADLESS_H
#define THREADLESS_H

#include <System/biros.h>

void loopSerialized(const ProcessL& P);

struct Threadless {
  struct sThreadless *s;
  Threadless();
  const int nextProcess();
};

#endif
