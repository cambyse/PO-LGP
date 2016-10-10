#pragma once

#include <Core/thread.h>

struct G4Poller: Thread{
  ACCESS(floatA, g4_poses)

  G4Poller();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Poller *s;
};
