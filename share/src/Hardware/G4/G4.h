#pragma once

#include <Core/module.h>

typedef struct _G4_FRAMEDATA G4_FRAMEDATA;

struct G4Poller : Module{
  ACCESS(floatA, currentPoses)
  ACCESS(floatA, buffer)

  G4Poller();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Poller *s;
};
