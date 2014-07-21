#pragma once

#include <Core/module.h>

struct G4Poller: Module {
  ACCESS(floatA, poses)

  G4Poller();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Poller *s;
};
