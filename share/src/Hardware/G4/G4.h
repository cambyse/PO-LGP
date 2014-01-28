#pragma once

#include <Core/module.h>
#include "data.h"

typedef DataStruct<floatA> G4DataStruct;

struct G4Poller: Module {
  //ACCESS(G4DataStruct, g4data)
  ACCESS(floatA, poses)

  G4Poller();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Poller *s;
};
