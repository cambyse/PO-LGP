#pragma once

#include <Core/module.h>
#include "data.h"

/*
struct G4DataStruct {
  floatA poses;
  struct timespec timestamp;

  friend std::istream &operator>>(std::istream &is, G4DataStruct &g4d);
  friend std::ostream &operator<<(std::ostream &os, G4DataStruct &g4d);
};
*/

typedef DataStruct<floatA> G4DataStruct;

struct G4Poller: Module {
  ACCESS(G4DataStruct, g4data)

  G4Poller();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Poller *s;
};
