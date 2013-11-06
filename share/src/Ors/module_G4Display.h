#pragma once

#include <Core/module.h>

struct G4Display: Module{
  ACCESS(floatA, currentPoses)

  G4Display();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Display *s;
};
