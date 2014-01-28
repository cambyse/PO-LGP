#pragma once

#include <Core/module.h>
#include <Hardware/G4/G4.h>

struct G4Display: Module{
  ACCESS(floatA, poses);

  G4Display();

  virtual void open();
  virtual void close();
  virtual void step();

  struct sG4Display *s;
};
