#pragma once

#include <Core/module.h>

struct FloatA_Recorder: Module{
  ACCESS(floatA, x)

  FloatA_Recorder();

  virtual void open();
  virtual void close();
  virtual void step();

  MT::String varName;
  ofstream file;
};
