#pragma once

#include "act.h"

typedef std::function<int()> Script;

struct Act_Script : Act, Thread{
  Script script;
  Act_Script(Roopi *r, const Script& S, double beatIntervalSec=-1.);
  ~Act_Script();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
