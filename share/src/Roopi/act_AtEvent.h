#pragma once

#include "act.h"
#include "act_Event.h"

typedef std::function<int()> Script;

struct Act_AtEvent : Act, Thread {
  Act::Ptr event;
  Script script;

  Act_AtEvent(Roopi *r, const Act::Ptr& event, const Script& script);
  ~Act_AtEvent();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
