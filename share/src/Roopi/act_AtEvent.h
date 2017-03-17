#pragma once

#include "act.h"
#include "act_Event.h"

typedef std::function<int()> Script;
typedef std::function<int()> Event;

struct Act_AtEvent : Act, Thread {
  shared_ptr<Act_Event> event;
  Script script;

  Act_AtEvent(Roopi *r, shared_ptr<Act_Event>& E, const Script& S);
  ~Act_AtEvent();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
