#pragma once

#include "act.h"

typedef std::function<bool(const ConditionVariableL&, const intA&, int whoChanged)> EventBoolean;

struct Act_Event : Act{
  const ConditionVariableL signalers;
  EventBoolean event;
  intA statuses;

  Act_Event(Roopi *r, const ConditionVariableL& signalers, const EventBoolean& event);
  ~Act_Event();

  void callback(ConditionVariable *s, int status);
  void selfCallback(ConditionVariable *s, int status);
};
