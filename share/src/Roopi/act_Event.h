#pragma once

#include "act.h"

typedef std::function<int(const SignalerL&, const intA&, int whoChanged)> EventFunction;

struct Act_Event : Act{
  const SignalerL signalers;
  EventFunction event;
  intA signalersStates;

  Act_Event(Roopi *r, const SignalerL& signalers, const EventFunction& eventFct);
  ~Act_Event();

  void callback(Signaler *s, int status);
  void selfCallback(Signaler *s, int status);
};
