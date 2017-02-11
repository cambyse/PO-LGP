#pragma once
#include "act.h"

struct Act_Tweets : Act{
  struct sAct_Tweets *s;
  Act_Tweets(Roopi *r);
  ~Act_Tweets();

  void registerAct(Act*);
  void deregisterAct(Act*);
};
