#pragma once

#include "act.h"

struct Act_Recorder : Act{
  struct sAct_Recorder *s;
  Act_Recorder(Roopi *r, const char* var_name, uint subSample=1);
  ~Act_Recorder();
};
