#pragma once
#include "act.h"

struct Act_PhysX : Act{
  struct sAct_PhysX *s;
  Act_PhysX(Roopi *r);
  ~Act_PhysX();
};
