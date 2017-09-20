#pragma once

#include "act.h"

struct Act_ComBaxter : Act {
  struct sAct_ComBaxter *s;

  Act_ComBaxter(Roopi *r);
  virtual ~Act_ComBaxter();

  typedef std::shared_ptr<Act_ComBaxter> Ptr;
};
