#pragma once

#include "act.h"

struct Act_ComRos : Act {
  struct RosCom_Spinner *rosSpinner = NULL;

  Act_ComRos(Roopi *r);
  virtual ~Act_ComRos();
};
