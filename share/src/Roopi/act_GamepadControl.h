#pragma once

#include "act.h"

// ============================================================================

struct Act_GamepadControl : Act {
  struct sAct_GamepadControl *s;

  Act_GamepadControl(Roopi *r);
  virtual ~Act_GamepadControl();
};
