#pragma once

#include "activity.h"

// ============================================================================

struct PlayFunnySoundActivity : Activity, Thread {
  PlayFunnySoundActivity();
  virtual ~PlayFunnySoundActivity();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
