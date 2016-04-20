#pragma once

#include "activity.h"

// ============================================================================

struct PlayForceSoundActivity : Activity, Thread {
  ACCESSname(arr, Fl)
  ACCESSname(arr, Fr)

  PlayForceSoundActivity();
  virtual ~PlayForceSoundActivity();

  virtual void open();
  virtual void step();
  virtual void close(){}
};
