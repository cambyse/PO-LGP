#pragma once

#include "activity.h"
//#include <pr2/roscom.h>

// ============================================================================

struct PlayForceSoundActivity : Activity, Thread {
  ACCESSname(arr, Fl)
  ACCESSname(arr, Fr)
//  ACCESSname(mlr::String, effects)

  PlayForceSoundActivity();
  virtual ~PlayForceSoundActivity();

  virtual void open();
  virtual void step();
  virtual void close(){}
};
