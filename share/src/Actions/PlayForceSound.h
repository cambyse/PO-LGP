#pragma once

#include "activity.h"
#include <pr2/roscom.h>

// ============================================================================

struct PlayForceSoundActivity : Activity, Thread {
  ACCESSname(CtrlMsg, ctrl_obs)
//  ACCESSname(mlr::String, effects)

  PlayForceSoundActivity();
  virtual ~PlayForceSoundActivity();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};
