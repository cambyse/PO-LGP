#pragma once

#include "activity.h"
//#include <pr2/roscom.h>

// ============================================================================

struct PlayFunnySoundActivity : Activity, Thread {
//  ACCESSname(CtrlMsg, ctrl_obs)
//  ACCESSname(mlr::String, effects)

  PlayFunnySoundActivity();
  virtual ~PlayFunnySoundActivity();

  virtual void open(){}
  virtual void step();
  virtual void close(){}
};