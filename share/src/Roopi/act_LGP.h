#pragma once

#include "act.h"

namespace mlr{
  struct KinematicWorld;
}

struct Act_LGP : Act {
  struct sAct_LGP *s;

  Act_LGP(Roopi *r);
  virtual ~Act_LGP();

  void setKinematics(const char* kinFile);
  void setLogic(const char* folFile);
  void fixLogicSequence(const mlr::String& seq);

  mlr::KinematicWorld& kin();
  struct FOL_World& fol();
//  struct OptLGP& opt();

  void start();
  void stop();

  typedef std::shared_ptr<Act_LGP> Ptr;
};
