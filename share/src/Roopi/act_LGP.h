#pragma once

#include "act.h"

namespace mlr{
  struct KinematicWorld;
}
struct FOL_World;
struct OptLGP;

struct Act_LGP : Act {
  struct sAct_LGP *s;

  Act_LGP(Roopi *r);
  virtual ~Act_LGP();

  void setKinematics(const char* kinFile);
  void setLogic(const char* folFile);
  void fixLogicSequence(const mlr::String& seq);

  mlr::KinematicWorld& kin();
  FOL_World& fol();

  void start();
  void stop();

  typedef std::shared_ptr<Act_LGP> Ptr;
};
