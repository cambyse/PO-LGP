#pragma once

#include <KOMO/komo-ext.h>
#include <list>
#include <tree_builder.h>

namespace mp
{

//komo wrapper
struct W
{
  W(KOMO_ext * komo)
    : komo_(komo)
    , world_(komo->world)
  {

  }
  void reset(const std::list<Vars>& branches, double initNoise=0.01);
  void setupConfigurations(const std::list<Vars>& branches);
  void addObjective(const Interval& it, const TreeBuilder& tb, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0);
  void addSwitch_stable(const Interval& it, const TreeBuilder& tb, const char* from, const char* to);
  void addSwitch_stableOn(const Interval& it, const TreeBuilder& tb, const char* from, const char* to);

  KOMO_ext * komo_;
  const rai::KinematicWorld & world_;
};
}
