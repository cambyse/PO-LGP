#pragma one

#include <KOMO/komo-ext.h>
#include <komo_factory.h>
#include <list>

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
  void addObjective(double start, double end, const mp::Vars & vars, Feature* map, ObjectiveType type=OT_sos, const arr& target=NoArr, double scale=-1., int order=-1, int deltaFromStep=0, int deltaToStep=0);
  void addSwitch_stable(double time, const mp::Vars & branch, const char* from, const char* to);
  void addSwitch_stableOn(double time, const mp::Vars & branch, const char* from, const char* to);

  KOMO_ext * komo_;
  const rai::KinematicWorld & world_;
};
}
