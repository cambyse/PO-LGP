#include <komo_wrapper.h>

#include "komo_tree_groundings.h"

#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>
#include <Kin/TM_qLimits.h>

#include <observation_tasks.h>

using namespace rai;

void groundTreePickUp(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreeUnStack(it, tb, facts, komo, verbose);

  //  if( verbose > 0 )
  //  {
  //    std::cout << "from: " << start << " -> " << end << " pick up " << facts[0] << " from " << facts[1] << std::endl;
  //  }
}

void groundTreeUnStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();
  mp::W(komo).addSwitch_stable({{it.time.to, it.time.to}, it.edge}, tb, eff, object);

  // costs
  //mp::W(komo).addObjective(end - 0.3,end, branch, new TM_Default(TMT_vecAlign, komo->world, "baxterR", Vector(ARR( 1.0, 0.0, 0.0 )), nullptr, ARR( 0.0, 0.0, -1.0 )), OT_sos, ARR(1.), 1e2, 0); // pb quat normalization
  mp::W(komo).addObjective({{it.time.to-0.3, it.time.to}, it.edge}, tb, new TM_InsideBox(komo->world, eff, NoVector, object, 0.04), OT_ineq, NoArr, 1e2, 0); // inside object at grasp moment

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << " -> " << it.time.to << " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  mp::Interval end{{it.time.to, it.time.to}, it.edge};
  mp::W(komo).addObjective(end, tb, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);
  mp::W(komo).addSwitch_stableOn(end, tb, place, object);

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << " -> " << it.time.to << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  mp::W(komo).addObjective(it, tb, new LimitsConstraint(0.05), OT_ineq, NoArr, 1e1, 0);

  //mp::W(komo).addObjective(start, end, branch, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2,0 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  mp::Interval end{{it.time.to-0.1, it.time.to}, it.edge};
  mp::W(komo).addObjective(end, tb, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2, 0 );

  if(verbose > 0)
  {
    std::cout << "from: " << it.time.from << " -> " << it.time.to << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack(const mp::Interval& it, const mp::TreeBuilder& tb, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(it, tb, facts, komo, verbose);
  //  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  //  if( verbose > 0 )
  //  {
  //    std::cout << " stack " << facts[0] << " on " << facts[1] << std::endl;
  //  }
}
