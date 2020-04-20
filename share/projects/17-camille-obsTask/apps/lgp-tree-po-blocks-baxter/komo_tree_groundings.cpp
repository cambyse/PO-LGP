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

void groundTreePickUp(double start, const mp::Vars& branch, const arr& scales, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const double end = start + 1.0;

  groundTreeUnStack(start, branch, scales, facts, komo, verbose);

  //  if( verbose > 0 )
  //  {
  //    std::cout << "from: " << start << " -> " << end << " pick up " << facts[0] << " from " << facts[1] << std::endl;
  //  }
}

void groundTreeUnStack(double start, const mp::Vars& branch, const arr& scales, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const double end = start + 1.0;

  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();
  mp::W(komo).addSwitch_stable(end, branch, eff, object);

  // costs

  //(double startTime, double endTime, const char* shape, const arr& whichAxis, const char* shapeRel, const arr& whichAxisRel, ObjectiveType type, const arr& target, double prec) {
  //komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
  mp::W(komo).addObjective(end - 0.3,end, branch, new TM_Default(TMT_vecAlign, komo->world, "baxterR", Vector(ARR( 1.0, 0.0, 0.0 )), nullptr, ARR( 0.0, 0.0, -1.0 )), OT_sos, ARR(1.), 1e2, 0);
  mp::W(komo).addObjective(start, end, branch, new TM_InsideBox(komo->world, eff, NoVector, object), OT_ineq, NoArr, 1e1, 0);

  if(verbose > 0)
  {
    std::cout << "from: " << start << " -> " << end << " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown(double start, const mp::Vars& branch, const arr& scales, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const double end = start + 1.0;
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  mp::W(komo).addObjective(end, end, branch, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);
  mp::W(komo).addSwitch_stableOn(end, branch, place, object);

  if(verbose > 0)
  {
    std::cout << "from: " << start << " -> " << end << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck(double start, const mp::Vars& branch, const arr& scales, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  const double end = start + 1.0;

  mp::W(komo).addObjective(start, end, branch, new LimitsConstraint(0.05), OT_ineq, NoArr, -1, 0);

  //mp::W(komo).addObjective(start, end, branch, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2,0 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  mp::W(komo).addObjective(end-0.1, end, branch, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2, 0 );

  if(verbose > 0)
  {
    std::cout << "from: " << start << " -> " << end << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack(double start, const mp::Vars& branch, const arr& scales, const std::vector<std::string>& facts, KOMO_ext* komo, int verbose)
{
  groundTreePutDown(start, branch, scales, facts, komo, verbose);
  //  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  //  if( verbose > 0 )
  //  {
  //    std::cout << " stack " << facts[0] << " on " << facts[1] << std::endl;
  //  }
}
