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

void groundTreePickUp( double start, const mp::Vars& branch, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  const double end = start + 1.0;

  //  double duration=1.0;

  //  //
  //  const double t_start = phase;
  //  const double t_end =   phase + duration;
  //  //
  //  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
  //  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  groundTreeUnStack(start, branch, scales, facts, komo, verbose);

//  if( verbose > 0 )
//  {
//    std::cout << "from: " << start << " -> " << end << " pick up " << facts[0] << " from " << facts[1] << std::endl;
//  }
}

void groundTreeUnStack( double start, const mp::Vars& branch, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  const double end = start + 1.0;

  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();
  mp::W(komo).addSwitch_stable(end, branch, eff, object);

  // costs
  mp::W(komo).addObjective(start, end, branch, new TM_InsideBox(komo->world, eff, NoVector, object), OT_ineq, NoArr, 1e1, 0);

  //  double duration=1.0;

  //  //
  //  const double t_start = phase;
  //  const double t_end =   phase + duration;
  //  //
  //  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
  //  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  //  // lift after pick
  //komo->setTask(t_end+.10, t_end+.10, new TM_Default(TMT_pos, komo->world,"baxterR"), OT_sos, {0.,0.,+.1}, 1e1, 1); //move up

  if( verbose > 0 )
  {
    std::cout << "from: " << start << " -> " << end << " : unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundTreePutDown( double start, const mp::Vars& branch, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  const double end = start + 1.0;
  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  mp::W(komo).addObjective(end, end, branch, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);
  mp::W(komo).addSwitch_stableOn(end, branch, place, object);

  //  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  if( verbose > 0 )
  {
    std::cout << "from: " << start << " -> " << end << " : put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundTreeCheck( double start, const mp::Vars& branch, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  const double end = start + 1.0;
  //  double duration=1.0;

  //  //
  //  const double t_start = phase + 0.5;
  //  const double t_end =   phase + duration;
  //  //
  //  komo->addObjective( t_start, t_end, new LimitsConstraint(0.05), OT_ineq, NoArr ); // avoid self collision with baxter
  //  //komo->setTask( t_start, t_end, new TM_Transition(komo->world), OT_sos, NoArr, 1e-1, 2);

  mp::W(komo).addObjective(start, end, branch, new LimitsConstraint(0.05), OT_ineq, NoArr, -1, 0);
  mp::W(komo).addObjective(start+0.2, end-0.1, branch, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2, 0 );

  if( verbose > 0 )
  {
    std::cout << "from: " << start << " -> " << end << " : check " << facts[0] << std::endl;
  }
}

void groundTreeStack( double start, const mp::Vars& branch, const arr& scales, const std::vector< std::string >& facts, KOMO_ext* komo, int verbose )
{
  //  double duration=1.0;

  //  //
  //  const double t_start = phase;
  //  const double t_end =   phase + duration;
  //  //

  groundTreePutDown(start, branch, scales, facts, komo, verbose);
  //  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

//  if( verbose > 0 )
//  {
//    std::cout << " stack " << facts[0] << " on " << facts[1] << std::endl;
//  }
}
