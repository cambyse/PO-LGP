#include <komo_wrapper.h>

#include "komo_groundings.h"

#include <Kin/switch.h>
#include <Kin/TM_angVel.h>
#include <Kin/TM_default.h>
#include <Kin/TM_gravity.h>
#include <Kin/TM_InsideBox.h>
#include <Kin/TM_AboveBox.h>
#include <Kin/TM_qLimits.h>

#include <observation_tasks.h>

using namespace rai;

double shapeSize(const KinematicWorld& K, const char* name, uint i=2);

void groundInit( KOMO_ext * komo, int verbose )
{

}

void groundPickUp( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundUnStack(phase, facts, komo, verbose);
}

void groundUnStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  const double t_start = phase;
  const double t_end =   phase + 1.0;

  // switch
  const auto& eff = "baxterR";
  const auto& object = facts[0].c_str();

  // approach
  komo->addObjective(t_end - 0.3, t_end, new TM_InsideBox(komo->world, eff, NoVector, object, 0.04), OT_ineq, NoArr, 1e2, 0); // inside object at grasp moment

  // switch
  komo->addSwitch(t_end, true, new KinematicSwitch(SW_effJoint, JT_free, eff, object, komo->world));

  // after (stay stable)
  komo->addObjective(t_end, t_end + 2.0, new TM_ZeroQVel(komo->world, object), OT_eq, NoArr, 3e1, 1, +1, -1);
  if(komo->k_order > 1)
    komo->addObjective(t_end, t_end, new TM_LinAngVel(komo->world, object), OT_eq, NoArr, 1e1, 2, +0, +1);
  else
    komo->addObjective(t_end, t_end, new TM_NoJumpFromParent(komo->world, object), OT_eq, NoArr, 1e2, 1, 0, 0);


  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundPutDown( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  const double t_start = phase;
  const double t_end =   phase + 1.0;

  const auto& object = facts[0].c_str();
  const auto& place = facts[1].c_str();

  // approach
  komo->addObjective(t_end, t_end, new TM_AboveBox(komo->world, object, place), OT_ineq, NoArr, 1e1, 0);

  // switch
  Transformation rel{0};
  rel.pos.set(0,0, .5*(shapeSize(komo->world, place) + shapeSize(komo->world, object)));
  komo->addSwitch(t_end, true, new KinematicSwitch(SW_effJoint, JT_transXYPhi, place, object, komo->world, SWInit_zero, 0, rel));

  // after (stay stable)
  komo->addObjective(t_end, -1.0, new TM_ZeroQVel(komo->world, object), OT_eq, NoArr, 3e1, 1, +1, -1);
  if(komo->k_order > 1)
    komo->addObjective(t_end, -1.0, new TM_LinAngVel(komo->world, object), OT_eq, NoArr, 1e1, 2, +0, +1);
  else
    komo->addObjective(t_end, -1.0, new TM_NoJumpFromParent(komo->world, object), OT_eq, NoArr, 1e2, 1, 0, 0);

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundCheck( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  //
  const double t_start = phase;
  const double t_end =   phase + 1.0;
  //
  komo->addObjective( t_start + 0.5, t_end, new LimitsConstraint(0.05), OT_ineq, NoArr ); // avoid self collision with baxter
  //komo->setTask( t_start, t_end, new TM_Transition(komo->world), OT_sos, NoArr, 1e-1, 2);

  //komo->addObjective( t_start, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  komo->addObjective( t_end - 0.1, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " check " << facts[0] << std::endl;
  }
}

void groundStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPutDown(phase, facts, komo, verbose);
}

//const double fixEffJointsWeight = 1e3;

//komo->setFixEffectiveJoints(-1., -1., fixEffJointsWeight );
//komo->setFixSwitchedObjects();

//komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
//komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

/*
// down before pick
//komo->setTask(t_start, t_start+.20, new TM_Default(TMT_pos, komo->world, "baxterR"), OT_sos, {0.,0.,-.1}, 1e1, 1); //move down

//disconnect object from table
//komo->setKinematicSwitch( t_end, true, "delete", facts[1].c_str(), facts[0].c_str() );
//connect graspRef with object
//komo->setKinematicSwitch( t_end, true, "ballZero", "baxterR", facts[0].c_str() );

// lift after pick
//komo->setTask(t_end+.10, t_end+.10, new TM_Default(TMT_pos, komo->world,"baxterR"), OT_sos, {0.,0.,+.1}, 1e1, 1); //move up
*/

//  const double radius = 0.25;

  // hacky collision avoidance
//  std::vector<std::string> blocks{"block_1", "block_2", "block_3"};

//  for(const auto & fixed_block: blocks)
//  {
//    if(facts[0] != fixed_block)
//    {
//      komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), fixed_block.c_str(), radius ), OT_ineq );
//    }
//  }

//  if( facts[0] != "block_1" )
//    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_1", radius ), OT_ineq );

//  if( facts[0] != "block_2" )
//    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_2", radius ), OT_ineq );

//  if( facts[0] != "block_3" )
//    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_3", radius ), OT_ineq );

//  if( facts[0] != "block_4" )
//    komo->setTask( t_end - 0.5, t_end, new ApproxPointToShape( komo->world, facts[0].c_str(), "block_4", radius ), OT_ineq );

//  if(t_end == 7.0 )
//    int a = 0;
//  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

