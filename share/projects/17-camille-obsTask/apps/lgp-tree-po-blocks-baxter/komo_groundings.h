#pragma one

#include <graph_planner.h>

//==========Application specific grounders===================================
void groundPrefixIfNeeded( KOMO_ext * komo, int verbose  )
{
  //  if( ! komo->isPrefixSetup() )
  //  {
  //    komo->setHoming( -1., -1., 1e-2 ); //gradient bug??
  //    komo->setTask( -1, -1, new AxisBound( "manhead", 1.4, AxisBound::Z, AxisBound::MIN ), OT_ineq, NoArr, 1e2 );
  //  }
}

void groundPickUp( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  //disconnect object from table
//  komo->setKinematicSwitch( t_end, true, "delete", "tableC", facts[0].c_str() );
//  //connect graspRef with object
//  komo->setKinematicSwitch( t_end, true, "ballZero", "baxterR", facts[0].c_str() );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": pick up " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundUnStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  komo->setAlign( t_end - 0.3, t_end, "baxterR", ARR( 1.0, 0.0, 0.0 ), nullptr, ARR( 0.0, 0.0, -1.0 ) );
  komo->setGrasp( t_end, "baxterR", facts[0].c_str(), 0 );

  //  // down before pick
  //komo->setTask(t_start, t_start+.20, new TM_Default(TMT_pos, komo->world, "baxterR"), OT_sos, {0.,0.,-.1}, 1e1, 1); //move down

  //disconnect object from table
  //komo->setKinematicSwitch( t_end, true, "delete", facts[1].c_str(), facts[0].c_str() );
  //connect graspRef with object
  //komo->setKinematicSwitch( t_end, true, "ballZero", "baxterR", facts[0].c_str() );

  //  // lift after pick
  //komo->setTask(t_end+.10, t_end+.10, new TM_Default(TMT_pos, komo->world,"baxterR"), OT_sos, {0.,0.,+.1}, 1e1, 1); //move up

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundPutDown( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  const double radius = 0.25;

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

  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundCheck( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase + 0.5;
  const double t_end =   phase + duration;
  //
  komo->addObjective( t_start, t_end, new LimitsConstraint(0.05), OT_ineq, NoArr ); // avoid self collision with baxter
  //komo->setTask( t_start, t_end, new TM_Transition(komo->world), OT_sos, NoArr, 1e-1, 2);

  komo->addObjective( t_start, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2 ); // slight offset (0.01) to break symmetry and avoid quternion normalization problem
  komo->addObjective( t_end - 0.1, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0.01, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " check " << facts[0] << std::endl;
  }
}

void groundStack( double phase, const std::vector< std::string >& facts, KOMO_ext * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  komo->setPlace( t_end, "baxterR", facts[0].c_str(), facts[1].c_str(), verbose );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " stack " << facts[0] << " on " << facts[1] << std::endl;
  }
}
