#include <functional>
#include <list>

#include <policy.h>
#include <policy_printer.h>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <iterative_deepening.h>
#include <graph_search.h>

#include <komo_planner.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <vertical_velocity.h>
#include <axis_alignment.h>
#include <over_plane.h>

#include <node_visitors.h>

/*
sort nodes before expanding?
dot -Tpng -o policy.png policy.gv

test a logic : mlr/share/example/DomainPlayer

QUESTIONS  :
- why no proxy?
- how to solve collision avoidance if objects penetrate
- kinematic switches ( commented part of the code ) in solvePath, solvePose, etc..

TODO :
=> constraints are difficult to evaluate with collision avoidance, mybe need a refactoring as in 3/
2/ symbolic search, use costs from other levels? -> How to inform?           | 1
5/ collision avoidance, rule for proxy ?, get out of a collision             | 2
*/
//===========================================================================


static void generatePngImage( const std::string & name )
{
  std::string nameCopy( name );
  const std::string ext( ".gv" );
  std::string newName = nameCopy.replace( nameCopy.find( ext ), ext.length(), ".png" );

  std::stringstream ss;
  ss << "dot"   << " ";
  ss << "-Tpng" << " ";
  ss << "-o"    << " ";
  ss << newName << " ";
  ss << name;

  system( ss.str().c_str() );
}

static void savePolicyToFile( const Policy::ptr & policy )
{
  std::stringstream namess, skenamess;
  namess << "policy-" << policy->id() << ".gv";
  skenamess << "policy-" << policy->id() << ".ske";
  auto skename = skenamess.str();
  auto name = namess.str();

  // save full policy
  {
    std::ofstream file;
    file.open( skename );
    policy->save( file );
    file.close();
  }
  // generate nice graph
  {
    std::ofstream file;
    file.open( name );
    PolicyPrinter printer( file );
    printer.print( policy );
    file.close();

    generatePngImage( name );
  }
}

//==========Application specific grounders===================================
void groundPickUp( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", "tableC", *symbols(0)  );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "handL", *symbols(0)  );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": pick up " << *symbols(0) << " from " << *symbols(1) << std::endl;
  }
}

void groundUnStack( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", *symbols(1), *symbols(0)  );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "handL", *symbols(0)  );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": unstack " << *symbols(0) << " from " << *symbols(1) << std::endl;
  }
}

void groundPutDown( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  komo->setPlace( t_end, "handL", *symbols(0), *symbols(1), verbose );

//  if( *symbols(1) == "tableC_center" )
//  {
//    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
//  }
//  else if( *symbols(1) == "tableC_left" )
//  {
//    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
//  }
//  else if( *symbols(1) == "tableC_right" )
//  {
//    komo->setPlace( t_end, "handL", *symbols(0), "tableC", verbose );
//  }
//  else
//  {
//    CHECK( 0 , "" );
//  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " put down " <<*symbols(0) << " at " << *symbols(1) << std::endl;
  }
}

void groundCheck( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase+0.5;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;
  komo->setTask( t_start, t_end, new ActiveGetSight( "manhead", *symbols(0), ARR( 0, -0.05, 0 ), ARR( 0, -1, 0 ), 0.5 ), OT_sumOfSqr, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " check " << *symbols(0) << std::endl;
  }
}

void groundStack( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  komo->setPlace( t_end, "handL", *symbols(0), *symbols(1), verbose );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " stack " <<*symbols(0) << " on " << *symbols(1) << std::endl;
  }
}

//===========================================================================

void plan_mcts()
{
  // instanciate planners
  auto tp = std::make_shared< tp::MCTSPlanner >();
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // set planner specific parameters
  tp->setMCParams( /*10*/100, -1, /*50*/100 );
  //tp->setMCParams( 50, -1, 50 );
  mp->setNSteps( 10 );

  // register symbols
  mp->registerTask( "komoPickUp"       , groundPickUp );
  mp->registerTask( "komoPutDown"      , groundPutDown );
  mp->registerTask( "komoCheck"        , groundCheck );
  mp->registerTask( "komoStack"        , groundStack );
  mp->registerTask( "komoUnStack"      , groundUnStack );

  // set start configurations
  //tp->setFol( "LGP-blocks-fol.g" );
  //mp->setKin( "LGP-blocks-kin.g" );

  tp->setFol( "LGP-blocks-fol-2w.g" );
  mp->setKin( "LGP-blocks-kin-2w.g" );


  for( uint i = 0; ! tp->terminated() && i < 1 ; ++i )
  {
    std::cout << "Task planning to generate " << i << "th policy" << std::endl;

    // TASK PLANNING
    tp->solve();
    auto policy = tp->getPolicy();
    auto po     = tp->getPlanningOrder();

    // save policy
    savePolicyToFile( policy );

    //-------------------------------------------------------------------
//    for( auto j = 0; j < 10; ++j )
//    {
//      tp->solve();
//      policy = tp->getPolicy();

//      // save policy
//      savePolicyToFile( policy );
//    }
    //-------------------------------------------------------------------

    std::cout << "Motion Planning for policy " << i << std::endl;

    // MOTION PLANNING
    mp->solveAndInform( po, policy );

    // print resulting cost
    std::cout << "cost of the policy " << i << " " << policy->cost() << std::endl;

    tp->integrate( policy );
  }

  std::cout << "best policy:" << tp->getPolicy()->id() << std::endl;
  mp->display( tp->getPolicy(), 3000 );
}

//===========================================================================

void plan_iterative_deepening()
{
  // instanciate planners
  auto tp = std::make_shared< tp::IterativeDeepeningPlanner >();

  tp->setDmax( 12 );

  //tp->setFol( "LGP-blocks-fol-2w.g" );
  tp->setFol( "LGP-blocks-fol.g" );


  tp->solve();

  mlr::wait( 30, true );
}

//===========================================================================

void plan_graph_search()
{
  auto tp = std::make_shared< tp::GraphSearchPlanner >();

  tp->setFol( "LGP-blocks-fol-easy-2w.g" );
  //tp->setFol( "LGP-blocks-fol.g" );

  tp->solve();

  mlr::wait( 30, true );
}

//===========================================================================

int main(int argc,char **argv)
{
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan_graph_search();
  //plan_iterative_deepening();
  //plan_mcts();

  return 0;
}
