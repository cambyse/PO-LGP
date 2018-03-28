#include <functional>
#include <list>

#include <chrono>

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
#include <axis_bound.h>
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

static void savePolicyToFile( const Policy::ptr & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;
  namess << "policy-" << policy->id() << suffix << ".gv";
  auto name = namess.str();

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
void groundPrefixIfNeeded( mp::ExtensibleKOMO * komo, int verbose  )
{
  if( ! komo->isPrefixSetup() )
  {
    komo->setHoming( -1., -1., 1e-2 ); //gradient bug??
    komo->setTask( -1, -1, new AxisBound( "manhead", 1.4, AxisBound::Z, AxisBound::MIN ), OT_ineq, NoArr, 1e2 );
  }
}

void groundPickUp( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

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

void groundUnStack( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

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

void groundPutDown( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  komo->setPlace( t_end, "handL", *symbols(0), *symbols(1), verbose );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " put down " <<*symbols(0) << " at " << *symbols(1) << std::endl;
  }
}

void groundCheck( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase + 0.5;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;
  komo->setTask( t_start, t_end, new ActiveGetSight( "manhead", *symbols(0), ARR( 0, -0.05, 0 ), ARR( 0, -1, 0 ), 0.65 ), OT_sumOfSqr, NoArr, 1e2 );
  komo->setTask( t_end - 0.1, t_end, new ActiveGetSight( "manhead", *symbols(0), ARR( 0, -0.05, 0 ), ARR( 0, -1, 0 ), 0.65 ), OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " check " << *symbols(0) << std::endl;
  }
}

void groundStack( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

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
    std::cout << "cost of the policy " << i << " " << policy->value() << std::endl;

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
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // set planner specific parameters
  tp->setInitialReward( -0.1 );
  mp->setNSteps( 5 );

  // register symbols
  mp->registerTask( "komoPickUp"       , groundPickUp );
  mp->registerTask( "komoPutDown"      , groundPutDown );
  mp->registerTask( "komoCheck"        , groundCheck );
  mp->registerTask( "komoStack"        , groundStack );
  mp->registerTask( "komoUnStack"      , groundUnStack );

  // set start configurations
  //tp->setFol( "LGP-blocks-fol.g" );
  //mp->setKin( "LGP-blocks-kin.g" );

  //tp->setFol( "LGP-blocks-fol-model-2.g" );
  //mp->setKin( "LGP-blocks-kin.g" );

  //tp->setFol( "LGP-blocks-fol-2w-model-2.g" );
  //mp->setKin( "LGP-blocks-kin-2w.g" );

  //tp->setFol( "LGP-blocks-fol-2w.g" );
  //mp->setKin( "LGP-blocks-kin-2w.g" );

  //tp->setFol( "LGP-blocks-fol-2w-model-2.g" );
  //mp->setKin( "LGP-blocks-kin-2w.g" );

  //tp->setFol( "LGP-blocks-fol-1w.g" );
  //mp->setKin( "LGP-blocks-kin-1w.g" );

  tp->setFol( "LGP-blocks-fol-1w-model-2.g" );
  mp->setKin( "LGP-blocks-kin-1w.g" );

///
  std::ofstream candidate, results;
  candidate.open( "policy-candidates.data" );
  results.open( "policy-results.data" );
  double graph_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;
///

{
auto start = std::chrono::high_resolution_clock::now();
  /// GRAPH BUILDING
  tp->buildGraph();
auto elapsed = std::chrono::high_resolution_clock::now() - start;
graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
  tp->saveGraphToFile( "graph.gv" );
  //generatePngImage( "graph.gv" );

  Policy::ptr policy, lastPolicy;

{
auto start = std::chrono::high_resolution_clock::now();
  tp->solve();
auto elapsed = std::chrono::high_resolution_clock::now() - start;
task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
  policy = tp->getPolicy();

  do
  {
    ///
    savePolicyToFile( policy );
    candidate << policy->id() << "," << std::max( -10.0, policy->value() ) << std::endl;
    ///

    lastPolicy = policy;

{
auto start = std::chrono::high_resolution_clock::now();
    /// MOTION PLANNING
    auto po     = tp->getPlanningOrder();
    mp->solveAndInform( po, policy );
auto elapsed = std::chrono::high_resolution_clock::now() - start;
motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
    ///
    savePolicyToFile( policy, "-informed" );
    results << policy->id() << "," << std::max( -10.0, policy->value() ) << std::endl;
    ///

{
auto start = std::chrono::high_resolution_clock::now();
    /// TASK PLANNING
    tp->integrate( policy );
    tp->solve();
auto elapsed = std::chrono::high_resolution_clock::now() - start;
task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
    policy = tp->getPolicy();

  } while( ! skeletonEquals( lastPolicy, policy ) );

///
  savePolicyToFile( policy, "-final" );
  candidate << policy->id() << "," << std::max( -10.0, policy->value() ) << std::endl;
  results << policy->id() << "," << std::max( -10.0, policy->value() ) << std::endl;

  candidate.close();
  results.close();
///

{
auto start = std::chrono::high_resolution_clock::now();
  mp->display( policy, 3000 );
auto elapsed = std::chrono::high_resolution_clock::now() - start;
joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}

///
  std::ofstream timings;
  timings.open("timings.data");
  timings << "graph_building_s="<< graph_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< graph_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();
///

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
