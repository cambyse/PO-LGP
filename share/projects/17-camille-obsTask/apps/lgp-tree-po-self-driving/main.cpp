#include <functional>
#include <list>

#include <policy.h>
#include <policy_printer.h>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <iterative_deepening.h>
#include <graph_search.h>

#include <komo_planner.h>

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

static void savePolicyToFile( const Policy::ptr & policy )
{
  std::stringstream namess, skenamess;
  namess << "policy-" << policy->id() << ".gv";
  skenamess << "policy-" << policy->id() << ".ske";
  auto skename = skenamess.str();
  auto name = namess.str();

  // save full policy
//  {
//    std::ofstream file;
//    file.open( skename );
//    policy->save( file );
//    file.close();
//  }
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
    komo->setPrefixSetup();

    // road bounds
    komo->setTask( 0.0, -1, new AxisBound( "car_ego", -0.15, AxisBound::Y, AxisBound::MIN ), OT_ineq );
    komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.15, AxisBound::Y, AxisBound::MAX ), OT_ineq );

    // min speed
    komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

    // truck speed
    arr truck_speed{ 0.03, 0, 0 };
    truck_speed( 0 ) = 0.03;
    komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
    //komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );
    //komo->setVelocity( 0.0, -1, "truck", NULL, OT_eq, truck_speed );


    // opposite car speed
    arr op_speed{ -0.03, 0, 0 };
    op_speed( 0 ) = -0.03;
    komo->setVelocity( 0.0, -1, "car_op", NULL, OT_eq, op_speed );

    komo->activateCollisions( "car_ego", "truck" );
    komo->activateCollisions( "car_ego", "car_op" );

    // min speed
    komo->setTask( 0.0, 1.0, new AxisBound( "car_ego", -0.1, AxisBound::Y, AxisBound::MAX ), OT_sumOfSqr );
    komo->setTask( 0.0, -1, new AxisBound( "car_ego",  0.00, AxisBound::X, AxisBound::MIN ), OT_ineq, - arr{ 0.03 }, 1e2, 1 );

    // collision
    komo->activateCollisions( "car_ego", "truck" );
    komo->activateCollisions( "car_ego", "car_op" );
    komo->setCollisions( true );
  }
}

void groundLook( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();
  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  // look
  komo->setTask( t_start + 0.9, t_end, new AxisBound( "car_ego", 0.0, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " look " <<*symbols(0) << " at " << *symbols(1) << std::endl;
  }
}

void groundOvertake( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();
  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  // overtake
  //komo->setTask( t_start -0.5, t_start + 0.5, new AxisBound( "car_ego", 0.05, AxisBound::Y, AxisBound::MIN ), OT_sumOfSqr );
  komo->setPosition( t_end, -1, "car_ego", *symbols(0), OT_sumOfSqr, { 0.6, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " overtake " <<*symbols(0) << std::endl;
  }
}

void groundFollow( double phase, const Graph& facts, Node *n, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();
  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  // overtake
  komo->setPosition( t_end, -1, "car_ego", "truck", OT_sumOfSqr, { -0.55, 0, 0 } );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " follow " <<*symbols(0) << " at " << *symbols(1) << std::endl;
  }
}

//===========================================================================

void plan_graph_search()
{
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // set planner specific parameters
  mp->setNSteps( 20 );

  // register symbols
  mp->registerTask( "komoLook"      , groundLook );
  mp->registerTask( "komoOvertake"  , groundOvertake );
  mp->registerTask( "komoFollow"    , groundFollow );


  // set start configurations
  //mp->setKin( "LGP-overtaking-kin-1w.g" );
  //tp->setFol( "LGP-overtaking-1w.g" );
  mp->setAgentFrames( { "car_ego" } );
  mp->setKin( "LGP-overtaking-kin-2w.g" );
  tp->setFol( "LGP-overtaking-2w.g" );
  //mp->setKin( "LGP-overtaking-kin-3w.g" );
  //tp->setFol( "LGP-overtaking-3w.g" );

  /// TASK PLANNING
  tp->solve();
  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );

  auto policy = tp->getPolicy();
  auto po     = tp->getPlanningOrder();

  // save policy
  savePolicyToFile( policy );

  /// MOTION PLANNING
  //mp->solveAndInform( po, policy );

  // print resulting cost
  //std::cout << "cost of the policy " << " " << policy->cost() << std::endl;

  mp->display( policy, 3000 );

//  tp->integrate( policy );

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
