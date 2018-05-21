#include <functional>
#include <list>

#include <policy.h>
#include <policy_printer.h>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <komo_planner.h>

#include <observation_tasks.h>
#include <approx_point_to_shape.h>
#include <vertical_velocity.h>
#include <axis_alignment.h>
#include <over_plane.h>

#include <node_visitors.h>

#include <graph_search.h>

#include <komo_planner.h>

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
  std::stringstream namess;
  namess << "policy-" << policy->id() << ".gv";
  auto name = namess.str();

  std::ofstream file;
  file.open( name );
  PolicyPrinter printer( file );
  printer.print( policy );
  file.close();

  generatePngImage( name );
}

//==========Application specific grounders===================================
//------grounders------------//
void groundGrasp( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  if( *symbols(1) == "container_0" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_0_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", *symbols(0), "container_0_left" /**symbols(1)*/ );
  }
  else if( *symbols(1) == "container_1" )
  {
    //disconnect object from table
    komo->setKinematicSwitch( t_end, true, "delete", "tableC", "container_1_bottom" );
    //connect graspRef with object
    komo->setKinematicSwitch( t_end, true, "ballZero", *symbols(0), "container_1_left" /**symbols(1)*/ );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << *symbols(1) << " with " << *symbols(0) << std::endl;
  }
}

void groundGraspObject( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_approach=phase + 0.25 * duration;
  const double t_switch =phase  + 0.5 * duration;
  //const double t_escape =phase  + 0.75 * duration;
  const double t_end =   phase  + duration;
  //

  //komo->setTask( t_approach, t_approach, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.5}), OT_sumOfSqr, NoArr, 1e2);
  //komo->setTask( t_start,    t_approach, new TaskMap_Default(posTMT,   komo->world, *symbols(0) ), OT_sumOfSqr, {0.,0.,-.2}, 1e1, 1);

  //komo->setTask( t_switch, t_end, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.1}), OT_sumOfSqr, NoArr, 1e2);
  //komo->setTask( t_switch, t_end, new TaskMap_Default(posTMT, komo->world, *symbols(0) ), OT_sumOfSqr, {0.,0.,.2}, 1e1, 1);

  // approach
  //komo->setTask( t_start, t_approach, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.4}), OT_sumOfSqr, NoArr, 1e2);
  //komo->setTask( t_approach, t_end, new VerticalVelocity( *symbols(0), { 0.0,0.0 } ), OT_eq, NoArr, 1e1, 1 );

  //disconnect object from table
  komo->setKinematicSwitch( t_switch, true, "delete", NULL, *symbols(1) );
  //connect graspRef with object
  komo->setKinematicSwitch( t_switch, true, "ballZero", *symbols(0), *symbols(1) );

  // escape
  //komo->setTask( t_escape, t_end, new TaskMap_Default(posTMT, komo->world, *symbols(0), NoVector, *symbols(1), {0.,0.,0.5}), OT_sumOfSqr, NoArr, 1e2);

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": grasping " << *symbols(1) << " with " << *symbols(0) << std::endl;
  }
}

void groundPlace( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //
  //std::cout << *symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;

  if( *symbols(1) == "container_0" )
  {
    komo->setPlace( t_end, *symbols(0), "container_0_front", *symbols(2), verbose );
  }
  else if( *symbols(1) == "container_1" )
  {
    komo->setPlace( t_end, *symbols(0), "container_1_front", *symbols(2), verbose );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " <<*symbols(0) << " place " << *symbols(1) << " on " << *symbols(2) << std::endl;
  }
}

//void groundHome( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
//{
//  double duration=n->get<double>();

//  const double t = phase+duration;

//  //komo->setHoming( t, t + 1.0, 1e-2 ); //gradient bug??
//}

void groundGetSight( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  mlr::String arg = *symbols(0);

  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
                                                                        arg,
                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                                        ARR( -0.0, 0.1, 0.4 ), ARR( 0, -1, 0 ) ),  // pivot position  in container frame
                OT_sumOfSqr, NoArr, 1e2 );

  komo->setTask( t_end-0.2, t_end, new ActiveGetSight      ( "manhead",
                                                                        arg,
                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
                                                                        ARR( -0.0, 0.1, 0.4 ), ARR( 0, -1, 0 ) ),  // pivot position  in container frame
                OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": getting sight of " << *symbols(0) << std::endl;
  }
}

void groundTakeView( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  // no movement
  auto *map = new TaskMap_Transition( komo->world );
  map->posCoeff = 0.;
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  komo->setTask( t_start, t_end, map, OT_sumOfSqr, NoArr, 1e2, 1 );

  // in sight expressed as a constraint
//  mlr::String arg = *symbols(0);
//  komo->setTask( t_start, t_end, new ActiveGetSight      ( "manhead",
//                                                                        arg,
//                                                                        //ARR( -0.0, -0.0, 0.0 ),    // object position in container frame
//                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
//                OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": taking view " << std::endl;
  }
}

//class OverPlaneConstraintManager
//{
//public:

void groundActivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;//phase + 1.0;       // hack: the grasp task lasts 1 step, so we begin one step after
  const double t_end =   phase + duration; //komo->maxPhase;
  //

  if( *symbols(0) == "container_0" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 0, 0, 1.0 ) ), OT_sumOfSqr, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_sumOfSqr, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_0", *symbols(1), 0.05 ), OT_sumOfSqr, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }
  else if( *symbols(0) == "container_1" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 0, 0, 1.0 ) ), OT_sumOfSqr, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_sumOfSqr, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_1", *symbols(1), 0.05 ), OT_sumOfSqr, NoArr, 1e2 );

    //activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": over plane of " << *symbols(0) << " activated" << std::endl;
  }
}

/*void groundDeactivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  for( auto req : activeTasks_ )
  {
    if( req.komo == komo && req.symbols == symbols )
    {
      //req.task->
    }
  }
}
private:

struct ActiveTask
{
  KOMO * komo;
  StringL symbols;
  Task * task;
};

std::list< ActiveTask > activeTasks_;

};*/


void groundObjectPairCollisionAvoidance( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  //std::cout << facts << std::endl;

  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =  komo->maxPhase;
  //

//  for( auto s1 : komo->world.getBodyByName( *symbols(0) )->shapes )
//  {
//    for( auto s2 : komo->world.getBodyByName( *symbols(1) )->shapes )
//    {
//      //komo->setTask( t_start, t_end, new ShapePairCollisionConstraint( komo->world, s1->name, s2->name, 0.1 ), OT_ineq, NoArr, 1e2 );
//    }
//  }
}

//===========================================================================

void plan()
{
  // instanciate planners
  auto tp = std::make_shared< tp::MCTSPlanner >();
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // register symbols
  mp->registerTask( "komoGrasp"       , groundGrasp );
  mp->registerTask( "komoGraspObject" , groundGraspObject );
  mp->registerTask( "komoPlace"       , groundPlace );
  mp->registerTask( "komoGetSight"    , groundGetSight );
  mp->registerTask( "komoTakeView"    , groundTakeView );
  mp->registerTask( "komoActivateOverPlane"   , groundActivateOverPlane );
  //komoFactory_.registerTask( "komoDeactivateOverPlane" , groundDeactivateOverPlane );
  mp->registerTask( "komoCollisionAvoidance", groundObjectPairCollisionAvoidance );

  // set start configurations
  tp->setFol( "LGP-obs-container-fol-place-pick-2.g" );
  mp->setKin( "LGP-obs-container-kin.g" );

  for( uint i = 0; ! tp->terminated() && i < 10 ; ++i )
  {
    std::cout << "Task planning to generate " << i << "th policy" << std::endl;

    // TASK PLANNING
    tp->solve();
    auto policy = tp->getPolicy();
    auto po     = tp->getPlanningOrder();

    // save policy
    savePolicyToFile( policy );

    std::cout << "Motion Planning for policy " << i << std::endl;

    // MOTION PLANNING
    mp->solveAndInform( po, policy );

    // print resulting cost
    //std::cout << "cost of the policy " << i << " " << policy->cost() << std::endl;

    tp->integrate( policy );
  }

  mp->display( tp->getPolicy(), 3000 );
}

void plan_graph()
{
  auto tp = std::make_shared< tp::GraphSearchPlanner >();
  auto mp = std::make_shared< mp::KOMOPlanner >();

  // set planner specific parameters
  tp->setInitialReward( -0.1 );
  mp->setNSteps( 5 );

  tp->setFol( "LGP-obs-container-fol-place-pick-2-no-take-view.g" );
  mp->setKin( "LGP-obs-container-kin.g" );

  tp->buildGraph();

  tp->saveGraphToFile( "graph.gv" );
  generatePngImage( "graph.gv" );
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan_graph();

  return 0;
}
