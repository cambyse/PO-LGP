#include <functional>
#include <list>

#include <chrono>

//#include <policy_printer.h>

//#include <policy_builder.h>

#include <graph_planner.h>

#include <komo_planner.h>

#include <observation_tasks.h>
//#include <approx_point_to_shape.h>
//#include <vertical_velocity.h>
//#include <axis_alignment.h>
//#include <over_plane.h>
//#include <axis_bound.h>

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

static void savePolicyToFile( const Skeleton & policy, const std::string & suffix = "" )
{
  std::stringstream namess, skenamess;
  namess << "policy-" << policy.id() << suffix << ".gv";
  auto name = namess.str();

  policy.save( name );
  policy.saveToGraphFile( name );
  // generate nice graph
//  {
//    std::ofstream file;
//    file.open( name );
//    PolicyPrinter printer( file );
//    printer.print( policy );
//    file.close();

//    generatePngImage( name );
//  }
}

//==========Application specific grounders===================================
void groundPrefixIfNeeded( mp::ExtensibleKOMO * komo, int verbose  )
{
//  if( ! komo->isPrefixSetup() )
//  {
//    komo->setHoming( -1., -1., 1e-2 ); //gradient bug??
//    komo->setTask( -1, -1, new AxisBound( "manhead", 1.4, AxisBound::Z, AxisBound::MIN ), OT_ineq, NoArr, 1e2 );
//  }
}

void groundPickUp( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", "tableC", facts[0].c_str() );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "baxterR", facts[0].c_str() );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": pick up " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundUnStack( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", facts[1].c_str(), facts[0].c_str() );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", "baxterR", facts[0].c_str() );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": unstack " << facts[0] << " from " << facts[1] << std::endl;
  }
}

void groundPutDown( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
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
    std::cout << t_start << "->" << t_end << ": " << " put down " << facts[0] << " at " << facts[1] << std::endl;
  }
}

void groundCheck( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  groundPrefixIfNeeded( komo, verbose );

  double duration=1.0;

  //
  const double t_start = phase + 0.5;
  const double t_end =   phase + duration;
  //

  komo->setTask( t_start, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sumOfSqr, NoArr, 1e2 );
  komo->setTask( t_end - 0.1, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": " << " check " << facts[0] << std::endl;
  }
}

void groundStack( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
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

//===========================================================================

void plan_graph_search()
{
  matp::GraphPlanner tp;
  mp::KOMOPlanner mp;

  // set planner specific parameters
  tp.setR0( -0.5 );
  tp.setMaxDepth( 15 );
  mp.setNSteps( 10 );

  // register symbols
  mp.registerTask( "pick-up"      , groundPickUp );
  mp.registerTask( "put-down"     , groundPutDown );
  mp.registerTask( "check"        , groundCheck );
  mp.registerTask( "stack"        , groundStack );
  mp.registerTask( "unstack"      , groundUnStack );

  // set start configurations
  //tp.setFol( "LGP-blocks-fol-unified.g" );
  //tp.setFol( "LGP-blocks-fol-model-2-unified.g" );
  //mp.setKin( "LGP-blocks-kin-unified.g" );

  // checked, probably doesn't work with n steps = 5
  //tp.setFol( "LGP-blocks-fol-2w-unified.g" );
  //tp.setFol( "LGP-blocks-fol-2w-model-2-unified.g" );
  //mp.setKin( "LGP-blocks-kin-2w-unified.g" );

  // checked
  tp.setFol( "LGP-blocks-fol-1w-unified.g" );
  //tp.setFol( "LGP-blocks-fol-1w-model-2-unified.g" );
  mp.setKin( "LGP-blocks-kin-1w-unified.g" );

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
  tp.buildGraph(true);
auto elapsed = std::chrono::high_resolution_clock::now() - start;
graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
  //tp.saveGraphToFile( "graph.gv" );
  //generatePngImage( "graph.gv" );

  Skeleton policy, lastPolicy;

{
auto start = std::chrono::high_resolution_clock::now();
  tp.solve();
auto elapsed = std::chrono::high_resolution_clock::now() - start;
task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
  policy = tp.getPolicy();

  do
  {
    ///
    savePolicyToFile( policy );
    candidate << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
    ///

    lastPolicy = policy;

{
auto start = std::chrono::high_resolution_clock::now();
    /// MOTION PLANNING
    auto po     = MotionPlanningParameters( policy.id() );
    po.setParam( "type", "markovJointPath" );
    mp.solveAndInform( po, policy );
auto elapsed = std::chrono::high_resolution_clock::now() - start;
motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
    ///
    savePolicyToFile( policy, "-informed" );
    results << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
    ///

{
auto start = std::chrono::high_resolution_clock::now();
    /// TASK PLANNING
    tp.integrate( policy );
    tp.solve();
auto elapsed = std::chrono::high_resolution_clock::now() - start;
task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}
    policy = tp.getPolicy();

  } while( lastPolicy != policy );

/////
  savePolicyToFile( policy, "-final" );
  candidate << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;
  results << policy.id() << "," << std::max( -10.0, policy.value() ) << std::endl;

  candidate.close();
  results.close();
/////

{
auto start = std::chrono::high_resolution_clock::now();
  mp.display( policy, 30 );
auto elapsed = std::chrono::high_resolution_clock::now() - start;
joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
}

/////
  std::ofstream timings;
  timings.open("timings.data");
  timings << "graph_building_s="<< graph_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< graph_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();
/////

  mlr::wait( 30, true );
}

void baxter()
{
  {
    mlr::KinematicWorld kin;
    kin.init( "LGP-blocks-kin-1w-unified.g" );
    kin.watch();
    kin.write( std::cout );

    mlr::wait( 300, true );
  }
}

//===========================================================================

int main(int argc,char **argv)
{
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan_graph_search();

  //baxter();

  return 0;
}
