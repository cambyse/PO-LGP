#include <functional>
#include <list>

#include <chrono>

#include <Kin/kinViewer.h>

#include <graph_planner.h>

#include <komo_planner.h>
#include <approx_shape_to_sphere.h>
#include <observation_tasks.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>


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

static void savePolicyToFile( const Policy & policy, const std::string & suffix = "" )
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

  //  // donw before pick
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

  if( facts[0] != "block_1" )
    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_1", radius ), OT_ineq );

  if( facts[0] != "block_2" )
    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_2", radius ), OT_ineq );

  if( facts[0] != "block_3" )
    komo->addObjective( t_end - 0.5, t_end + 0.5, new ApproxShapeToSphere( komo->world, facts[0].c_str(), "block_3", radius ), OT_ineq );

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

  komo->addObjective( t_start, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_sos, NoArr, 1e2 );
  komo->addObjective( t_end - 0.1, t_end, new ActiveGetSight( "head", facts[0].c_str(), ARR( 0.05, 0, 0 ), ARR( -1, 0, 0 ), 0.65 ), OT_eq, NoArr, 1e2 );

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

//===========================================================================

void plan_graph_search()
{
  ///
  std::ofstream candidate, results;
  candidate.open( "policy-candidates.data" );
  results.open( "policy-results.data" );
  double graph_building_s = 0;
  double task_planning_s = 0;
  double motion_planning_s = 0;
  double joint_motion_planning_s = 0;

  namespace ba = boost::accumulators;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_length;
  boost::accumulators::accumulator_set<double, ba::features< ba::tag::variance, ba::tag::mean, ba::tag::min, ba::tag::max > > acc_acc_cost;

  for( auto i = 0; i < 1; ++i )
  {
    matp::GraphPlanner tp;
    mp::KOMOPlanner mp;

    // set planner specific parameters
    tp.setR0( -5 ); //-0.25//-0.1//-0.015 );
    tp.setMaxDepth( 20 );
    mp.setNSteps( 20 );
    mp.setMinMarkovianCost( 0.00 );
    ///
    // register symbols
    mp.registerTask( "pick-up"      , groundPickUp );
    mp.registerTask( "put-down"     , groundPutDown );
    mp.registerTask( "check"        , groundCheck );
    mp.registerTask( "stack"        , groundStack );
    mp.registerTask( "unstack"      , groundUnStack );

    // set start configurations
    // D
    //tp.setFol( "LGP-blocks-fol-model-2-unified.g" );
    //mp.setKin( "LGP-blocks-kin-unified.g" );

    // C
    //tp.setFol( "LGP-blocks-fol-unified.g" );
    //mp.setKin( "LGP-blocks-kin-unified.g" );

    // checked, probably doesn't work with n steps = 5
    // B
    //tp.setFol( "LGP-blocks-fol-2w-unified.g" );
    //mp.setKin( "LGP-blocks-kin-2w-unified.g" );

    //tp.setFol( "LGP-blocks-fol-2w-model-2-unified.g" );

    // checked
    // A
    tp.setFol( "LGP-blocks-fol-1w-unified.g" );
    mp.setKin( "LGP-blocks-kin-1w-unified.g" );

    //tp.setFol( "LGP-blocks-fol-1w-model-2-unified.g" );

    // 4 blocks linear
    //tp.setFol( "LGP-blocks-fol-1w-unified-4-blocks.g" );
    //mp.setKin( "LGP-blocks-kin-1w-unified-4-blocks.g" );

    // 4 blocks new version
    //tp.setFol( "LGP-blocks-fol-1w-unified-4-blocks-new.g" );
    //tp.setFol( "LGP-blocks-fol-2w-model-2-unified.g" );
    //mp.setKin( "LGP-blocks-kin-1w-unified-4-blocks-new.g" );

    {
      auto start = std::chrono::high_resolution_clock::now();
      /// GRAPH BUILDING
      tp.buildGraph(true);
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      graph_building_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    Policy policy, lastPolicy;

    {
      auto start = std::chrono::high_resolution_clock::now();
      tp.solve();
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      task_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    policy = tp.getPolicy();

    uint nIt = 0;
    const uint maxIt = 1000;
    do
    {
      nIt++;
      ///
      savePolicyToFile( policy );
      candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
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
      results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
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

    } while( lastPolicy != policy && nIt != maxIt );

    /////
    savePolicyToFile( policy, "-final" );
    candidate << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;
    results << policy.id() << "," << std::min( 10.0, -policy.value() ) << std::endl;

    candidate.close();
    results.close();
    /////
    {
      auto start = std::chrono::high_resolution_clock::now();
      mp.display( policy, 3000 );
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      joint_motion_planning_s+=std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() / 1000000.0;
    }
    /////


//    // eval
//    auto eval = mp.evaluateLastSolution();
//    acc_length( eval.first );
//    acc_acc_cost( eval.second );
  }

  std::ofstream timings;
  timings.open("timings.data");
  timings << "graph_building_s="<< graph_building_s << std::endl;
  timings << "task_planning_s="<< task_planning_s << std::endl;
  timings << "motion_planning_s="<< motion_planning_s << std::endl;
  timings << "joint_motion_planning_s="<< joint_motion_planning_s << std::endl;
  timings << "total_s="<< graph_building_s + task_planning_s + motion_planning_s + joint_motion_planning_s << std::endl;

  timings.close();

  // evaluation
  std::cout << "LENGTH: [" <<  ba::min( acc_length ) << " " << ba::max( acc_length ) << "] mean:" << ba::mean( acc_length ) << " std_dev:" << sqrt( ba::variance( acc_length ) ) << std::endl;
  std::cout << "ACC COSTS: [" << ba::min( acc_acc_cost ) << " " << ba::max( acc_acc_cost ) << "] mean:" << ba::mean( acc_acc_cost ) << " std_dev:" << sqrt( ba::variance( acc_acc_cost ) ) << std::endl;
}

void baxter()
{
  {
    rai::KinematicWorld kin;
    kin.init( "LGP-blocks-kin-unified-b6.g" );

//    const double zf = 1.47;
//    const double s = 0.55;
//    kin.gl().camera.setPosition(s * 10., s * 4.5, zf + s * ( 3.5 - zf ));

    const double zf = 1.0;
    const double s = 0.35;
    kin.gl().camera.setPosition(s * 10., s * 0, zf + s * ( 1.5 - zf ));

    kin.gl().camera.focus(0.5, 0, zf);
    kin.gl().camera.upright();

    kin.watch();
    kin.write( std::cout );

    rai::wait( 300, true );
  }
}

//===========================================================================

int main(int argc,char **argv)
{
  rai::initCmdLine(argc,argv);

  rnd.clockSeed();

  plan_graph_search();

  //baxter();

  return 0;
}
