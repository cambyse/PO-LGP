#include <functional>
#include <list>
#include <Kin/frame.h>
#include <approx_shape_to_sphere.h>
#include <graph_planner.h>

#include <komo_planner.h>

#include <axis_bound.h>

//TODO:
//-add tamp controller to control main loop
//-deduce number of var parameters from .kin file
//

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
}

//===========================================================================

void init( mp::ExtensibleKOMO * komo, int verbose )
{
  // ego car
  arr ego_desired_speed{ 0.04, 0, 0 };
  komo->setVelocity( 0.0, -1.0, "car_ego", NULL, OT_sumOfSqr, ego_desired_speed );

  // car speeds
  arr desired_speed{ 0.03, 0, 0 };
  komo->setVelocity( 0.0, -1, "car_1", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_2", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_3", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_4", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_5", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_6", NULL, OT_sumOfSqr, desired_speed );
  komo->setVelocity( 0.0, -1, "car_7", NULL, OT_sumOfSqr, desired_speed );

  const auto radius = 0.35;
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_1", "car_2", radius ), OT_ineq );
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_2", "car_3", radius ), OT_ineq );
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_3", "car_4", radius ), OT_ineq );
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_4", "car_5", radius ), OT_ineq );
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_5", "car_6", radius ), OT_ineq );
  komo->setTask( 0.0, -1, new ApproxShapeToSphere( komo->world, "car_6", "car_7", radius ), OT_ineq );

  /// RANDOMIZE SCENE
  double scale = 0.3;

  // initial position
  mlr::KinematicWorld world;
  world.copy(komo->world);

  //randomVec={-1.0, -1.0};

//  uint i = 0;
//  for( const auto & f: world.frames )
//  {
//    if( f->ats["random_bounds"]  )
//    {
//      auto random_bounds = f->ats.get<arr>("random_bounds");

//      for( uint j = 0; j < 1/*joint_offsets[f->joint->dim*/; ++j )
//      {
//        world.q(f->joint->qIndex + j) = random_bounds(j) * randomVec[i];
//        ++i;
//      }
//    }
//  }

//  world.calc_Q_from_q();
//  world.calc_fwdPropagateFrames();

//  //world.watch(true);

//  komo->setModel(world);
}

void groundContinue( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
}

void groundMergeBetween( double phase, const std::vector< std::string >& facts, mp::ExtensibleKOMO * komo, int verbose )
{
  auto car_before = facts[0];
  auto car_next = facts[1];

  komo->setPosition( phase+1, -1, "car_ego", car_next.c_str(), OT_sumOfSqr, {-0.7, 0, 0} );
  komo->setPosition( phase+1, -1, car_before.c_str(), "car_ego", OT_sumOfSqr, {-0.7, 0, 0} );

  //setKeepDistanceTask( phase+1, -1, komo, car_successors );
  //std::cout << "merge between " << car_before << " and " << car_next << std::endl;
}

std::vector< double > randomVector( uint dim )
{
  std::vector< double > vec( dim );

  for( auto & v : vec )
  {
    v = rnd.uni(-1.0, 1.0);
  }

  return vec;
}

void saveDataToFileveDataToFile( const std::string filename, const std::unordered_map< Skeleton, std::list< std::vector< double > >, SkeletonHasher > & skeletonsToStart )
{
  std::ofstream of;
  of.open(filename);

  uint skeId = 0;
  for( const auto dataPair : skeletonsToStart )
  {
    const auto skeleton = dataPair.first;

    savePolicyToFile( skeleton, "-" + std::to_string(skeId) );
    skeleton.save("skeleton-" + std::to_string(skeId) + ".ske");

    const auto deltas = dataPair.second;
    for( const auto vec : deltas )
    {
      for( auto d : vec )
      {
        of << d << ";";
      }
      of << skeId << std::endl;
    }
    skeId++;
  }

  of.close();
}

void plan()
{
  using namespace std::placeholders;

  std::unordered_map< Skeleton, std::list< std::vector< double > >, SkeletonHasher > skeletonsToStart;
  for(uint i = 0; i < 1 /*500*/; ++i)
  {
    std::cout << "*********" << std::endl;
    std::cout << "***"<< i << "***" << std::endl;
    std::cout << "*********" << std::endl;

    matp::GraphPlanner tp;
    mp::KOMOPlanner mp;

    // set worlds
    mp.setKin( "LGP-merging-kin.g" );
    tp.setFol( "LGP-merging-1w.g" );

    // set planner specific parameters
    mp.setNSteps( 20 );

    // register symbols
    auto vec = mp.drawRandomVector({-1.0, -1.0}); // randomization
    mp.registerInit( init );//std::bind( init, _1, _2, vec ) );
    mp.registerTask( "continue"        , groundContinue );
    mp.registerTask( "merge_between"   , groundMergeBetween );

    /// DECISION GRAPH
    tp.setR0( -0.001 );  // balance exploration
    tp.setMaxDepth( 5 );
    tp.buildGraph();

    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    /// LOOP
    Skeleton policy, lastPolicy;
    tp.solve();
    policy = tp.getPolicy();

    uint nIt = 0;
    const uint maxIt = 1000;
    do
    {
      nIt++;

      lastPolicy = policy;

      /// MOTION PLANNING
      auto po     = MotionPlanningParameters( policy.id() );
      po.setParam( "type", "markovJointPath" );
      mp.solveAndInform( po, policy );

      ///
      //savePolicyToFile( policy, "-informed" );
      ///

      /// TASK PLANNING
      tp.integrate( policy );
      tp.solve();

      policy = tp.getPolicy();
    }
    while( lastPolicy != policy && nIt != maxIt );

    skeletonsToStart[policy].push_back(vec);

    //savePolicyToFile( policy, "-final" );

    mp.display( policy, 3000 );
    mlr::wait( 30, true );
  }

  saveDataToFileveDataToFile("result-data.csv", skeletonsToStart);
}

//===========================================================================

int main(int argc,char **argv)
{
  mlr::initCmdLine(argc,argv);

  plan();

  return 0;
}
