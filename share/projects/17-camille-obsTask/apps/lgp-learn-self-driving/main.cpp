#include <list>
#include <Kin/frame.h>
#include <approx_shape_to_sphere.h>
#include <graph_planner.h>
#include <komo_planner.h>
#include <tamp_controller.h>
#include <axis_bound.h>

//TODO:
//

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

    skeleton.saveAll( "-" + std::to_string(skeId) );

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
  std::unordered_map< Skeleton, std::list< std::vector< double > >, SkeletonHasher > skeletonsToStart;

  for( uint i = 0; i < 500; ++i )
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
    mp.registerInit( init );
    mp.registerTask( "continue"        , groundContinue );
    mp.registerTask( "merge_between"   , groundMergeBetween );

    /// DECISION GRAPH
    tp.setR0( -0.001 );  // balance exploration
    tp.setMaxDepth( 5 );
    tp.buildGraph();

    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    // set initial parameters
    //auto vec = mp.drawRandomVector(); // random
//    auto vec = mp.drawRandomVector({0.7626620612244897,-0.05964662244897959});//plan 0
//    auto vec = mp.drawRandomVector({-0.7991766000000001,-0.5479612000000003});//plan 1
//    auto vec = mp.drawRandomVector({-0.3448865342857143,0.026629800000000006});//plan 2
    auto vec = mp.drawRandomVector({0.03870015128205127,-0.8419756923076921});//plan 3
//    auto vec = mp.drawRandomVector({-0.5610455045454545,-0.2883110378636364});//plan 4
//    auto vec = mp.drawRandomVector({0.15762804745098036,0.6434303333333331});//plan 5
//    auto vec = mp.drawRandomVector({-0.2806867376470588,0.3627838470588235});//plan 6

    TAMPController controller( tp, mp );

    auto policy = controller.plan(1000, false, false, true, 30);

    skeletonsToStart[policy].push_back(vec);
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
