#include <list>
#include <unordered_set>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <Kin/frame.h>
#include <car_kinematic.h>
#include <approx_shape_to_sphere.h>
#include <graph_planner.h>
#include <komo_planner.h>
#include <markovian_tamp_controller.h>
#include <joint_path_tamp_controller.h>
#include <axis_bound.h>


//===========================================================================

void init( mp::ExtensibleKOMO * komo, int verbose )
{
  // ego car
  arr ego_desired_speed{ 0.04, 0, 0 };
  komo->setVelocity( 0.0, -1.0, "car_ego", NULL, OT_sumOfSqr, ego_desired_speed );
  komo->setTask( 0.0, -1.0, new CarKinematic( "car_ego" ), OT_eq, NoArr, 1e2, 1 );

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

  for( uint i = 0; i < 10000; ++i )
  {
    std::cout << "*********************" << std::endl;
    std::cout << "*******"<< i << "******" << std::endl;
    std::cout << "*********************" << std::endl;

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
    tp.setR0( -0.01 );  // balance exploration
    tp.setMaxDepth( 5 );
    tp.buildGraph();

    //tp.saveGraphToFile( "graph.gv" );
    //generatePngImage( "graph.gv" );

    // set initial parameters
    auto vec = mp.drawRandomVector(); // random
//    auto vec = mp.drawRandomVector({0.745718947008547,-0.08723548758290603});//plan 0
//    auto vec = mp.drawRandomVector({-0.7867631799485865,-0.5741730488431879});//plan 1
//    auto vec = mp.drawRandomVector({-0.38302520754166647,0.04392461586388886});//plan 2
//    auto vec = mp.drawRandomVector({-0.020404977034782636,-0.8433419808695655});//plan 3
//    auto vec = mp.drawRandomVector({-0.5512255441448696,-0.2612012252334004});//plan 4
//    auto vec = mp.drawRandomVector({0.16063258833188399,0.6229994332567289});//plan 5
//    auto vec = mp.drawRandomVector({-0.26539484409956043,0.3638281736456812});//plan 6

//    auto vec = mp.drawRandomVector({0.970007,0.035656});
//    auto vec = mp.drawRandomVector({0.970007,-0.535656});

    //MarkovianTAMPController controller( tp, mp );
    JointPathTAMPController controller( tp, mp );

    auto policy = controller.plan(1000, false, true, true, 30);

    skeletonsToStart[policy].push_back(vec);

    if( i && i % 100 == 0 )
    {
      saveDataToFileveDataToFile("result-data-" + std::to_string(i) + ".csv", skeletonsToStart);
    }
  }
  saveDataToFileveDataToFile("result-data.csv", skeletonsToStart);
}

std::list< std::vector< double > > parseDeltas( const std::string & filepath )
{
  std::list< std::vector< double > > deltas;

  std::ifstream infile;
  infile.open(filepath);

  std::string line;

  while ( std::getline(infile, line) )
  {
    char split_char = ';';
    std::istringstream iss(line);
    std::vector< double > delta;

    for ( std::string each; std::getline( iss, each, split_char ); delta.push_back( std::stod( each ) ) );

    delta.pop_back(); // remove last elements because it is the skeleton number

    deltas.push_back( delta );
  }

  return deltas;
}

std::unordered_set< Skeleton, SkeletonHasher > parseSkeletons( const std::string & folderpath )
{
  std::unordered_set< Skeleton, SkeletonHasher > skeletons;

  auto p = boost::filesystem::path( folderpath );

  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator( p ) ) )
  {
    auto filepath = entry.path().string();

    if( filepath.find(".po") != std::string::npos )
    {
      Skeleton ske; ske.load( filepath );
      skeletons.insert( ske );
    }
  }

  return skeletons;
}

std::string getKinFilepath( const std::string & folderpath )
{
  auto p = boost::filesystem::path( folderpath );

  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator( p ) ) )
  {
    auto filepath = entry.path().string();

    if( filepath.find("-kin.g") != std::string::npos )
    {
      return filepath;
    }
  }

  return "";
}

void saveSkeletonValuesToFile( const std::string filename, const std::map< std::vector< double >, std::unordered_set< Skeleton, SkeletonHasher > > & deltasToValues )
{
  std::ofstream of;
  of.open( filename );

  for( const auto dataPair : deltasToValues )
  {
    const auto delta = dataPair.first;
    const auto skeletons = dataPair.second;

    for( auto d : delta )
    {
      of << d << ";";
    }

    of << "nan" << ";";

    for( const auto skeleton : skeletons )
    {
      //skeleton.saveAll( "-" + std::to_string(skeId) );
      of << skeleton.value() << ";";
    }

    of << std::endl;
  }

  of.close();
}

void evaluate_all_skeletons( const std::string & result_filepath  )
{
  // retrieve deltas
  auto deltas = parseDeltas( result_filepath );

  // retrieve skeletons
  const auto folderpath = boost::filesystem::path( result_filepath ).parent_path().string();
  auto skeletons = parseSkeletons( folderpath );

  // retrieve kin file
  auto kin_filepath = getKinFilepath( folderpath );

  // replan for each skeleton and each delta
  std::map< std::vector< double >, std::unordered_set< Skeleton, SkeletonHasher > > deltasToValues;
  for( const auto & vec : deltas )
  {
    for( auto skeleton : skeletons )
    {
      // plan for each skeleton
      mp::KOMOPlanner mp;

      // set worlds
      mp.setKin( kin_filepath.c_str() );

      // set planner specific parameters
      mp.setNSteps( 20 );

      // register symbols
      mp.registerInit( init );
      mp.registerTask( "continue"        , groundContinue );
      mp.registerTask( "merge_between"   , groundMergeBetween );

      /// APPLY DELTAS
      mp.drawRandomVector( vec );

      /// MOTION PLANNING
      auto po     = MotionPlanningParameters( skeleton.id() );
      po.setParam( "type", "jointPath" );
      mp.solveAndInform( po, skeleton );

      deltasToValues[vec].insert( skeleton );
    }
  }

  saveSkeletonValuesToFile( folderpath + "/" + "result-values.csv", deltasToValues );
}

//===========================================================================

int main(int argc,char **argv)
{
  mlr::initCmdLine(argc,argv);

  plan();
  //evaluate_all_skeletons("joint_car_kin/100/result-data-100.csv");

  return 0;
}
