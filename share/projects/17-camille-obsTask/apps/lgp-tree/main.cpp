#include <functional>
#include <list>

#include <policy.h>
#include <policy_printer.h>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <komo_planner.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>
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

//===========================================================================

void plan()
{
  // instanciate planners
  TaskPlanner::ptr   tp = std::make_shared< tp::MCTSPlanner >();
  MotionPlanner::ptr mp = std::make_shared< mp::KOMOPlanner >();

  // set start configurations
  tp->setFol( "LGP-obs-container-fol-place-pick-2.g" );
  mp->setKin( "LGP-obs-container-kin.g" );

  for( uint i = 0; ! tp->terminated() && i < 30 ; ++i )
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
    std::cout << "cost of the policy " << i << " " << policy->cost() << std::endl;

    tp->integrate( policy );
  }

  mp->display( tp->getPolicy(), 3000 );
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

  //orsDrawAlpha = 1.;
  //orsDrawJoints=orsDrawMarkers=false;
  //  orsDrawCores = true;
  //if(mlr::getParameter<bool>("intact")){
  //  test();
  //}else{
    //    test();
    plan();
  //}

  return 0;
}
