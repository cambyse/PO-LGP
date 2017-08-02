#include <functional>
#include <list>

#include <policy.hpp>

#include <policy_builder.h>

#include <mcts_planner.h>
#include <komo_planner.h>

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>
#include <node_visitors.h>


/*
back track, take history into account?
sort nodes before expanding?
back track result of pose computation when one of the pose is not possible or generally different between worlds!
less rollouts?
dot -Tpng -o policy.png policy.gv

test a logic : mlr/share/example/DomainPlayer

QUESTIONS  :
- why no proxy?
- how to solve collision avoidance if objects penetrate
- kinematic switches ( commented part of the code ) in solvePath, solvePose, etc..

TODO :
1/ decision if optimization succeded, how? reactivate for seq and paths!!    | 1
=> constraints are difficult to evaluate with collision avoidance, mybe need a refactoring as in 3/
2/ symbolic search, use costs from other levels? -> How to inform?           | 1
3/ refactoring geometric levels, backtrack can be common?                    | 2
5/ collision avoidance, rule for proxy ?, get out of a collision             | 2
6/ activation / deactivation of tasks                                        | 2
7/ correct memory management                                                 | 2
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

//===========================================================================

void plan()
{
  // instanciate planners
  TaskPlanner::ptr   tp = std::make_shared< tp::MCTSPlanner >();
  MotionPlanner::ptr mp = std::make_shared< mp::KOMOPlanner >();

  // set start configurations
  tp->setFol( "LGP-obs-container-fol-place-pick-2.g" );
  mp->setKin( "LGP-obs-container-kin.g" );

  for( uint i = 0; i < 50 && ! tp->terminated(); ++i )
  {
    // TASK PLANNING
    tp->solve();
    auto policy = tp->getPolicy();

    std::cout << "Optimizing motions for policy " << i << std::endl;

    // save policy
    {
      std::stringstream namess;
      namess << "policy-" << i << ".gv";
      auto name = namess.str();

      std::ofstream file;
      file.open( name );
      PolicyPrinter printer( file );
      printer.print( policy );
      file.close();

      generatePngImage( name );
    }

    // MOTION PLANNING
    mp->solveAndInform( policy );

    // print resulting cost
    std::cout << "cost of the policy " << i << " " << policy->cost() << std::endl;

    tp->integrate( policy );
  }

  mp->display( tp->getPolicy(), 30 );

  /*
  // store policy and display it
  auto currentBestPolicy = C.getPolicy();
  policies.insert( currentBestPolicy );
  //

  /////// 2 - Policy Optimization ////////
  uint maxAlternatives = 0;
  for( auto alternatives = 0; ! C.isPolicyFringeEmpty() && alternatives < maxAlternatives; alternatives++ )
  {
   C.generateAlternativeSymbolicPolicy();

//    {
//      // save search tree
//      std::stringstream namess;
//      namess << "search-alternative-" << C.alternativeNumber() << ".gv";
//      C.printSearchTree( namess.str() );
//    }
    C.solveGeometrically();

    {
    // save policy
    std::stringstream namess;
    namess << "policy-alternative-" << alternatives << ".gv";
    C.printPolicy( namess.str() );
    }

    // store policy and display it
    auto altPolicy = C.getPolicy();
    policies.insert( altPolicy );

    if( altPolicy->cost() > currentBestPolicy->cost() )
    {
      C.revertToPreviousPolicy();
    }
    else
    {
      currentBestPolicy = altPolicy;
    }
  }

  /////// 3 - Display ////////
  PolicyVisualizer viz( *policies.begin(), "nominal" );

  //C.updateDisplay( WorldID( -1 ), false, false, true );
  mlr::wait( 3000, true );*/
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
