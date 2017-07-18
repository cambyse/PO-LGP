#include <functional>

#include <MCTS/solver_PlainMC.h>

#include "ao_search.h"

#include <observation_tasks.h>
#include <object_pair_collision_avoidance.h>
#include <geometric_levels.h>
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
4/ how to know if a was is successfull -> Call back every task?              | 2
5/ collision avoidance, rule for proxy ?, get out of a collision             | 2
6/ activation / deactivation of tasks                                        | 2
7/ correct memory management                                                 | 2
8/ refactor to consider an arbitrary number of geometric levels              | 2
*/
//===========================================================================

/*static void iterate( SearchSpaceTree & C, ofstream & fil )
{
  //    C.root->checkConsistency();
  { //expand
    PartiallyObservableNode* n = popBest(C.mcFringe, mcHeuristic);
    //      ActionNode* n = NULL;
    //      for(uint k=0;k<10;k++){ n=C.root->treePolicy_softMax(0.); if(n) break; }
    if(n)
    {
      n->expand();
      for(PartiallyObservableNode* c:n->children())
      {
        c->addMCRollouts(10,10);
        C.mcFringe.append(c);
        if(c->isTerminal()) C.terminals.append(c);
        if(n->poseCount())  C.poseFringe.append(c);
        //          if(n->seqCount) C.seqFringe.append(c);
        //if(c->isTerminal) C.seqFringe.append(c);
      }
    }
  }

  { //add additional MC rollouts
    for(uint mc=0;mc<10;mc++)
    {
      PartiallyObservableNode* n = NULL;
      for(uint k=0;k<10;k++)
      {
        n=C.root->treePolicy_random(); if(n) break;
      }
      if(n)
      {
        n->addMCRollouts(2,10);
      }
    }
  }

  C.root->recomputeAllMCStats();

  //    C.updateDisplay();

  { //optimize a pose
    PartiallyObservableNode* n = popBest(C.poseFringe, poseHeuristic);
    if(n)
    {
      //      cout <<"### POSE TESTING node " <<*n <<endl;
      //      mlr::wait();
      n->solvePoseProblem();
      if(n->poseFeasible())
      {
        for(PartiallyObservableNode* c:n->children()) C.poseFringe.append(c); //test all children
        if(n->isTerminal()) C.seqFringe.append(n); //test seq or path
      }
      C.node = n;
    }
  }

  { //optimize a seq
    PartiallyObservableNode* n = popBest(C.seqFringe, seqHeuristic);
    if(n)
    {
      //      cout <<"### SEQ TESTING node " <<*n <<endl;
      //      mlr::wait();
      n->solveSeqProblem();
      if(n->seqFeasible())
      {
        //          for(MNode* c:n->children) C.seqFringe.append(c);
        if(n->isTerminal()) C.pathFringe.append(n);
      }
      C.node = n;
    }
  }

  { //optimize a path
    PartiallyObservableNode* n = popBest(C.pathFringe, pathHeuristic);
    if(n)
    {
      //      cout <<"### PATH TESTING node " <<*n <<endl;
      //      mlr::wait();
      n->solvePathProblem(10);
      if(n->pathFeasible()) C.done.append(n);
      C.node = n;
    }
  }

  for(auto *n:C.terminals) CHECK(n->isTerminal(),"");

  //    C.updateDisplay();
  for(PartiallyObservableNode *n:C.mcFringe) if(!n->mcStats()->n)
  {
    //      cout <<"recomputing MC rollouts for: " <<*n->decision <<endl;
    //      mlr::wait();
    //      C.root->rootMC->verbose = 2;
    n->addMCRollouts(10,10);
    //      C.updateDisplay();
  }

  PartiallyObservableNode *bt = getBest(C.terminals, seqCost);
  PartiallyObservableNode *bp = getBest(C.done, pathCost);
  mlr::String out;
  out <<"TIME= "        <<mlr::cpuTime()  <<" KIN= " <<COUNT_kin    <<" EVALS= " <<COUNT_evals
      <<" POSE= "       <<COUNT_poseOpt   <<" SEQ= " <<COUNT_seqOpt <<" PATH= " <<COUNT_pathOpt
      <<" bestPose= "   <<(bt?poseCost(bt):100.)
      <<" bestSeq= "    <<(bt?seqCost(bt):100.)
      <<" pathSeq= "    <<(bp?pathCost(bp):100.)
      <<" #solutions= " <<C.done.N;

  fil  <<out <<endl;
  cout <<out <<endl;

  if(bt) C.node=bt;
  if(bp) C.node=bp;
  C.updateDisplay();
  //    mlr::wait();

  //    { //optimize a path
  //      ActionNode* n = pqPath.pop();
  //      if(n){
  //        n->solvePathProblem();
  //        if(n->symTerminal && n->pathFeasible){ //this is a symbolic solution
  //          pqDone.add(n, n->symCost + n->pathCost);
  //        }
  //      }
  //    }

  //    cout <<"===================== CURRENT QUEUES:" <<endl;
  //    cout <<"MCfringe:" <<C.MCfringe <<endl;
  //    cout <<"seqFringe:" <<C.seqFringe <<endl;
  //    cout <<"pathFringe:" <<C.pathFringe <<endl;
  //    cout <<"pqDone:" <<pqDone <<endl;

}*/

//===========================================================================

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
  const double t_end =   phase + duration;
  //

  //disconnect object from table
  komo->setKinematicSwitch( t_end, true, "delete", NULL, *symbols(1) );
  //connect graspRef with object
  komo->setKinematicSwitch( t_end, true, "ballZero", *symbols(0), *symbols(1) );

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
                                                                        ARR( -0.0, 0.1, 0.4 ) ),  // pivot position  in container frame
                OT_sumOfSqr, NoArr, 1e2 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": getting sight of " << *symbols(0) << std::endl;
  }
}

void groundTakeView( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  double duration=n->get<double>();

  //
  const double t_start = phase;
  const double t_end =   phase + duration;
  //

  auto *map = new TaskMap_Transition( komo->world );
  map->posCoeff = 0.;
  map->velCoeff = 1.;
  map->accCoeff = 0.;
  komo->setTask( t_start, t_end, map, OT_sumOfSqr, NoArr, 1e2, 1 );

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": taking view " << std::endl;
  }
}

class OverPlaneConstraintManager
{
public:

void groundActivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double duration=n->get<double>();

  //
  const double t_start = phase + 1.0;       // hack: the grasp task lasts 1 step, so we begin one step after
  const double t_end =   komo->maxPhase;
  //

  if( *symbols(0) == "container_0" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 0, 0, 1.0 ) ), OT_eq, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_0", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_0", *symbols(1), 0.05 ), OT_ineq, NoArr, 1e2 );

    activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }
  else if( *symbols(0) == "container_1" )
  {
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 0, 0, 1.0 ) ), OT_eq, NoArr, 1e2 );
    komo->setTask( t_start, t_end, new AxisAlignment( "container_1", ARR( 1.0, 0, 0 ) ), OT_eq, NoArr, 1e2 );

    auto task = komo->setTask( t_start, t_end, new OverPlaneConstraint( komo->world, "container_1", *symbols(1), 0.05 ), OT_ineq, NoArr, 1e2 );

    activeTasks_.push_back( ActiveTask{ komo, symbols, task } );
  }

  if( verbose > 0 )
  {
    std::cout << t_start << "->" << t_end << ": over plane of " << *symbols(0) << " activated" << std::endl;
  }
}

void groundDeactivateOverPlane( double phase, const Graph& facts, Node *n, KOMO * komo, int verbose )
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

};


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

  for( auto s1 : komo->world.getBodyByName( *symbols(0) )->shapes )
  {
    for( auto s2 : komo->world.getBodyByName( *symbols(1) )->shapes )
    {
      //komo->setTask( t_start, t_end, new ShapePairCollisionConstraint( komo->world, s1->name, s2->name, 0.1 ), OT_ineq, NoArr, 1e2 );
    }
  }
}

//===========================================================================

void plan_AOS()
{
  using namespace std::placeholders;

  OverPlaneConstraintManager overPlane;

  auto groundActivateOverPlane = std::bind( &OverPlaneConstraintManager::groundActivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );
  auto groundDeactivateOverPlane = std::bind( &OverPlaneConstraintManager::groundDeactivateOverPlane, &overPlane, _1, _2, _3, _4, _5 );

  // register symbols
  KOMOFactory komoFactory;
  komoFactory.registerTask( "komoGrasp"       , groundGrasp );
  komoFactory.registerTask( "komoGraspObject"       , groundGraspObject );
  komoFactory.registerTask( "komoPlace"       , groundPlace );
  komoFactory.registerTask( "komoGetSight"    , groundGetSight );
  komoFactory.registerTask( "komoTakeView"    , groundTakeView );
  komoFactory.registerTask( "komoActivateOverPlane"   , groundActivateOverPlane );
  komoFactory.registerTask( "komoDeactivateOverPlane" , groundDeactivateOverPlane );
  komoFactory.registerTask( "komoCollisionAvoidance", groundObjectPairCollisionAvoidance );
  //komoFactory.registerTask( "komoHome", groundHome );

  // instanciate search tree
  AOSearch C( komoFactory );
  //C.registerGeometricLevel( GeometricLevelFactoryBase::ptr( new GenericGeometricLevelFactory< PoseLevelType >( komoFactory ) ) );

  //C.prepareFol("LGP-obs-container-fol-place-2.g");
  C.prepareFol("LGP-obs-container-fol-place-pick-2.g");
  C.prepareKin("LGP-obs-container-kin.g");         // parse initial scene LGP-coop-kin.g

  C.prepareDisplay();
  C.prepareTree();      // create root node

  std::set< Policy::ptr, PolicyCompare > policies;

  /////// 1 - Find Initial Policy ////////

  uint i = 0;
  while( ! C.isJointPathSolved() && i < 100 )
  {
    ++i;

    std::cout << "----------" << i << "----------" << std::endl;

    /// SYMBOLIC SEARCH
    C.solveSymbolically();
    //C.addMcRollouts();  // potentially changes the policy not necessary to call it

    if( C.isSymbolicallySolved() )
    {
      {
      // save search tree
      std::stringstream namess;
      namess << "search-" << i << ".gv";
      C.printSearchTree( namess.str() );
      }

      {
      // save policy
      std::stringstream namess;
      namess << "policy-" << i << ".gv";
      C.printPolicy( namess.str() );
      }

      /// GEOMETRIC OPTIMIZATION
      C.solveGeometrically();
    }
  }

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
  mlr::wait( 3000, true );
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
    plan_AOS();
  //}

  return 0;
}
