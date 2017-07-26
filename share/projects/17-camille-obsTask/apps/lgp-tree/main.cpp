#include <functional>
#include <list>

#include <policy.hpp>

#include <task_planner.hpp>
#include <policy_builder.hpp>

#include <motion_planner.hpp>

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

void plan_AOS()
{
  // task planning
  tp::TaskPlanner tp;
  tp.setFol( "LGP-obs-container-fol-place-pick-2.g" );
  tp.solve();
  auto policy = tp.getPolicy();

  uint i = 0;
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

  // motion planning
  mp::MotionPlanner mp;
  mp.setKin( "LGP-obs-container-kin.g" );
  mp.inform( policy );

  // instanciate search tree
  /*AOSearch C( komoFactory );
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
    plan_AOS();
  //}

  return 0;
}
