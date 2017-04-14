#include <MCTS/solver_PlainMC.h>

#include "ao_search.h"

#include <observation_tasks.h>

/*
back track, take history into account?
sort nodes before expanding?
labelInfeasible()
back track result of pose computation when one of the pose is not possible or generally different between worlds!
less rollouts?
iterations
build kinematic world for optimization, with all believed objects inserted as inert objects for colision avoidance at least!

dot -Tpng -o policy.png policy.gv
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

void groundGrasp( double phase, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double time=n->get<double>();

  komo.setGrasp( phase+time, *symbols(0), *symbols(1), verbose);
}

void groundPlace( double phase, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double time=n->get<double>();

  komo.setPlace( phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
}

void groundHandover( double phase, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double time=n->get<double>();

  komo.setHandover( phase+time, *symbols(0), *symbols(1), *symbols(2), verbose);
}

void groundAttach( double phase, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double time=n->get<double>();

  Node *attachableSymbol = facts.getNode("attachable");
  CHECK(attachableSymbol!=NULL,"");
  Node *attachableFact = facts.getEdge({attachableSymbol, n->parents(1), n->parents(2)});
  mlr::Transformation rel = attachableFact->get<mlr::Transformation>();
  komo.setAttach( phase+time, *symbols(0), *symbols(1), *symbols(2), rel, verbose);
}

void groundGetSight( double phase, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
  StringL symbols;
  for(Node *p:n->parents) symbols.append(&p->keys.last());

  double time=n->get<double>();

  mlr::String arg = *symbols(0);

  if( arg == "target_location_0" )  // tmp, pivot point and object location has to be deduced from the scene!!
    komo.setTask( phase+time, phase+time + 1.0, new HeadGetSight( ARR(  0.0, -1.0, 1.9 ),    // object position
                                                      ARR( -0.2, -0.6, 1.9 ) ),  // pivot position
                                                      OT_sumOfSqr, NoArr, 1e2 );
  else if( arg == "target_location_1" ) // tmp, pivot point and object location has to be deduced from the scene!!!
    komo.setTask( phase+time, phase+time + 1.0, new HeadGetSight( ARR( -1.0,  0.0, 1.9 ),    // object position
                                                      ARR( -0.6, -0.2, 1.9 ) ),  // pivot position
                                                      OT_sumOfSqr, NoArr, 1e2 );
  else if( arg == "target_location_2" ) // tmp, pivot point and object location has to be deduced from the scene!!!
    komo.setTask( phase+time, phase+time + 1.0, new HeadGetSight( ARR( 1.0,  0.0, 1.9 ),    // object position
                                                      ARR( 0.6, -0.2, 1.9 ) ),  // pivot position
                                                      OT_sumOfSqr, NoArr, 1e2 );

}

void groundTakeView( double, const Graph& facts, Node *n, KOMO & komo, int verbose )
{
}

//===========================================================================

/*void plan_BHTS()
{
  // register symbols
  KOMOFactory komoFactory;
  komoFactory.registerTask( "komoGrasp"       , groundGrasp );
  komoFactory.registerTask( "komoPlace"       , groundPlace );
  komoFactory.registerTask( "komoHandover"    , groundHandover );
  komoFactory.registerTask( "komoAttach"      , groundAttach );
  komoFactory.registerTask( "komoGetSight"    , groundGetSight );
  komoFactory.registerTask( "komoTakeView"    , groundTakeView );

  // instanciate search tree
  SearchSpaceTree C( komoFactory );

  //C.fol.verbose = 5;
//  C.prepareFol("LGP-obs-fol.g");
//  C.prepareKin("LGP-obs-kin.g");

  C.prepareFol("LGP-obs-fol-2-simple.g");        // with two candidate positions
  C.prepareKin("LGP-obs-kin-2.g");

  //C.prepareFol("LGP-coop-fol.g");
  //C.prepareKin("LGP-coop-kin.g");       // parse initial scene LGP-coop-kin.g

  C.prepareTree();      // create root node
  C.prepareDisplay();

  //  C.kin.watch(true);
  //  mlr::wait();

  // algo starts here

  C.mcFringe.append(C.root);
  C.poseFringe.append(C.root);
  //  C.seqFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  ofstream fil("z.dat");

  for(uint k=0;k<100;k++)
  {
    iterate( C, fil );
  }
  fil.close();

  C.pathView.writeToFiles=false; // camille
  C.updateDisplay();
  mlr::wait(.1);
  //mlr::wait();
}*/

void plan_AOS()
{
  // register symbols
  KOMOFactory komoFactory;
  komoFactory.registerTask( "komoGrasp"       , groundGrasp );
  komoFactory.registerTask( "komoPlace"       , groundPlace );
  komoFactory.registerTask( "komoHandover"    , groundHandover );
  komoFactory.registerTask( "komoAttach"      , groundAttach );
  komoFactory.registerTask( "komoGetSight"    , groundGetSight );
  komoFactory.registerTask( "komoTakeView"    , groundTakeView );

  // instanciate search tree
  AOSearch C( komoFactory );
  C.prepareFol("LGP-obs-fol-3-simple.g");        // with two candidate positions
  C.prepareKin("LGP-obs-kin-3.g");

  //C.prepareFol("LGP-coop-fol.g");
  //C.prepareKin("LGP-coop-kin.g");         // parse initial scene LGP-coop-kin.g

  C.prepareDisplay();

  C.prepareTree();      // create root node

  uint i = 0;
  while( ! C.isJointPathSolved() && i < 10 )
  {
    ++i;
    /// SYMBOLIC SEARCH
    C.solveSymbolically();

    if( C.isSymbolicallySolved() )
    {
      /// POSE OPTIMIZATION
      C.optimizePoses();      // optimizes poses of the current best solution

      if( C.isPoseSolved() )
      {
        /// SEQUENCE OPTIMIZATION
        C.optimizeSequences();  // optimizes sequences of the current best solution

        if( C.isSequenceSolved() )
        {
          /// PATH OPTIMIZATION
          C.optimizePaths();      // optimizes paths of the current best solution

          if( C.isPathSolved() )
          {
            /// JOINT PATH OPTIMIZATION
            C.optimizeJointPaths();   // optimizes joint paths of the current best solution
          }
        }
      }
    }
  }

  // save policy
  std::stringstream ss;
  C.printPolicy( ss );

  // save to file
  std::ofstream fs;
  fs.open( "policy.gv" );
  fs << ss.str();
  fs.close();

  // display
  C.updateDisplay( WorldID( -1 ), false, false, true );
  mlr::wait( 60 );
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

  orsDrawAlpha = 1.;
  orsDrawJoints=orsDrawMarkers=false;
  //  orsDrawCores = true;
  //if(mlr::getParameter<bool>("intact")){
  //  test();
  //}else{
    //    test();
    plan_AOS();
  //}

  return 0;
}
