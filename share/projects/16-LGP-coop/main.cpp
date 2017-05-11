

#include "code.h"
#include <Algo/priorityQueue.h>
#include <MCTS/solver_PlainMC.h>

//===========================================================================

void test(){
  Coop C;

  C.prepareFol();
  C.prepareKin();
  C.prepareTree();
  C.prepareDisplay();

  C.pathView.writeToFiles=true;
//  C.fol.verbose=5;

  C.expandNode();
  FILE("z.fol") <<C.fol;
  C.root->checkConsistency();
  C.root->recomputeAllFolStates();
  FILE("z2.fol") <<C.fol;
  C.root->checkConsistency();

//  C.root->write(cout, true);

//  C.displayTree();

  StringA cmds={ "p", "0", "p", "3", "p", "1"};//, "p", "4", "p", "s", "q" };
//  cmds={ "1", "1", "0", "x", "q" };
//  cmds={ "1", "0", "3", "0", "3", "0", "4", "0", "x", "s", "q" }; //screwdriver 'hand over'
  cmds={ "0", "2", "2", "3", "x", "s", "q" }; //screwdriver 'hand over'
//  cmds={ "0", "2", "5", "x", "s", "q" }; //screwdriver 'hand over'
//  cmds={ "1", "s", "q" }; //screwdriver 'hand over'
//  cmds={ "m", "m","m","m","q" };
  cmds={ "p", "0", "p", "12", "p", "1", "p", "15", "p", "2", "15", "p", "s", "x" };

  cmds={ "1", "2", "6", "9", "6", "0", "4", "5", "12", "2", "12", "s", "x" };

  bool interactive = mlr::getParameter<bool>("intact", false);
  bool random = mlr::getParameter<bool>("random", false);

  for(uint s=0;;s++){
    C.updateDisplay();
    C.printChoices();

    if(interactive){
      mlr::String cmd = C.queryForChoice();
      if(!C.execChoice(cmd)) break;
    }else if(random){
      if(!C.execRandomChoice()) break;
    }else{
      if(s>=cmds.N) break;
      if(!C.execChoice(cmds(s))) break;
    }
  }

  mlr::wait();
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

typedef ManipulationTree_Node MNode;


double poseHeuristic(MNode* n){
  return n->symCost;
}

double mcHeuristic(MNode* n){
  if(n->poseCount) return -10.+n->poseCost;
  return 1.;
}

double seqHeuristic(MNode* n){
  return n->symCost;
}

double poseCost(MNode* n){
  if(!n->poseCount || !n->poseFeasible) return 100.;
  return .1*n->symCost+n->poseCost;
}

double seqCost(MNode* n){
  if(!n->seqCount || !n->seqFeasible) return 100.;
  return .1*n->symCost+n->seqCost;
}

double pathHeuristic(MNode* n){
  return seqCost(n);
}

double pathCost(MNode* n){
  if(!n->path.N || !n->pathFeasible) return 100.;
  return .1*n->symCost + n->seqCost + n->pathCost;
}

MNode* getBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*)){
  if(!fringe.N) return NULL;
  MNode* best=NULL;
  for(MNode* n:fringe)
    if(!best || heuristic(n)<heuristic(best)) best=n;
  return best;
}

MNode* popBest(mlr::Array<MNode*>& fringe, double heuristic(MNode*)){
  if(!fringe.N) return NULL;
  MNode* best=getBest(fringe, heuristic);
  fringe.removeValue(best);
  return best;
}

void plan_BHTS(){
  Coop C;

  C.prepareFol(true);
  C.prepareKin();
  C.prepareTree();
  C.prepareDisplay();

//  C.kin.watch(true);
//  mlr::wait();

  C.mcFringe.append(C.root);
  C.poseFringe.append(C.root);
//  C.seqFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  ofstream fil("z.dat");

  for(uint k=0;k<100;k++){


//    C.root->checkConsistency();
    { //expand
      MNode* n = popBest(C.mcFringe, mcHeuristic);
      //      ManipulationTree_Node* n = NULL;
//      for(uint k=0;k<10;k++){ n=C.root->treePolicy_softMax(0.); if(n) break; }
      if(n){
        n->expand();
        for(ManipulationTree_Node* c:n->children){
          c->addMCRollouts(10,10);
          C.mcFringe.append(c);
          if(c->isTerminal) C.terminals.append(c);
          if(n->poseCount) C.poseFringe.append(c);
//          if(n->seqCount) C.seqFringe.append(c);
          //if(c->isTerminal) C.seqFringe.append(c);
        }
      }
    }

    { //add MC rollouts
      for(uint mc=0;mc<10;mc++){
        ManipulationTree_Node* n = NULL;
        for(uint k=0;k<10;k++){ n=C.root->treePolicy_random(); if(n) break; }
        if(n){
          n->addMCRollouts(2,10);
        }
      }
    }

    C.root->recomputeAllMCStats();

//    C.updateDisplay();

    { //optimize a pose
      MNode* n = popBest(C.poseFringe, poseHeuristic);
      if(n){
        //      cout <<"### POSE TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solvePoseProblem();
        if(n->poseFeasible){
          for(MNode* c:n->children) C.poseFringe.append(c); //test all children
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    { //optimize a seq
      MNode* n = popBest(C.seqFringe, seqHeuristic);
      if(n){
        //      cout <<"### SEQ TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solveSeqProblem();
        if(n->seqFeasible){
//          for(MNode* c:n->children) C.seqFringe.append(c);
          if(n->isTerminal) C.pathFringe.append(n);
        }
        C.node = n;
      }
    }

    { //optimize a path
      MNode* n = popBest(C.pathFringe, pathHeuristic);
      if(n){
        //      cout <<"### PATH TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solvePathProblem(10);
        if(n->pathFeasible) C.done.append(n);
        C.node = n;
      }
    }

    for(auto *n:C.terminals) CHECK(n->isTerminal,"");

//    C.updateDisplay();
    for(MNode *n:C.mcFringe) if(!n->mcStats->n){
//      cout <<"recomputing MC rollouts for: " <<*n->decision <<endl;
//      mlr::wait();
//      C.root->rootMC->verbose = 2;
      n->addMCRollouts(10,10);
//      C.updateDisplay();
    }

    MNode *bt = getBest(C.terminals, seqCost);
    MNode *bp = getBest(C.done, pathCost);
    mlr::String out;
    out <<"TIME= " <<mlr::cpuTime() <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals
       <<" POSE= " <<COUNT_poseOpt <<" SEQ= " <<COUNT_seqOpt <<" PATH= " <<COUNT_pathOpt
      <<" bestPose= " <<(bt?poseCost(bt):100.)
     <<" bestSeq= " <<(bt?seqCost(bt):100.)
          <<" pathSeq= " <<(bp?pathCost(bp):100.)
         <<" #solutions= " <<C.done.N;

    fil <<out <<endl;
    cout <<out <<endl;

    if(bt) C.node=bt;
    if(bp) C.node=bp;
    C.updateDisplay();
//    mlr::wait();

//    { //optimize a path
//      ManipulationTree_Node* n = pqPath.pop();
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

  }
  fil.close();

  C.pathView.writeToFiles=true;
  C.updateDisplay();
  mlr::wait(.1);
  //mlr::wait();

}

//===========================================================================

void plan_MBTS(){
  Coop C;

  C.prepareFol(true);
  C.prepareKin();
  C.prepareAStar();
  C.prepareDisplay();

  C.updateDisplay();
  C.displayTree();

  ofstream fil("z.dat");

  for(uint k=0;k<100;k++){
    { //expand
      MNode* n = popBest(C.mcFringe, mcHeuristic);
      //      ManipulationTree_Node* n = NULL;
//      for(uint k=0;k<10;k++){ n=C.root->treePolicy_softMax(0.); if(n) break; }
      if(n){
        n->expand();
        for(ManipulationTree_Node* c:n->children){
          c->addMCRollouts(10,10);
          C.mcFringe.append(c);
          if(c->isTerminal) C.terminals.append(c);
          if(n->poseCount) C.poseFringe.append(c);
//          if(n->seqCount) C.seqFringe.append(c);
          //if(c->isTerminal) C.seqFringe.append(c);
        }
      }
    }

    { //add MC rollouts
      for(uint mc=0;mc<10;mc++){
        ManipulationTree_Node* n = NULL;
        for(uint k=0;k<10;k++){ n=C.root->treePolicy_random(); if(n) break; }
        if(n){
          n->addMCRollouts(2,10);
        }
      }
    }

    C.root->recomputeAllMCStats();

//    C.updateDisplay();

    { //optimize a pose
      MNode* n = popBest(C.poseFringe, poseHeuristic);
      if(n){
        //      cout <<"### POSE TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solvePoseProblem();
        if(n->poseFeasible){
          for(MNode* c:n->children) C.poseFringe.append(c); //test all children
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    { //optimize a seq
      MNode* n = popBest(C.seqFringe, seqHeuristic);
      if(n){
        //      cout <<"### SEQ TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solveSeqProblem();
        if(n->seqFeasible){
//          for(MNode* c:n->children) C.seqFringe.append(c);
          if(n->isTerminal) C.pathFringe.append(n);
        }
        C.node = n;
      }
    }

    { //optimize a path
      MNode* n = popBest(C.pathFringe, pathHeuristic);
      if(n){
        //      cout <<"### PATH TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solvePathProblem(10);
        if(n->pathFeasible) C.done.append(n);
        C.node = n;
      }
    }

    for(auto *n:C.terminals) CHECK(n->isTerminal,"");

//    C.updateDisplay();
    for(MNode *n:C.mcFringe) if(!n->mcStats->n){
//      cout <<"recomputing MC rollouts for: " <<*n->decision <<endl;
//      mlr::wait();
//      C.root->rootMC->verbose = 2;
      n->addMCRollouts(10,10);
//      C.updateDisplay();
    }

    MNode *bt = getBest(C.terminals, seqCost);
    MNode *bp = getBest(C.done, pathCost);
    mlr::String out;
    out <<"TIME= " <<mlr::cpuTime() <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals
       <<" POSE= " <<COUNT_poseOpt <<" SEQ= " <<COUNT_seqOpt <<" PATH= " <<COUNT_pathOpt
      <<" bestPose= " <<(bt?poseCost(bt):100.)
     <<" bestSeq= " <<(bt?seqCost(bt):100.)
          <<" pathSeq= " <<(bp?pathCost(bp):100.)
         <<" #solutions= " <<C.done.N;

    fil <<out <<endl;
    cout <<out <<endl;

    if(bt) C.node=bt;
    if(bp) C.node=bp;
    C.updateDisplay();
//    mlr::wait();

//    { //optimize a path
//      ManipulationTree_Node* n = pqPath.pop();
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

  }
  fil.close();

  C.pathView.writeToFiles=true;
  C.updateDisplay();
  mlr::wait(.1);
  //mlr::wait();

}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  rnd.clockSeed();

//  orsDrawAlpha = 1.;
//  orsDrawJoints=orsDrawMarkers=false;
//  orsDrawCores = true;
  if(mlr::getParameter<bool>("intact")){
    test();
  }else{
//    test();
    plan_BHTS();
  }

  return 0;
}
