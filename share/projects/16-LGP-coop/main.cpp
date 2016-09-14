

#include "code.h"
#include <Algo/priorityQueue.h>
#include <MCTS/solver_PlainMC.h>

//===========================================================================

void test(){
  Coop C;

  C.prepareKin();
  C.prepareFol();
  C.prepareTree();
  C.prepareDisplay();

//  C.fol.verbose=5;

  C.expandNode();
  FILE("z.fol") <<C.fol;
  C.root->checkConsistency();
  C.root->recomputeAllFolStates();
  FILE("z2.fol") <<C.fol;
  C.root->checkConsistency();

//  C.root->write(cout, true);

//  C.displayTree();

  StringA cmds={ "p", "0", "3", "1"};//, "p", "4", "p", "s", "q" };
//  cmds={ "1", "1", "0", "x", "q" };
//  cmds={ "1", "0", "3", "0", "3", "0", "4", "0", "x", "s", "q" }; //screwdriver 'hand over'
  cmds={ "0", "2", "2", "3", "x", "s", "q" }; //screwdriver 'hand over'
//  cmds={ "1", "s", "q" }; //screwdriver 'hand over'
//  cmds={ "m", "m","m","m","q" };
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
      if(!C.execChoice(cmds(s))) break;
    }
  }

  mlr::wait();
  threadCloseModules();
  cout <<"BYE BYE" <<endl;
}

//===========================================================================

typedef ManipulationTree_Node MNode;


double seqHeuristic(MNode* n){
  return n->symCost;
  return n->symCost+n->costSoFar;
}

double seqCost(MNode* n){
  if(!n->seq.N) return 100.;
  if(!n->seqFeasible) return 100.;
  return n->symCost+n->seqCost;
}

double pathHeuristic(MNode* n){
  return seqCost(n);
}

double pathCost(MNode* n){
  if(!n->path.N) return 100.;
  if(!n->pathFeasible) return 100.;
  return n->symCost + n->seqCost + n->pathCost;
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
  best->inFringe2=false;
  return best;
}

void setAllChildCostSoFar(MNode* n, double x){
  n->costSoFar = x;
  for(MNode *c: n->children) setAllChildCostSoFar(c,x);
}

void plan_BHTS(){
  Coop C;

  C.prepareKin();
  C.prepareFol();
  C.prepareTree();
  C.prepareDisplay();

//  C.MCfringe.append(C.root);
  C.seqFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  for(uint k=0;k<100;k++){


//    C.root->checkConsistency();
    { //expand
      ManipulationTree_Node* n = NULL;
      for(uint k=0;k<10;k++){ n=C.root->treePolicy_softMax(2.); if(n) break; }
      if(n){
        n->expand();
        for(ManipulationTree_Node* c:n->children){
          c->addMCRollouts(10,10);
          if(c->isTerminal) C.terminals.append(c);
          if(n->seq.N) C.seqFringe.append(c);
        }
      }
    }

    { //add MC rollouts
      for(uint mc=0;mc<10;mc++){
        ManipulationTree_Node* n = NULL;
        for(uint k=0;k<10;k++){ n=C.root->treePolicy_random(); if(n) break; }
        if(n){
          n->addMCRollouts(10,10);
        }
      }
    }

    C.root->recomputeAllMCStats();

//    C.updateDisplay();

    { //optimize a seq
      MNode* n = popBest(C.seqFringe, seqHeuristic);
      if(n){
        //      cout <<"### SEQ TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solveSeqProblem();
        setAllChildCostSoFar(n, n->seqCost);
        if(n->seqFeasible) for(MNode* c:n->children) C.seqFringe.append(c);
        if(n->seqFeasible && n->isTerminal) C.pathFringe.append(n);
        C.node = n;
      }
    }

    { //optimize a path
      MNode* n = popBest(C.pathFringe, pathHeuristic);
      if(n){
        //      cout <<"### PATH TESTING node " <<*n <<endl;
        //      mlr::wait();
        n->solvePathProblem(10);
        setAllChildCostSoFar(n, n->pathCost);
        if(n->pathFeasible) C.done.append(n);
        C.node = n;
      }
    }

    for(auto *n:C.terminals) CHECK(n->isTerminal,"");

    C.updateDisplay();
    MNode *bt = getBest(C.terminals, seqCost);
    MNode *bp = getBest(C.done, pathCost);
    cout <<"EVALS = " <<COUNT_evals <<" SEQ= " <<COUNT_seqOpt <<" PATH= " <<COUNT_pathOpt
           <<" bestSeq= " <<(bt?seqCost(bt):100.)
          <<" pathSeq= " <<(bp?pathCost(bp):100.)
         <<" #solutions= " <<C.done.N <<endl;

    if(bt) C.node=bt;
    if(bp) C.node=bp;
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
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha = 1.;
//  orsDrawCores = true;
  if(mlr::getParameter<bool>("intact")){
    test();
  }else{
//    test();
    plan_BHTS();
  }

  return 0;
}
