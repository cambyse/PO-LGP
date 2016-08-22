

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
//  C.displayTree();

  StringA cmds={ "p", "0", "3", "1"};//, "p", "4", "p", "s", "q" };
//  cmds={ "1", "1", "0", "x", "q" };
  cmds={ "1", "0", "5", "0", "3", "0", "4", "0", "s", "x", "q" }; //screwdriver 'hand over'
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

MNode* popBestFromMCfringe(mlr::Array<MNode*>& fringe){
  MNode* best=NULL;
  for(MNode* n:fringe)
    if(!best || n->symCost+n->costSoFar < best->symCost+best->costSoFar) best=n;
  fringe.removeValue(best);
  return best;
}

MNode* popBestFromSeqFringe(mlr::Array<MNode*>& fringe){
  MNode* best=NULL;
  for(MNode* n:fringe)
    if(!best || n->symCost+n->costSoFar < best->symCost+best->costSoFar) best=n;
  fringe.removeValue(best);
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


  mlr::Array<ManipulationTree_Node*> MCfringe;
  mlr::Array<ManipulationTree_Node*> terminals;
  mlr::Array<ManipulationTree_Node*> seqFringe;
  mlr::Array<ManipulationTree_Node*> pathFringe;
  mlr::Array<ManipulationTree_Node*> pqDone;

  MCfringe.append(C.root);
  seqFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  for(;;){

    { //add MC rollouts
      ManipulationTree_Node* n = popBestFromMCfringe(MCfringe);
      n->expand();
      for(ManipulationTree_Node* c:n->children){
        c->addMCRollouts(10,10);
        if(!c->symTerminal) MCfringe.append(c);
        else terminals.append(c);
        if(n->seq.N) seqFringe.append(n);
      }
    }

    C.updateDisplay();

    { //optimize a seq
      MNode* n = popBestFromSeqFringe(seqFringe);
      if(n){
        n->solveSeqProblem();
        setAllChildCostSoFar(n, n->seqCost);
        if(n->seqFeasible) for(MNode* c:n->children) seqFringe.append(c);
        if(n->seqFeasible && n->symTerminal) pathFringe.append(n);
      }
    }

    C.updateDisplay();
//    { //optimize a path
//      ManipulationTree_Node* n = pqPath.pop();
//      if(n){
//        n->solvePathProblem();
//        if(n->symTerminal && n->pathFeasible){ //this is a symbolic solution
//          pqDone.add(n, n->symCost + n->pathCost);
//        }
//      }
//    }

    cout <<"===================== CURRENT QUEUES:" <<endl;
    cout <<"MCfringe:" <<MCfringe <<endl;
    cout <<"seqFringe:" <<seqFringe <<endl;
    cout <<"pathFringe:" <<pathFringe <<endl;
//    cout <<"pqDone:" <<pqDone <<endl;

  }
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha = 1.;
//  orsDrawCores = true;
//  test();
  plan_BHTS();

  return 0;
}
