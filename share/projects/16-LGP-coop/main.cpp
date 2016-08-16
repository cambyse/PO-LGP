

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
//  cmds={ "1", "0", "5", "0", "3", "0", "4", "0", "s", "x", "q" }; //screwdriver 'hand over'
  cmds={ "m", "m","m","m","q" };
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

void plan_BHTS(){
  Coop C;

  C.prepareKin();
  C.prepareFol();
  C.prepareTree();
  C.prepareDisplay();


  PriorityQueue<ManipulationTree_Node*> pqMC;
  PriorityQueue<ManipulationTree_Node*> pqTerminal;
  PriorityQueue<ManipulationTree_Node*> pqSeq;
  PriorityQueue<ManipulationTree_Node*> pqPath;
  PriorityQueue<ManipulationTree_Node*> pqDone;

  pqMC.add(C.root, 100.);
//  pqSeg.add(C.root, 100.);


  for(;;){

    { //add MC rollouts
      ManipulationTree_Node* n = pqMC.pop();
      n->expand();
      for(ManipulationTree_Node* c:n->children){
        c->addMCRollouts(10,10);
        if(c->symTerminal){
          pqTerminal(c, c->symCost);
        }else{
          pqMC->add(c, c->symCost);
        }
      }

//      n->addMCRollouts(20, 10);
//      n->expandOneActionOnly( n->mc->getBestAction() );

//      if(!n->parent || n->parent->seq.N){ //parent has optimized seq
//        pqSeg.add(n, n->symCost);
//      }
    }

//    { //optimize a seq
//      ManipulationTree_Node* n = pqSeq.pop();
//      if(n){
//        n->solveSeqProblem();
//        if(n->symTerminal && n->seqFeasible){ //this is a symbolic solution
//          pqPath.add(n, n->symCost + n->seqCost);
//        }
//      }
//    }

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
    cout <<"pqMC:" <<pqMC <<endl;
    cout <<"pqSeq:" <<pqSeq <<endl;
    cout <<"pqPath:" <<pqPath <<endl;
    cout <<"pqDone:" <<pqDone <<endl;

  }
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  orsDrawAlpha = 1.;
//  orsDrawCores = true;
  test();

  return 0;
}
