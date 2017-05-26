

#include "code.h"
#include <MCTS/solver_PlainMC.h>

//===========================================================================

typedef ManipulationTree_Node MNode;
typedef ManipulationTree_NodeL MNodeL;

MNode* getBest(mlr::Array<MNode*>& fringe, uint level){
  if(!fringe.N) return NULL;
  MNode* best=NULL;
  for(MNode* n:fringe){
    if(n->isInfeasible || !n->count(level)) continue;
    if(!best || (n->feasible(level) && n->cost(level)<best->cost(level))) best=n;
  }
  return best;
}

MNode* popBest(mlr::Array<MNode*>& fringe, uint level){
  if(!fringe.N) return NULL;
  MNode* best=getBest(fringe, level);
  fringe.removeValue(best);
  return best;
}

//===========================================================================

void plan_BHTS(){
  Coop C;

  C.prepareFol(true);
  C.prepareKin();
  C.prepareTree();
  C.prepareDisplay();

  C.expandFringe.append(C.root);

  C.poseFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  ofstream fil("z.dat");

  for(uint k=0;k<1000;k++){


    { //expand

      MNode *n =  popBest(C.expandFringe, 0);
      CHECK(n,"");
      n->expand();
      for(MNode* ch:n->children){
        if(ch->isTerminal){
          C.terminals.append(ch);
          MNodeL path = ch->getTreePath();
          for(MNode *n:path) if(!n->count(1)) C.pose2Fringe.append(n); //pose2 is a FIFO
        }else{
          C.expandFringe.append(ch);
        }
        if(n->count(1)) C.poseFringe.append(ch);
      }
    }

    if(rnd.uni()<.5){ //optimize a pose
      MNode* n = popBest(C.poseFringe, 0);
      if(n && !n->count(1)){
        n->optLevel(1);
        if(n->feasible(1)){
          for(MNode* c:n->children) C.poseFringe.append(c); //test all children
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    if(C.pose2Fringe.N){
      MNode *n =  C.pose2Fringe.popFirst();
      if(n && !n->count(1)){
        n->optLevel(1);
        if(n->feasible(1)){
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    { //optimize a seq
      MNode* n = popBest(C.seqFringe, 1);
      if(n && !n->count(2)){
        n->optLevel(2);
        if(n->feasible(2)){
          if(n->isTerminal) C.pathFringe.append(n);
        }
        C.node = n;
      }
    }

    { //optimize a path
      MNode* n = popBest(C.pathFringe, 2);
      if(n && !n->count(3)){
        n->optLevel(3);
        if(n->feasible(3)) C.done.append(n);
        C.node = n;
      }
    }

    for(auto *n:C.terminals) CHECK(n->isTerminal,"");

    //-- update queues (if something got infeasible)
    for(uint i=C.expandFringe.N;i--;) if(C.expandFringe.elem(i)->isInfeasible) C.expandFringe.remove(i);
    for(uint i=C.poseFringe.N;i--;) if(C.poseFringe.elem(i)->isInfeasible) C.poseFringe.remove(i);
    for(uint i=C.pose2Fringe.N;i--;) if(C.pose2Fringe.elem(i)->isInfeasible) C.pose2Fringe.remove(i);
    for(uint i=C.seqFringe.N;i--;) if(C.seqFringe.elem(i)->isInfeasible) C.seqFringe.remove(i);
    for(uint i=C.pathFringe.N;i--;) if(C.pathFringe.elem(i)->isInfeasible) C.pathFringe.remove(i);
    for(uint i=C.terminals.N;i--;) if(C.terminals.elem(i)->isInfeasible) C.terminals.remove(i);


    FILE("z.fol") <<C.fol;

    MNode *bpose = getBest(C.terminals, 1);
    MNode *bseq  = getBest(C.terminals, 2);
    MNode *bpath = getBest(C.done, 3);
    mlr::String out;
    out <<"TIME= " <<mlr::cpuTime() <<" KIN= " <<COUNT_kin <<" EVALS= " <<COUNT_evals
       <<" POSE= " <<COUNT_opt(1) <<" SEQ= " <<COUNT_opt(2) <<" PATH= " <<COUNT_opt(3)
      <<" bestPose= " <<(bpose?bpose->f(1):100.)
      <<" bestSeq = " <<(bseq ?bseq ->f(2):100.)
      <<" pathPath= " <<(bpath?bpath->f(3):100.)
      <<" #solutions= " <<C.done.N;

    fil <<out <<endl;
    cout <<out <<endl;

    if(bseq) C.node=bseq;
    if(bpath) C.node=bpath;
    if(!(k%10)) C.updateDisplay();
//    mlr::wait();

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

//  rnd.clockSeed();

  plan_BHTS();

  return 0;
}
