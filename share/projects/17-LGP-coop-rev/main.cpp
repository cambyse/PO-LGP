

#include "code.h"
#include <MCTS/solver_PlainMC.h>

//===========================================================================

typedef ManipulationTree_Node MNode;
typedef ManipulationTree_NodeL MNodeL;

double astarHeuristic(MNode* n){
  return n->cost(0) + n->h(0);
}

double poseHeuristic(MNode* n){
  return n->cost(0) + n->h(0); //symCost;
}

double mcHeuristic(MNode* n){
  if(n->count(1)) return -10.+n->cost(1);
  return 1.;
}

double seqHeuristic(MNode* n){
  return n->cost(0) + n->h(0); //symCost;
}

double poseCost(MNode* n){
  if(!n->count(1) || !n->feasible(1)) return 100.;
  return .1*(n->cost(0)+n->h(0)) + n->cost(1);
}

double seqCost(MNode* n){
  if(!n->count(2) || !n->feasible(2)) return 100.;
  return .1*(n->cost(0)+n->h(0))+n->cost(2);
}

double pathHeuristic(MNode* n){
  return seqCost(n);
}

double pathCost(MNode* n){
  if(!n->path.N || !n->feasible(3)) return 100.;
  return .1*(n->cost(0)+n->h(0)) + n->cost(2) + n->cost(3);
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

//===========================================================================

void plan_BHTS(){
  Coop C;

  C.prepareFol(true);
  C.prepareKin();
  C.prepareTree();
  C.prepareDisplay();

//  C.mcFringe.append(C.root);
  C.astar.append(C.root);

  C.poseFringe.append(C.root);
//  C.poseQueue.add(0., C.root);
//  C.seqFringe.append(C.root);

  C.updateDisplay();
  C.displayTree();

  ofstream fil("z.dat");

  for(uint k=0;k<100;k++){


//    C.root->checkConsistency();
    { //expand

      if(!C.astar.N){
        LOG(-2) <<"AStar: queue is empty -> failure?";
      }
//      MNode *n =  C.astar.pop();
      MNode *n =  popBest(C.astar, astarHeuristic);
      CHECK(n,"");
      n->expand();
      for(ManipulationTree_Node* ch:n->children){
        C.astar.append(ch); //add(ch->g(0) + ch->h(0), ch, true);
        if(ch->isTerminal){
          C.terminals.append(ch);
          MNodeL path = ch->getTreePath();
          for(MNode *n:path) if(!n->count(1))
//            C.pose2Queue.add(0., n, true); //pose2 is a FIFO
            C.pose2Fringe.append(n); //pose2 is a FIFO
        }
//        if(n->poseCount) C.poseQueue.add(poseHeuristic(ch), ch, true);
        if(n->count(1)) C.poseFringe.append(ch);
//          if(n->seqCount) C.seqFringe.append(c);
        //if(c->isTerminal) C.seqFringe.append(c);
      }
    }

    if(rnd.uni()<.1){ //optimize a pose
//      MNode *n =  C.poseQueue.pop();
      MNode* n = popBest(C.poseFringe, poseHeuristic);
      if(!n->count(1)){
        n->solvePoseProblem();
        if(n->feasible(1)){
          for(MNode* c:n->children)
//            C.poseQueue.add(poseHeuristic(c), c, true); //test all children
            C.poseFringe.append(c); //test all children
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    if(C.pose2Fringe.N){
//      MNode *n =  C.pose2Queue.pop();
      MNode *n =  C.pose2Fringe.popFirst();
      if(n){
        n->solvePoseProblem();
        if(n->feasible(1)){
          if(n->isTerminal) C.seqFringe.append(n); //test seq or path
        }
        C.node = n;
      }
    }

    { //optimize a seq
      MNode* n = popBest(C.seqFringe, seqHeuristic);
      if(n){
        n->solveSeqProblem();
        if(n->feasible(2)){
          if(n->isTerminal) C.pathFringe.append(n);
        }
        C.node = n;
      }
    }

    { //optimize a path
      MNode* n = popBest(C.pathFringe, pathHeuristic);
      if(n){
        n->solvePathProblem(10);
        if(n->feasible(3)) C.done.append(n);
        C.node = n;
      }
    }

    for(auto *n:C.terminals) CHECK(n->isTerminal,"");

    //-- update queues (if something got infeasible)
    for(uint i=C.astar.N;i--;) if(C.astar.elem(i)->isInfeasible) C.astar.remove(i);
    for(uint i=C.poseFringe.N;i--;) if(C.poseFringe.elem(i)->isInfeasible) C.poseFringe.remove(i);
    for(uint i=C.pose2Fringe.N;i--;) if(C.pose2Fringe.elem(i)->isInfeasible) C.pose2Fringe.remove(i);
    for(uint i=C.seqFringe.N;i--;) if(C.seqFringe.elem(i)->isInfeasible) C.seqFringe.remove(i);
    for(uint i=C.pathFringe.N;i--;) if(C.pathFringe.elem(i)->isInfeasible) C.pathFringe.remove(i);

//    for(auto& q:C.astar) q.p=q.x->f();
//    C.astar.sort();


    FILE("z.fol") <<C.fol;

//    C.updateDisplay();
//    for(MNode *n:C.mcFringe) if(!n->mcStats->n){
////      cout <<"recomputing MC rollouts for: " <<*n->decision <<endl;
////      mlr::wait();
////      C.root->rootMC->verbose = 2;
//      n->addMCRollouts(10,10);
////      C.updateDisplay();
//    }

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
    mlr::wait();

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
