//#include <pr2/actionMachine.h>
#include "manipSim.h"
#include <Ors/ors.h>

ors::KinematicWorld* world=NULL;

void OrsGraph2RelationalGraph(KeyValueGraph& G, ors::KinematicWorld& W){
  G.clear();

  //do this first to ensure they have the same indexing
  for(ors::Body *b:world->bodies){
    G.append<ors::Body>(STRINGS("body", b->name), b);
  }

  for(ors::Body *b:world->bodies){
    G.append<ors::Transformation>(STRINGS("pose"), ARRAY(G(b->index)), new ors::Transformation(b->X));
//    if(b->ats["ctrlable"]) G.append<bool>(STRINGS("controllable"), ARRAY(G(b->index)), NULL);
    if(b->ats["canGrasp"]) G.append<bool>(STRINGS("canGrasp"), ARRAY(G(b->index)), NULL);
    if(b->ats["fixed"])    G.append<bool>(STRINGS("fixed"), ARRAY(G(b->index)), NULL);
  }

  for(ors::Joint *j:world->joints){
    if(j->type==ors::JT_fixed)
      G.append<bool>(STRINGS("rigid"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
    if(j->type==ors::JT_transXYPhi)
      G.append<bool>(STRINGS("support"), ARRAY(G(j->from->index), G(j->to->index)), NULL);
  }

}

uint Domain::numObjects(){
  return world->bodies.N;
}

void Domain::getInitialState(State &s){
  OrsGraph2RelationalGraph(s.G, *world);
}

//===========================================================================

void sample(){
  ors::KinematicWorld W("model.kvg");
  world = &W;

  //-- fwd expansion
  SearchNodeL T;
  SearchNode *root=new SearchNode(T);
  SearchNode *goal=NULL;

  cout <<"initial state=\n" <<*root <<endl;

  for(uint k=0;k<10;k++){
    SearchNode *n = T(k);
    Action a = n->getRandomFeasibleAction();
    cout <<"random action=" <<a <<endl;
    SearchNode *m = new SearchNode(*n, a);
    m->state.expandReachable();
    cout <<"new state=\n" <<*m <<endl;

    MT::wait();

//      if(checkGoalIsFeasible(s)){
//        goal = T.last();
//        break;
//      }
//    }
  }

  return;
  //backtracking
  SearchNodeL plan = backtrack<SearchNode>(T,goal);
  for(SearchNode *n:plan){
    cout <<"pre-action=" <<n->getPreAction() <<endl;
    cout <<"state=" <<n->getState() <<endl;
  }

}

