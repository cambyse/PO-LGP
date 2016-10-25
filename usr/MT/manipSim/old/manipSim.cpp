#include "manipSim.h"

Domain domain;

enum Predicate : uint{ rigid=0, support, trans2DPhi, _N_Predicate };
const char* predicateString(uint p){
  static const char* bla[]={"rigid", "support", "trans2DPhi", "no"};
  return bla[p];
}

const char* actionPredicateString(ActionPredicate a){
  static const char* bla[]={"create_", "break_", "no_"};
  return bla[a];
}

Action& NoAction = *((Action*)NULL);
SearchNode& NoSearchNode = *((SearchNode*)NULL);

void Action::setRandom(uint n){
  i=rnd(n);
  j=rnd(n-1);  if(j>=i) j++;
  a=(ActionPredicate)rnd(_N_ActionPredicate);
  p=predicateString(rnd(_N_Predicate));
}

void State::compControllable(){
//  NodeL objs = G.getNodes("body");

//  cout <<G <<endl;
  NodeL ctrltags = G.getNodes("controllable");
  for_list_rev(Node, i, ctrltags) delete i;

  ctrltags = G.getNodes("canGrasp");
  NodeL ctrlables;

  for(Node *o:ctrltags) ctrlables.append(o->parents(0));

  for(Node *o:ctrlables){
    for(Node *r:o->parentOf){
      if(r->keys(0)=="rigid"){
        if(r->parents(0)==o)
          ctrlables.setAppend(r->parents(1));
        else
          ctrlables.setAppend(r->parents(0));
      }
    }
  }

  for(Node *o:ctrlables){
    bool hasTag=false;
    for(Node *r:o->parentOf) if(r->keys(0)=="controllable"){ hasTag=true; break; }
    if(!hasTag) G.newNode<bool>({"controllable"}, {o}, NULL, false);
  }
//  cout <<G <<endl;
}

void State::expandReachable(){
}

bool State::testAction(const Action& a, bool apply){
  bool applicable=false;
  Node *o1 = G(a.i);
  Node *o2 = G(a.j);
  if(a.a==create_){
    if(!o1->ParentOf()["controllable"]) return false;
    if(a.p=="rigid"){
      if(!o1->ParentOf()["canGrasp"]) return false;
    }
    applicable = true;
    if(apply) G.newNode<bool>({a.p}, {o1,o2}, NULL, false);
  }
  if(a.a==break_){
    Node *toBeBroken=NULL;
    for(Node *p:o1->parentOf){
      if(p->keys(0)==a.p){
        if((p->parents(0)==o1 && p->parents(1)==o2) ||
           (p->parents(0)==o2 && p->parents(1)==o1)){ toBeBroken=p;  break; }
      }
    }
    if(toBeBroken){
      bool good=true;
      if(!o1->parentOf.N || !o2->parentOf.N) good=false; //if it has no relation!
      for(Node *i:o1->parentOf){ if(i->keys(0)=="pose" || i->keys(0)=="controllable") continue;  if(i->keys(0)=="fixed" || i->keys(0)=="canGrasp") break;  if(i->parents.N<2) good=false; }
      for(Node *i:o2->parentOf){ if(i->keys(0)=="pose" || i->keys(0)=="controllable") continue;  if(i->keys(0)=="fixed" || i->keys(0)=="canGrasp") break;  if(i->parents.N<2) good=false; }
      if(!good) toBeBroken=NULL;
    }
    if(toBeBroken){
      applicable = true;
      CHECK(&toBeBroken->container==&G,"");
      if(apply) delete toBeBroken;
    }
  }
  return applicable;
}

void State::write(ostream& os) const{
  os <<"PreAction: " <<preAction <<endl;
  os <<"Graph:" <<G <<endl;
  os <<endl;
}

SearchNode::SearchNode(SearchNodeL& container_)
  :container(&container_), preNode(NULL){
  container->append(this);
  domain.getInitialState(state);
  state.compControllable();
}

SearchNode::SearchNode(const SearchNode& preNode_, const Action& preAction)
  :container(preNode_.container), preNode(&preNode_){
  CHECK(preNode, "");
  container->append(this);

  state = preNode->state;
  state.preAction = preAction;
  if(!state.testAction(preAction, true)){
    cout <<"*** can't execute that action" <<endl;
  }else{
    state.compControllable();
  }

}

Action SearchNode::getRandomFeasibleAction(){
  Action a;
  for(;;){
    a.setRandom(domain.numObjects());
    if(state.testAction(a, false)) break;
  }
  return a;
}






