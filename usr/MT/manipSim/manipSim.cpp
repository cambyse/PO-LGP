#include "manipSim.h"

Domain domain;

enum Predicate : uint{ rigid=0, trans2DPhi, _N_Predicate };
const char* predicateString(uint p){
  static const char* bla[]={"rigid", "trans2DPhi", "no"};
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
//  ItemL objs = G.getItems("body");

//  cout <<G <<endl;
  ItemL ctrltags = G.getItems("controllable");
  for_list_rev(Item, i, ctrltags) delete i;

  ctrltags = G.getItems("canGrasp");
  ItemL ctrlables;

  for(Item *o:ctrltags) ctrlables.append(o->parents(0));

  for(Item *o:ctrlables){
    for(Item *r:o->parentOf){
      if(r->keys(0)=="rigid"){
        if(r->parents(0)==o)
          ctrlables.setAppend(r->parents(1));
        else
          ctrlables.setAppend(r->parents(0));
      }
    }
  }

  for(Item *o:ctrlables){
    bool hasTag=false;
    for(Item *r:o->parentOf) if(r->keys(0)=="controllable"){ hasTag=true; break; }
    if(!hasTag) G.append<bool>(STRINGS("controllable"), ARRAY(o), NULL);
  }
//  cout <<G <<endl;
}

void State::expandReachable(){
}

bool State::testAction(const Action& a, bool apply){
  bool applicable=false;
  Item *o1 = G(a.i);
  Item *o2 = G(a.j);
  if(a.a==create_){
    if(!o1->ParentOf()["controllable"]) return false;
    if(a.p=="rigid"){
      if(!o1->ParentOf()["canGrasp"]) return false;
    }
    applicable = true;
    if(apply) G.append<bool>(STRINGS(a.p), ARRAY(o1,o2), NULL);
  }
  if(a.a==break_){
    Item *toBeBroken=NULL;
    for(Item *p:o1->parentOf){
      if(p->keys(0)==a.p){
        if((p->parents(0)==o1 && p->parents(1)==o2) ||
           (p->parents(0)==o2 && p->parents(1)==o1)){ toBeBroken=p;  break; }
      }
    }
    if(toBeBroken){
      bool good=true;
      if(!o1->parentOf.N || !o2->parentOf.N) good=false; //if it has no relation!
      for(Item *i:o1->parentOf){ if(i->keys(0)=="pose" || i->keys(0)=="controllable") continue;  if(i->keys(0)=="fixed" || i->keys(0)=="canGrasp") break;  if(i->parents.N<2) good=false; }
      for(Item *i:o2->parentOf){ if(i->keys(0)=="pose" || i->keys(0)=="controllable") continue;  if(i->keys(0)=="fixed" || i->keys(0)=="canGrasp") break;  if(i->parents.N<2) good=false; }
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






