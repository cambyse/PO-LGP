#include "manipSim.h"

Domain domain;

const char* predicateString(Predicate p){
  static const char* bla[]={"isFree", "rigid", "trans2DPhi", "no"};
  return bla[p];
}
const char* actionPredicateString(ActionPredicate a){
  static const char* bla[]={"create_", "break_", "no_"};
  return bla[a];
}

Action& NoAction = *((Action*)NULL);
SearchNode& NoSearchNode = *((SearchNode*)NULL);


//ActionL_ SearchNode::getFeasibleActions(){
//  ActionL_ A;
//  for(uint i=0;i<domain.N;i){
//    A.append(new Action());

//  }

//}


void State::compControllable(){
  ItemL objs = G.getItems("obj");
  ItemL ctrlables;
  for(uint i=0;i<=objs.N;i++) if(domain.isDirectlyControllable(i)) ctrlables.append(objs(i));

  for(uint i=0;i<ctrlables.N;i++){
    Item *o = ctrlables(i);
    for(Item *r:o->parentOf){
      if(r->keys(0)=="rel" && r->keys(1)=="rigid"){
        if(r->parents(0)==o)
          ctrlables.setAppend(r->parents(1));
        else
          ctrlables.setAppend(r->parents(0));
      }
    }
  }

  controllable.resize(objs.N);
  controllable=false;
  for(Item *o:ctrlables) controllable(o->index)=true;
}

void State::expandReachable(){

}

void State::write(ostream& os) const{
  os <<"PreAction: " <<preAction <<endl;
  os <<"Relations: ";
  for(Relation* r:R) os <<*r <<", ";
  os <<endl <<"Poses:" <<endl;
  for(Pose* p:P) os <<*p <<endl;
  os <<"Graph:" <<G <<endl;
  os <<endl;
}



SearchNode::SearchNode(SearchNodeL& container_)
  :container(&container_), preNode(NULL){
  container->append(this);
  domain.getInitialState(state);
}

SearchNode::SearchNode(const SearchNode& preNode_, const Action& preAction)
  :container(preNode_.container), preNode(&preNode_){
  CHECK(preNode, "");
  container->append(this);
  domain.getNewState(state, preNode->state, preAction);
}

Action SearchNode::getRandomFeasibleAction(){
  Action a;
  a.i=rnd(domain.numObjects());
  a.j=rnd(domain.numObjects());
  a.a=(ActionPredicate)rnd(_N_ActionPredicate);
  a.p=(Predicate)rnd(_N_Predicate);
  return a;
}




