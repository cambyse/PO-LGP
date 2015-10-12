#include "activity.h"

//===========================================================================

void Activity::associateToExistingFact(Node* fact){
  this->fact = fact;
  for(Node *p:fact->parents) symbols.append(p->keys.last()); //adopt the symbols
  if(fact->getValueType()==typeid(Graph)) params.copy(fact->graph(), NULL); //copy the parameters (but DON'T become also a subgraph of state!)
  configure();
}

void Activity::createFactRepresentative(Graph& state){
  CHECK(symbols.N>0,"need symbols to create a Fact that represents this activity");
  if(!params.N) MLR_MSG("Are you sure to create a fact without params?");
  this->fact = new Node_typed<Graph>(state, {}, state.getNodes(symbols), &params, false);
  configure();
}

//===========================================================================


//===========================================================================
// Activity
void Activity::configure(Node *fact) {
  name.clear();
  for(Node *p : fact->parents){
    name << p->keys.last();
  }
  Activity::fact = fact;
}

//===========================================================================
Activity* newActivity(Node *fact){
  Node *symbol=fact->parents(0);
  while(symbol->parents.N) symbol=symbol->parents(0);

  Node *specs=fact;
  while(specs->getValueType()!=typeid(Graph) && specs->parents.N) specs=specs->parents(0);

  //-- add refs to specs for other symbols
  if(specs->getValueType()==typeid(Graph)) for(uint i=1;i<fact->parents.N;i++){
    new Node_typed<mlr::String>(specs->graph(), {STRING("ref"<<i)}, {}, new mlr::String(fact->parents(i)->keys.last()), true);
  }

  LOG(3) <<"creating new activity of symbol '" <<*symbol <<"' and specs '" <<*specs <<"'";
  Node *actType = registry().getNode("Activity", symbol->keys.last());
  if(!actType){
    LOG(3) <<"cannot create activity " <<*fact << "(symbol=" <<*symbol <<", specs=" <<*specs <<")";
    return NULL;
  }
  CHECK(actType->getValueType()==typeid(Type),"");
  Activity *act = (Activity*)(actType->getValue<Type>()->newInstance());
  act->associateToExistingFact(fact);
  return act;
}

//===========================================================================
Graph* getSpecsFromFact(Node *fact) {
  Graph* specs = &NoGraph;
  if(fact->getValueType()==typeid(Graph)) {
    specs = &fact->graph();
  }
  return specs;
}

//===========================================================================
RUN_ON_INIT_BEGIN(Activity)
ActivityL::memMove=true;
RUN_ON_INIT_END(Activity)
