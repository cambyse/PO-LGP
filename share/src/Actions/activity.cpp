#include "activity.h"

//===========================================================================

void Activity::associateToExistingFact(Node* fact){
  this->fact = fact;
  for(Node *p:fact->parents) symbols.append(p->keys.last()); //adopt the symbols
  if(fact->isGraph()) params.copy(fact->graph()); //copy the parameters (but DON'T become also a subgraph of state!)
  configure();
}

//===========================================================================

/* This is never used. Perhaps require that a fact always exists first.
void Activity::createFactRepresentative(Graph& state){
  CHECK(symbols.N>0,"need symbols to create a Fact that represents this activity");
  if(!params.N) MLR_MSG("Are you sure to create a fact without params?");
  this->fact = new Node_typed<Graph*>(state, {}, state.getNodes(symbols), &params, false);
  configure();
}
*/

//===========================================================================

Activity* newActivity(Node *fact){
  Node *activitySymbol=fact->parents(0);
  while(activitySymbol->parents.N) activitySymbol=activitySymbol->parents(0);

  Node *activityParams=fact;
  while(!activityParams->isGraph() && activityParams->parents.N) activityParams=activityParams->parents(0);

  //-- all other symbols in the literal are added to the params
  if(activityParams->isGraph()) for(uint i=1;i<fact->parents.N;i++){
    CHECK(!activityParams->graph()[STRING("ref"<<i-1)], "can't specify ref"<<i-1<<" both, as symbols and as parameter");
    new Node_typed<mlr::String>(activityParams->graph(), {STRING("ref"<<i-1)}, {}, mlr::String(fact->parents(i)->keys.last()));
  }

  LOG(3) <<"creating new activity of symbol '" <<*activitySymbol <<"' and specs '" <<*activityParams <<"'";
  Node *activityType = registry().getNode({"Activity", activitySymbol->keys.last()});
  if(!activityType){
    LOG(3) <<"cannot create activity " <<*fact << "(symbol=" <<*activitySymbol <<", specs=" <<*activityParams <<")";
    return NULL;
  }
  CHECK(activityType->getValueType()==typeid(Type*),"");

  Activity *act = (Activity*)(activityType->V<Type*>()->newInstance());
  act->associateToExistingFact(fact);
  return act;
}

//===========================================================================

RUN_ON_INIT_BEGIN(Activity)
ActivityL::memMove=true;
RUN_ON_INIT_END(Activity)
