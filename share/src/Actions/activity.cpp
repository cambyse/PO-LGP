#include "activity.h"

//===========================================================================

Activity* newActivity(Node *fact){
  Node *activitySymbol=fact->parents(0);
  while(activitySymbol->parents.N) activitySymbol=activitySymbol->parents(0);

  Node *activityParams=fact;
  while(!activityParams->isGraph() && activityParams->parents.N) activityParams=activityParams->parents(0);

  //-- all other symbols in the literal are added to the params
  if(activityParams->isGraph()) for(uint i=0;i<fact->parents.N;i++){
    CHECK(!activityParams->graph()[STRING("sym"<<i)], "can't specify sym"<<i<<" both, as symbols and as parameter");
    activityParams->graph().newNode<mlr::String>({STRING("sym"<<i)}, {}, mlr::String(fact->parents(i)->keys.last()));
  }

  //-- check if an activity class with this symbol name is registered
  LOG(3) <<"creating new activity of symbol '" <<*activitySymbol <<"' and specs '" <<*activityParams <<"'";
  Node *activityType = registry()->getNode({"Activity", activitySymbol->keys.last()});
  if(!activityType){
    LOG(3) <<"cannot create activity " <<*fact << "(symbol=" <<*activitySymbol <<", specs=" <<*activityParams <<")";
    return NULL;
  }
  CHECK(activityType->isOfType<Type*>(),"");

  //-- yes -> create new instance and configure it
  Activity *act = (Activity*)(activityType->get<Type*>()->newInstance());
  act->fact = fact;
  for(Node *p:fact->parents) act->symbols.append(p->keys.last()); //adopt the symbols
  if(fact->isGraph()) act->params.copy(fact->graph(), false, true); //copy the parameters (but DON'T become also a subgraph of state!)
  act->configure();

  return act;
}

//===========================================================================

RUN_ON_INIT_BEGIN(Activity)
ActivityL::memMove=true;
RUN_ON_INIT_END(Activity)
