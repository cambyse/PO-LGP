#include "activity.h"

Singleton<Graph> ActivityRegistry;

Graph& activityRegistry(){ return ActivityRegistry(); }


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

  Node *actType = activityRegistry().getNode(symbol->keys.last());
  if(!actType){
    LOG(-1) <<"cannot create activity " <<*fact << "(symbol=" <<*symbol <<", specs=" <<*specs <<")";
    return NULL;
  }
  CHECK(actType->getValueType()==typeid(Type),"");
  Activity *act = (Activity*)(actType->getValue<Type>()->newInstance());
  act->configure(specs);
  act->fact = fact;
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
