#pragma once

#include <Core/graph.h>
#include "relationalMachine.h"
#include <Core/registry.h>

struct Activity {
  Item *fact; ///< pointer to the fact in the state of a KB

  bool active; ///< whether this activity is active (TODO: redundant?)
  double actionTime; ///< for how long it this activity running yet

  Activity():fact(NULL), active(true), actionTime(0.){}
//  Activity(Item *fact):fact(NULL), active(true), actionTime(0.){}
  virtual ~Activity(){}
  virtual void configure(Item *fact) = 0;
  virtual void step(RelationalMachine& RM, double dt) = 0;
  virtual void write(ostream& os) { os <<"Activity (t=" <<actionTime <<") "; if(fact) os <<*fact; else os <<"()"; }
};

typedef MT::Array<Activity*> ActivityL;

Graph& activityRegistry();
template<class T> void registerActivity(const char* key){
  new Item_typed<Type>(activityRegistry(), {key}, {}, new Type_typed<T,void>, true);
}

inline Activity* newActivity(Item *fact){
  Item *actType = activityRegistry().getItem(fact->parents(0)->keys.last());
  if(!actType){
    LOG(-1) <<"cannot create activity " <<*fact <<endl;
    return NULL;
  }
  CHECK(actType->getValueType()==typeid(Type),"");
  Activity *act = (Activity*)(actType->getValue<Type>()->newInstance());
  act->configure(fact);
  return act;
}

struct ActivityContainer{
  ActivityL acts;
  Activity* addActivity(Activity *act);
  Activity* addActivity(const char* key, const Graph& params);
  Activity* addActivity(Item *fact){ return addActivity(newActivity(fact)); }

  void delActivity(Item *fact);
  void delActivity(Activity *act);
};
