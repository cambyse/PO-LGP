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
  virtual void write(ostream& os) const { os <<"Activity (t=" <<actionTime <<") "; if(fact) os <<*fact; else os <<"()"; }
};
stdOutPipe(Activity)

typedef MT::Array<Activity*> ActivityL;

Graph& activityRegistry();
template<class T> void registerActivity(const char* key){
  new Item_typed<Type>(activityRegistry(), {key}, {}, new Type_typed<T,void>, true);
}

Activity* newActivity(Item *fact);
