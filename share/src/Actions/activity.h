#pragma once

#include <Core/graph.h>
#include <Core/module.h>

struct Activity {
  MT::String name;     ///< name, just for reporting
  Node *fact;          ///< pointer to the fact in the state of a KB
  double activityTime; ///< for how long it this activity running yet

  Activity():fact(NULL), activityTime(0.){}
  virtual ~Activity(){}
  virtual void configure(Node *fact) = 0;
  virtual void step(double dt) = 0;
  void write(ostream& os) const { os <<"Activity '" <<name <<"' (t=" <<activityTime <<") "; if(fact) os <<*fact; else os <<"()"; }
};
stdOutPipe(Activity)

//===========================================================================

typedef MT::Array<Activity*> ActivityL;

/// global registry of activity classes/types (implementations)
Graph& activityRegistry();

/// register an activity class/type
template<class T> void registerActivity(const char* key){
  new Node_typed<Type>(activityRegistry(), {key}, {}, new Type_typed<T,void>, true);
}

/// create/launch a new activity based on the fact
Activity* newActivity(Node *fact);

//===========================================================================

struct ActivitySpinnerModule : Module{
  ACCESS(ActivityL, A)

  /// @name module implementations
  void open(){}
  void step(){
    A.readAccess();
    for(Activity *act:A()) act->step(0.01);
    A.deAccess();
  }
  void close(){}
};
