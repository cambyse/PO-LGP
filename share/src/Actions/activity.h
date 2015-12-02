#pragma once

#include <Core/graph.h>
#include <Core/module.h>

//===========================================================================
/**
 * Activities are "things" that change the state of the KB.
 *
 * Activity (an ABC) defines the interface for each concrete activity.
 *
 * There are
 * - ControlActivities for motion control and
 * - SensorActivities that trigger on sensor events
 */
struct Activity {
  MT::String name;     ///< name, just for reporting
  Node *fact;          ///< pointer to the fact in the state of a KB
  double activityTime; ///< for how long it this activity running yet

  Activity():fact(NULL), activityTime(0.){}
  virtual ~Activity(){}

  void write(ostream& os) const { os <<"Activity '" <<name <<"' (t=" <<activityTime <<") "; if(fact) os <<*fact; else os <<"()"; }

  /// @name activity interface to overwrite
  /// Read the facts and configure the activity
  virtual void configure(Node *fact);

  /// Do whatever the Activity has to do and change the state of the KB
  virtual void step(double dt) = 0;
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

/// Return the spec from the fact
Graph* getSpecsFromFact(Node *fact);
