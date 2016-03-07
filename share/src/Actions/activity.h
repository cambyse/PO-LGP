#pragma once

#include <Core/graph.h>
#include <Core/module.h>

//===========================================================================

/** Activites are the glue between facts in the relational state (RelationalMachineModule),
 *  and real things, like creating a control task (in the TaskControllerModule),
 *  listening to a sensor, starting a thread, or maybe even launching a node.
 */
struct Activity {
  Node *fact;          ///< pointer to the fact in the state of a KB
  double activityTime; ///< for how long is this activity running yet

  StringA symbols;     ///< for convenience: copies of the fact->parent keys
  Graph params;        ///< for convenience: a copy of the fact parameters PLUS refX keys for all symbols

  Activity():fact(NULL), activityTime(0.){}
  virtual ~Activity(){}

  //-- 'callbacks' that are called from the RelationalMachine if something changes
  void associateToExistingFact(Node *fact);
  //void createFactRepresentative(Graph& state); this is never used

  /// configure yourself from the 'symbols' and 'params'
  virtual void configure(){}

  /// the activity spinner runs with 100Hz and calls this for all activities -- use only for
  /// non-computational heavy quick updates. Computationally heavy things should be threaded!
  virtual void activitySpinnerStep(double dt){ activityTime += dt; }

  //-- 'responses' of activities
  void setEffect(const char* effect);
  void terminate();

  void write(ostream& os) const { os <<"Activity (" <<symbols <<"){" <<params <<"} (t=" <<activityTime <<") "; if(fact) os <<*fact; else os <<"()"; }
};
stdOutPipe(Activity)

//===========================================================================

typedef mlr::Array<Activity*> ActivityL;

/// register an activity class/type
template<class T> void registerActivity(const char* key){
  new Node_typed<Type*>(registry(), {"Activity", key}, {}, new Type_typed<T,void>);
}

/// create/launch a new activity based on the fact
Activity* newActivity(Node *fact);

/// create/launch a new activity based on the type, symbols and params; adds a fact to relationalState
/* This is never used. Also: implement this by first creating the fact, then calling newActivity(fact) and checking the type
template<class T>
void newActivity(Graph& relationalState, const StringA& symbols, const Graph& params){
  Activity *act = dynamic_cast<Activity*>(new T);
  act->symbols = symbols;
  act->params = params;

  //-- add refs to specs for other symbols
  for(uint i=1;i<symbols.N;i++){
    CHECK(!act->params[STRING("ref"<<i-1)], "can't specify ref"<<i-1<<" both, as symbols and as parameter");
    new Node_typed<mlr::String>(act->params, {STRING("ref"<<i-1)}, {}, new mlr::String(symbols(i)), true);
  }

  act->createFactRepresentative(relationalState);
  registry().find<Variable<ActivityL> >("A") -> set()->append(act);
}
*/

//===========================================================================

struct ActivitySpinnerModule : Module{
  ACCESSnew(ActivityL, A)

  ActivitySpinnerModule() : Module("ActivitySpinnerModule", .01) {}

  void open(){}
  void step(){
    A.readAccess();
    for(Activity *act:A()) act->activitySpinnerStep(0.01);
//    for(Activity *act:A()) act->write(cout);
    A.deAccess();
  }
  void close(){}
};

