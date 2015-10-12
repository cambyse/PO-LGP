#pragma once

#include <Core/graph.h>
#include <Core/module.h>

//===========================================================================
/**
 * Activities are "things" that change the state of the KB.
 *
 * Activity (an ABC) defines the interface for each concrete activity.
 */
struct Activity {
  StringA symbols;     ///< the symbols that an abstract fact representing this activity should have
  Graph params;        ///< parameters of this activity
  Node *fact;          ///< pointer to the fact in the state of a KB
  double activityTime; ///< for how long it this activity running yet

  Activity():fact(NULL), activityTime(0.){}
  virtual ~Activity(){}
  void associateToExistingFact(Node *fact);
  void createFactRepresentative(Graph& state);

  virtual void configure() = 0; ///< configure yourself from an abstract fact (and its graph parameters)
  virtual void step(double dt) = 0;

  void write(ostream& os) const { os <<"Activity (" <<symbols <<"){" <<params <<"} (t=" <<activityTime <<") "; if(fact) os <<*fact; else os <<"()"; }
};
stdOutPipe(Activity)

//===========================================================================

typedef mlr::Array<Activity*> ActivityL;

/// register an activity class/type
template<class T> void registerActivity(const char* key){
  new Node_typed<Type>(registry(), {"Activity", key}, {}, new Type_typed<T,void>, true);
}

/// create/launch a new activity based on the fact
Activity* newActivity(Node *fact);

/// create/launch a new activity based on the type, symbols and params; adds a fact to relationalState
template<class T>
void newActivity(Graph& relationalState, const StringA& symbols, const Graph& params){
  Activity *act = dynamic_cast<Activity*>(new T);
  act->symbols = symbols;
  act->params = params;

  //-- add refs to specs for other symbols
  for(uint i=1;i<symbols.N;i++){
    new Node_typed<mlr::String>(act->params, {STRING("ref"<<i)}, {}, new mlr::String(symbols(i)), true);
  }

  act->createFactRepresentative(relationalState);
  registry().getValue<Variable<ActivityL> >("A") -> set()->append(act);
}

//===========================================================================

struct ActivitySpinnerModule : Module{
  ACCESSnew(ActivityL, A)

  ActivitySpinnerModule() : Module("ActivitySpinnerModule", .01) {}

  void open(){}
  void step(){
    A.readAccess();
    for(Activity *act:A()) act->step(0.01);
//    for(Activity *act:A()) act->write(cout);
    A.deAccess();
  }
  void close(){}
};

/// Return the spec from the fact
Graph* getSpecsFromFact(Node *fact);
