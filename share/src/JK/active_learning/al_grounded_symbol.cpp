#include "al_grounded_symbol.h"

#include "al_process.h"
#include <JK/utils/oracle.h>
#include <devTools/logging.h>
#include <biros/logging.h>
#include <JK/utils/featureGenerator.h>

SET_LOG(al_gs, INFO);

AL_GroundedSymbol::AL_GroundedSymbol(MT::String& name, uint arity, bool build_derived_predicates) :
  GroundedSymbol(name, arity, build_derived_predicates) 
{
  this->type = AL;
}

AL_GroundedSymbol::AL_GroundedSymbol(ActiveLearner* al, MT::String& name, uint arity, bool build_derived_predicates) :
  GroundedSymbol(name, arity, build_derived_predicates),
  classificator(al)
{
  this->type = AL;
}

bool AL_GroundedSymbol::holds(arr& x) const {
  MT::Array<arr> f;
  f.append(x);
  DEBUG_VAR(al_gs, x);

  return classificator->classify(f);
}

Oracle_GroundedSymbol::Oracle_GroundedSymbol(MT::String& name, uint arity, bool build_derived_predicates) :
  GroundedSymbol(name, arity, build_derived_predicates) {
    this->type = AL;  
  }
Oracle_GroundedSymbol::Oracle_GroundedSymbol(ActiveLearningProblem &problem, MT::String& name, uint arity, bool build_derived_predicates) :
  GroundedSymbol(name, arity, build_derived_predicates),
  problem(problem) {
    this->type = AL;  
  }
bool Oracle_GroundedSymbol::holds(arr& x) const {
  MT::Array<arr> c;
  x.reshape(8);

  c.append(x.sub(0,2));
  c.append(x.sub(3,3));
  c.append(x.sub(4,6));
  c.append(x.sub(7,7));
  c.reshape(1,4);
  

  return problem.oracle->classify(c);
}
