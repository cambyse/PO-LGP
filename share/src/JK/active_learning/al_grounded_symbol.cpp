#include "al_grounded_symbol.h"

#include "al_process.h"
#include <JK/utils/oracle.h>
#include <biros/logging.h>

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
