#ifndef _AL_GROUNDED_SYMBOL_H_
#define _AL_GROUNDED_SYMBOL_H_

#include "al.h"

#include <relational/symbolGrounding.h>

class sAL_GroundedSymbol;
class ActiveLearner;

class AL_GroundedSymbol : public relational::GroundedSymbol {
  public:
    AL_GroundedSymbol(ActiveLearner* al, MT::String& name, uint arity, bool build_derived_predicates = false);
    AL_GroundedSymbol(MT::String& name, uint arity, bool build_derived_predicates = false);
    virtual bool holds(arr& x) const;

		ActiveLearner* classificator;
};

#endif
