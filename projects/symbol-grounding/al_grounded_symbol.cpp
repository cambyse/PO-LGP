#include "al_grounded_symbol.h"

#include "activeLearningProcess.h"
#include "oracle.h"

struct sAL_GroundedSymbol {
  sAL_GroundedSymbol(ActiveLearner& al) : al(al) {};
  ActiveLearner& al;  
};

AL_GroundedSymbol::AL_GroundedSymbol(ActiveLearner& al, MT::String& name, uint arity, bool build_derived_predicates) :
  GroundedSymbol(name, arity, build_derived_predicates),
  s(new sAL_GroundedSymbol(al))
{ }

bool AL_GroundedSymbol::holds(arr& x) const {
  MT::Array<arr> f;
  f.append(x);
  return s->al.classify(f);
}

void AL_GroundedSymbol::train(const uint samples, Oracle* oracle) {
  ClassificatorV cl;
  cl.classificator = &s->al;
  if (NULL == oracle) oracle = new HumanOracle(GroundedSymbol::name);
  cl.oracle = oracle;

  ActiveLearningP alp;
  alp.classificator = &cl;
  alp.traindata = new TrainingsDataV;
}

