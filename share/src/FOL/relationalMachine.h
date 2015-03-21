#include "fol.h"

struct RelationalMachine{
  Graph KB; ///< knowledge base
  Graph *state; ///< the state within the KB (is a subgraph item of KB)
  Graph *tmp;   ///< a tmp subgraph of the KB (private)

  RelationalMachine(const char* filename);

  bool queryCondition(MT::String query); ///< return indicates coverage of the condition
  bool applyEffect(MT::String effect); ///< return indicates change of state
  bool fwdChainRules(); ///< progresses the state by applying all rules until convergence
  void declareNewSymbol(MT::String symbol);
  MT::String getState();
  MT::String getRules();
  StringA getSymbols();
};
