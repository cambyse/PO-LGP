#include "fol.h"

/** The RelationalMachine maintains a knowledge base, which stores both, a state (=set/conjunction of facts) and a set
 * of rules (classically clauses, here 1st-order rules which may represent a stationary policy, script, or policy tree).
 *
 * Two situations change the state:
 * 1) An external client (sensor, action feedback, or planner) explicitly adds a fact via 'applyEffect'
 * 2) Within 'fwdChainRules', all rules are checked for their condition and if they hold, their effects are applied to the state
 *
 * The standard usage is nothing but alternating 'applyEffect' (triggered from external clients) and 'fwdChainRules'/
 *
 */
struct RelationalMachine{
  Graph KB;     ///< knowledge base
  Graph *state; ///< the state within the KB (is a subgraph item of KB)
  Graph *tmp;   ///< a tmp subgraph of the KB (private)
  bool verbose;

  RelationalMachine(const char* filename);

  bool queryCondition(MT::String query); ///< return indicates coverage of the condition
  bool applyEffect(MT::String effect);   ///< return indicates change of state
  ItemL fwdChainRules();                 ///< progresses the state by applying all rules until convergence

  void declareNewSymbol(MT::String symbol);
  MT::String getState();
  MT::String getRules();
  StringA getSymbols();
};

inline RelationalMachine& operator<<(RelationalMachine& RM, const char* effect){
  RM.applyEffect(MT::String(effect));
  return RM;
}

inline std::ostream& operator<<(std::ostream& os, RelationalMachine& RM){
  os <<RM.getState();
  return os;
}
