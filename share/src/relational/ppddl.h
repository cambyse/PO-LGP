#if 0

#include "plan.h"

namespace PRADA {

// PPDDL: reads objects, start-state, goal
void readPPDDLdomain(SymbolicState& start_state, Reward& goal, const char * filename);
// void readPPDDLdomain(SymbolicState& start_state, PredicateListGoal& goal, LogicEngine& le, const char * filename);
// void readPPDDLdomain(SymbolicState& start_state, MaximizeFunctionGoal& goal, LogicEngine& le, const char * filename);


// NID --> PPDDL
void writePPDDL_description(const RuleSet& rules, bool all_outcome, const SymbolicState& state, const LiteralListReward& goal, const char* filename);
void writeRulesAsPPDDL(const RuleSet& rules, bool all_outcome, ostream& out);
// void writeRulesAsPPDDL2(const RuleSet& rules, const LogicEngine& le, ostream& out);


}

#endif