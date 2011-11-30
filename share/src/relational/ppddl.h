#include "plan.h"

namespace TL {

// PPDDL: reads objects, start-state, goal
void readPPDDLdomain(TL::State& start_state, TL::Reward& goal, const char * filename);
// void readPPDDLdomain(TL::State& start_state, TL::PredicateListGoal& goal, LogicEngine& le, const char * filename);
// void readPPDDLdomain(TL::State& start_state, TL::MaximizeFunctionGoal& goal, LogicEngine& le, const char * filename);


// NID --> PPDDL
void writePPDDL_description(const TL::RuleSet& rules, bool all_outcome, const TL::State& state, const TL::LiteralListReward& goal, const char* filename);
void writeRulesAsPPDDL(const TL::RuleSet& rules, bool all_outcome, ostream& out);
// void writeRulesAsPPDDL2(const TL::RuleSet& rules, const TL::LogicEngine& le, ostream& out);


}
