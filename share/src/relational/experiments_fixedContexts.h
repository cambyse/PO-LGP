#if 0

#ifndef TL__RULE_LEARNER_FIXED_CONTEXTS
#define TL__RULE_LEARNER_FIXED_CONTEXTS

#include "relational/symbols.h"
#include "relational/learn.h"
#include "relational/explore.h"



namespace PRADA {


class RuleLearner_FixedContexts : public RuleLearner {

public:
  Atom* action;
  MT::Array< LitL > contexts;
  MT::Array< SymbolicExperienceL > experiences_per_context;
  MT::Array< uintA > experienceIds_per_context;  // redundant auxiliary container
  
  public:
    RuleLearner_FixedContexts(Atom* action, const MT::Array< LitL >& contexts, double alpha_PEN, double p_min, double p_min_noisyDefaultRule);
     
    void learn_rules(RuleSetContainer& rulesC, SymbolicExperienceL& experiences, const char* logfile = DEFAULT_LOGFILE); 
    void learn_rules(RuleSetContainer& rulesC, SymbolicExperienceL& experiences, arr& experience_weights, const char* logfile = DEFAULT_LOGFILE); 
};


class AbstractRuleExplorer_FixedContexts : public AbstractRuleExplorer {
  RuleSet fixed_partial_rules;
public:
  AbstractRuleExplorer_FixedContexts(const RuleSet& fixed_partial_rules, double complexity_penalty_coeff, double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                                      RuleSet& fixed_rules_for_fixed_actions, uint density_estimation_type);
  
  bool actionIsKnown(const SymbolicState& state, Atom* action);
  Atom* decideAction(const SymbolicState& state, NID_Planner* planner, uint behavior_type, bool use_known_state_partial);
};

}

#endif // TL__RULE_LEARNER_FIXED_CONTEXTS

#endif
