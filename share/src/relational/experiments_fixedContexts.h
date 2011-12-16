#ifndef TL__RULE_LEARNER_FIXED_CONTEXTS
#define TL__RULE_LEARNER_FIXED_CONTEXTS

#include "relational/logicDefinitions.h"
#include "relational/ruleLearner.h"
#include "relational/ruleExplorer.h"



namespace TL {


class RuleLearner_FixedContexts : public RuleLearner {

public:
  TL::Atom* action;
  MT::Array< LitL > contexts;
  MT::Array< ExperienceL > experiences_per_context;
  MT::Array< uintA > experienceIds_per_context;  // redundant auxiliary container
  
  public:
    RuleLearner_FixedContexts(TL::Atom* action, const MT::Array< LitL >& contexts, double alpha_PEN, double p_min, double p_min_noisyDefaultRule);
     
    void learn_rules(TL::RuleSetContainer& rulesC, ExperienceL& experiences, const char* logfile = DEFAULT_LOGFILE); 
    void learn_rules(TL::RuleSetContainer& rulesC, ExperienceL& experiences, arr& experience_weights, const char* logfile = DEFAULT_LOGFILE); 
};


class AbstractRuleExplorer_FixedContexts : public AbstractRuleExplorer {
  RuleSet fixed_partial_rules;
public:
  AbstractRuleExplorer_FixedContexts(const RuleSet& fixed_partial_rules, double complexity_penalty_coeff, double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                                      TL::RuleSet& fixed_rules_for_fixed_actions, uint density_estimation_type);
  
  bool actionIsKnown(const TL::State& state, TL::Atom* action);
  TL::Atom* decideAction(const TL::State& state, TL::NID_Planner* planner, uint behavior_type, bool use_known_state_partial);
};

}

#endif // TL__RULE_LEARNER_FIXED_CONTEXTS
