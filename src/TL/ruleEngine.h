/*  
    Copyright 2009   Tobias Lang
    
    Homepage:  cs.tu-berlin.de/~lang/
    E-mail:    lang@cs.tu-berlin.de
    
    This file is part of libPRADA.

    libPRADA is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    libPRADA is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with libPRADA.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef TL__RULE_ENGINE
#define TL__RULE_ENGINE

#include "TL/logicEngine.h"


#define STATE_TRANSITION__NOISE_OUTCOME 11


namespace TL {

namespace RuleEngine {
  
  // Set this first!!
  void setLogicEngine(TL::LogicEngine* le);
  
  
  
  /****************************************
    BASIC HELPERS
   ***************************************/
    
  void calcDeicticRefs(const TL::Rule& rule, uintA& drefs); // no LE
  void calcDeicticRefs(const TL::Rule& rule, uintA& drefs, boolA& inNegatedLiteralsOnly); // no LE
  void calcTerms(const TL::Rule& rule, uintA& terms); // no LE
  void calcAbsentLiterals(const TL::Rule& rule, PredIA& literals, bool positiveOnly = false);
  bool usesPI(const TL::Rule& rule, TL::PredicateInstance* pi);
  // calcs which cover_context contain deictic refs
  void DRcontext(const TL::Rule& rule, boolA& containsDR); // no LE
  void changingConcepts(uintA& changingIds_preds, uintA& changingIds_funcs, const TL::RuleSet& rules);
  bool stupidContext(const TL::Rule& rule);
  uint numPredicateInstances(const TL::Rule& rule); // no LE
  bool isGrounded(TL::Rule* predT);
  // we allow for ungrounded free vars in negative context literals, e.g. in -on(39,X)
  bool isGrounded_positives(TL::Rule* predT);
  void removeDoublePredicateInstances(const TL::RuleSet& ground_rules);
  void removeNonChangingConcepts(TL::RuleSet& ground_rules, const TL::RuleSet& abstract_rules);
  void checkRules(const TL::RuleSet& rules);
  void cleanup(TL::Rule& rule);  // order context and outcomes, clean-up DRs
  
  
  
  /****************************************
    RULE MANIPULATION
   ***************************************/
  void insert(TL::Rule& rule, TL::PredicateInstance& literal);
  // rule application in state s
  TL::Rule* ground(TL::Rule* r, TL::Substitution* sub);
  void ground(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants);
  void ground_with_filtering(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants, const TL::State& s, bool delete_nonchanging_concepts = false);
//   void ground_with_filtering(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants, const uintA& blocks, const uintA& balls, uint tableID, const arr& sizes);
  void makeOriginal(TL::Rule& w);
  
  
  
  /****************************************
    TRANSITION KNOWLEDGE
   ***************************************/
  // Assumption: outcome contains primitives only
  void calcSuccessorState(const TL::State& precessor, const PredIA& outcome, TL::State& successor, bool deriveDerived);
  double calcSuccessorState(const TL::State& precessor, TL::Rule* rule, uint& flag, TL::State& successor, bool deriveDerived);
  // returns true iff unique covering rule; otherwise returns false (--> undefined successor state)
  double calcSuccessorState(const TL::State& precessor, const TL::RuleSet& ground_rules, TL::PredicateInstance* action, uint& flag, TL::State& successor, bool deriveDerived);
	
	// assumes that rule is indeed applicable and rule is grounded
  double probability_groundRule(TL::Rule* groundRule, const TL::State& pre, const TL::State& post, double noiseStateProbability = 0.0);
	// grounds rule first (checks whether context and action hold)
  double probability_abstractRule(TL::Rule* abstractRule, const TL::State& pre, TL::PredicateInstance* groundedAction, const TL::State& post, double noiseStateProbability = 0.0, TL::Substitution* sub = NULL);

  double probability_defaultRule(TL::Rule* defaultRule, const TL::State& pre, const TL::State& post, double noiseStateProbability = 0.0);
  
  
  
  /****************************************
    COVERAGE
   ***************************************/
  
  // Calculates the substitutions "subs" of context variables
  // such that the context of "rule" are satisified in state "s".
  // Ensures that deictic references are different from action arguments!
  bool cover_context(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs, TL::Substitution* actionSub);
	
  // RULE COVERAGE
  // action prescribed to be "groundedAction"
  // (Calculates the substitutions "subs" which ground the _action_ and the context
	// of "rule" such that they are satisified in "action" and state "s".)
  bool cover_rule_groundedAction(const TL::State& s, TL::PredicateInstance* groundedAction, const TL::Rule* rule, TL::SubstitutionSet& subs);
  // General version.
  // Returns all possible instantiations for this rule.
  // (Remember: action arguments are allowed
  // to have different substitutions, while deictic reference can only have one sub once the
  // subs for the action arguments are given.)
  bool cover_rule(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs);

  bool cover_groundRule_groundedAction(const TL::State& s, TL::PredicateInstance* groundedAction, const TL::Rule* ground_rule);
    
	// WRAPPERS FOR RULE-SETS
	// action prescribed to be "groundedAction"
  void coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction, TL::RuleSet& coveringGroundedRules);
  void coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction, uintA& coveringRuleIDs);
  void coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction, TL::RuleSet& coveringGroundedRules);
  void coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction, uintA& coveringRuleIDs);
  TL::Rule* uniqueCoveringRule_groundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction);
  uint uniqueAbstractCoveringRule_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction);
  // General version for all actions
  void coveringRules(const TL::RuleSet& allRules, const TL::State& s, TL::RuleSet& coveringGroundedRules);
  void coveringGroundedRules(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::RuleSet& coveringGroundedRules);
  
  // For several actions
  void coveringRules(uintA& coveringRulesIDs, const TL::RuleSet& abstract_rules, const PredIA& ground_actions, const TL::State& s);
  void coveringGroundedRules(uintA& coveringRulesIDs, const TL::RuleSet& ground_rules, const PredIA& ground_actions, const TL::State& s);
  
  // explaining state transitions
  void coveringOutcomes(TL::Rule* groundedRule, const TL::State& pre, const TL::State& post, uintA& covering_outcomes);
  void coveringOutcomes(TL::Rule* abstractRule, const TL::State& pre, TL::PredicateInstance* groundedAction, const TL::State& post, uintA& covering_outcomes);
  
  void calcGroundDeicticReferences(uintA& ground_drefs, const TL::State& state, TL::PredicateInstance* groundedAction, TL::Rule* rule);
  
  
  
  /****************************************
    SPECIAL RULES
   ***************************************/
	// default rule (which explains everything as noise)
  TL::Rule* generateDefaultRule(double noiseProb = 0.5, double minProb = 0., double change = 2.0);
  bool isDefaultRule(TL::Rule* rule);
  // no-action rule
  TL::Rule* getDoNothingRule();

  
  /****************************************
   READ & WRITE
  ***************************************/
  
  void writeNice(const TL::RuleSet&, ostream& os = cout, bool breaks = false); // no LE
  void readRules(const char* filename, RuleSet& rules);
  void readRulesPlain(const char* filename, RuleSet& rules);
  void writeRulesPlain(const char* filename, const RuleSet& rules, bool skip_noise_outcome = true);
}

}



#endif // TL__RULE_ENGINE
