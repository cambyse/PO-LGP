#if 0

#include "ruleLearner_ground.h"
#include "logicReasoning.h"
#include "ruleReasoning.h"
#include <math.h>
#include <Algo/algos.h>


namespace PRADA {

const uint SearchOperator_ground::PARAM_OPT__GRAD_DESC = 0;
const uint SearchOperator_ground::PARAM_OPT__NEWTON = 1;
const uint SearchOperator_ground::PARAM_OPT__RPROP = 2;
const uint SearchOperator_ground::PARAM_OPT__CONJGRAD = 3;
const uint SearchOperator_ground::PARAM_OPT__COND_GRAD = 4;



void calcCoverage_ground(SymbolicExperienceL& covered_experiences, uintA& covered_experiences_ids, const Rule* ground_r, const SymbolicExperienceL& examples) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcCoverage_ground [START]"<<endl;
  if (DEBUG>0) ground_r->write(cout);
  covered_experiences.clear();
  covered_experiences_ids.clear();
  uint i;
  FOR1D(examples, i) {
    if (DEBUG>0) cout<<"ex"<<i<< " " << endl;
    if (DEBUG>1) examples(i)->write(cout);
    else if (DEBUG>0) cout<<*examples(i)->action<<endl;
    if (ruleReasoning::cover_groundRule_groundedAction(examples(i)->pre, examples(i)->action, ground_r)) {
      covered_experiences.append(examples(i));
      covered_experiences_ids.append(i);
      if (DEBUG>0) cout<<" --> 1"<<endl;
    }
    else {
      if (DEBUG>0) cout<<" --> 0"<<endl;
    }
  }
  if (DEBUG>0) cout<<"calcCoverage_ground [END]"<<endl;
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    RuleSetContainer_ground
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

RuleSetContainer_ground::RuleSetContainer_ground() {
  init(NULL);
}

RuleSetContainer_ground::RuleSetContainer_ground(const SymbolicExperienceL* _p_examples) {
  init(_p_examples);
}

void RuleSetContainer_ground::init(const SymbolicExperienceL* _p_examples) {
  rules.clear();
  nonDefaultRules_per_experience.clear();  // only non-default rules!
  experiences_per_rule.clear();
  experiences_per_ruleOutcome.clear();
  
  this->p_examples = _p_examples;
  if (this->p_examples != NULL) {
    nonDefaultRules_per_experience.resize(this->p_examples->N);
  }
}

void RuleSetContainer_ground::append(Rule* rule, uintA& examples_of_this_rule, MT::Array< uintA >& examples_per_outcome_of_this_rule) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RuleSetContainer_ground::append [START]"<<endl;}
  if (DEBUG>0) {rule->write(cout);  PRINT(examples_of_this_rule);  PRINT(examples_per_outcome_of_this_rule);}
  if (ruleReasoning::isDefaultRule(rule)  &&   rules.num() > 0) {
    HALT("don't append default rule");
  }
  // (1) update rules
  rules.append(rule);
  
  if (!ruleReasoning::isDefaultRule(rule)) {
    // (2 - A) update experiences_per_rule
    experiences_per_rule.append(examples_of_this_rule);
    // (3) update nonDefaultRules_per_experience
    uint i;
    FOR1D(examples_of_this_rule, i) {
      nonDefaultRules_per_experience(examples_of_this_rule(i)).append(rules.num()-1);
    }
    // (4) update experiences_per_ruleOutcome
    experiences_per_ruleOutcome.append(examples_per_outcome_of_this_rule);
  }
  else {
    // (2 - B) update experiences_per_rule
    uintA empty;
    experiences_per_rule.append(empty);
    // (4 - B) update experiences_per_ruleOutcome
    MT::Array< uintA > empty_outcome;
    experiences_per_ruleOutcome.append(empty_outcome);
  }
  if (DEBUG>0) {PRINT(nonDefaultRules_per_experience);  PRINT(experiences_per_rule);  PRINT(experiences_per_ruleOutcome);}
  if (DEBUG>0) {cout<<"RuleSetContainer_ground::append [END]"<<endl;}
}


void RuleSetContainer_ground::remove(uint id) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RuleSetContainer_ground::remove [START]"<<endl;}
  if (DEBUG>0) {PRINT(id);  cout<<"RULES BEFORE REMOVE:"<<endl;  writeNice(cout, true);}
  
  // (1) rules
  rules.remove(id);
  // (2) nonDefaultRules_per_experience: account for new ids of rules behind rule #id
  uint i, k;
  FOR1D(experiences_per_rule(id), i) {
    nonDefaultRules_per_experience(experiences_per_rule(id)(i)).removeValue(id);
  }
  FOR1D(nonDefaultRules_per_experience, i) {
    FOR1D(nonDefaultRules_per_experience(i), k) {
      if (nonDefaultRules_per_experience(i)(k) > id)
        nonDefaultRules_per_experience(i)(k)--;
    }
  }
  // (3) experiences_per_rule  and  experiences_per_ruleOutcome
  MT::Array< uintA > experiences_per_rule__new(rules.num()); // rules have already new size
  MT::Array< MT::Array < uintA > > experiences_per_ruleOutcome__new(rules.num());
  FOR1D_(rules, i) {
    if (i<id) {
      experiences_per_rule__new(i) = experiences_per_rule(i);
      experiences_per_ruleOutcome__new(i) = experiences_per_ruleOutcome(i);
    }
    else {
      experiences_per_rule__new(i) = experiences_per_rule(i+1);
      experiences_per_ruleOutcome__new(i) = experiences_per_ruleOutcome(i+1);
    }
  }
  experiences_per_rule = experiences_per_rule__new;
  experiences_per_ruleOutcome = experiences_per_ruleOutcome__new;
  
  if (DEBUG>0) {cout<<"RULES AFTER REMOVE:"<<endl;  writeNice(cout, true);}
//   sanityCheck();
  if (DEBUG>0) {cout<<"RuleSetContainer_ground::remove [END]"<<endl;}
}

void RuleSetContainer_ground::clear() {
  rules.clear();
  uint i, k;
  FOR1D(nonDefaultRules_per_experience, i) {
    nonDefaultRules_per_experience(i).clear();
  }
  FOR1D(experiences_per_rule, i) {
    experiences_per_rule(i).clear();
  }
  FOR2D(experiences_per_ruleOutcome, i, k) {
    experiences_per_ruleOutcome(i)(k).clear();
  }
  FOR1D(experiences_per_ruleOutcome, i) {
    experiences_per_ruleOutcome(i).clear();
  }
  experiences_per_rule.clear();
  experiences_per_ruleOutcome.clear();
}

void RuleSetContainer_ground::recomputeDefaultRule() {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"recomputeDefault [START]"<<endl;
  CHECK(ruleReasoning::isDefaultRule(rules.elem(0)), "First rule should be default rule, digger");
  // prepare data-fields
  if (experiences_per_rule.N == 0) {
    uintA empty;
    experiences_per_rule.append(empty);
    MT::Array< uintA > empty_outcome(2);
    experiences_per_ruleOutcome.append(empty_outcome);
  }
  experiences_per_rule(0).clear();
  experiences_per_ruleOutcome(0).clear();
  MT::Array< uintA > empty_outcome(2);
  experiences_per_ruleOutcome(0) = empty_outcome;
  // experiences_per_rule
  uint i;
  FOR1D((*p_examples), i) {
    if (nonDefaultRules_per_experience(i).N != 1) {
      experiences_per_rule(0).append(i);
      if (((*p_examples)(i))->noChange())
        experiences_per_ruleOutcome(0)(0).append(i);
      else
        experiences_per_ruleOutcome(0)(1).append(i);
    }
  }
  if (DEBUG>1) {PRINT(experiences_per_rule(0));}
  // finalize rule
  if ((*p_examples).N == 0   ||   experiences_per_rule(0).N == 0) {
    double DEFAULT_CHANGE_PROB = 0.5;
    rules.overwrite(0, ruleReasoning::generateDefaultRule(DEFAULT_CHANGE_PROB));
  }
  else {
    uint changes = 0;
    FOR1D(experiences_per_rule(0), i) {
      if ((*p_examples)(experiences_per_rule(0)(i))->pre != (*p_examples)(experiences_per_rule(0)(i))->post)
        changes++;
    }
    double prob_change = ((double) changes) / experiences_per_rule(0).N;
    if (DEBUG>1) {PRINT(prob_change);}
    Rule* new_default_rule = ruleReasoning::generateDefaultRule(prob_change, 0.05);
    rules.overwrite(0, new_default_rule);
  }
  if (DEBUG>1) {rules.elem(0)->write(cout);}
  if (DEBUG>0) {cout<<"recomputeDefault [END]"<<endl;}
}

void RuleSetContainer_ground::getResponsibilities(arr& responsibilities, MT::Array< uintA >& covered_experiences, uintA& covered_experiences_num) const {
  responsibilities.clear();
  covered_experiences.clear();
  covered_experiences_num.clear();
  responsibilities.resize(rules.num());
  covered_experiences.resize(rules.num());
  covered_experiences_num.resize(rules.num());
  covered_experiences_num.setUni(0);
  uint i;
  FOR1D((*p_examples), i) {
    if (nonDefaultRules_per_experience(i).N == 1) {
      covered_experiences(nonDefaultRules_per_experience(i)(0)).append(i);
      covered_experiences_num(nonDefaultRules_per_experience(i)(0))++;
    }
    else {
      covered_experiences(0).append(i);
      covered_experiences_num(0)++;
    }
  }
  FOR1D(responsibilities, i) {
    responsibilities(i) = (1.0 * covered_experiences_num(i)) / p_examples->N;
  }
}


void rule_write_hack_ground(Rule* rule, MT::Array< uintA >& outcome_tripletts, ostream& os) {
  CHECK(outcome_tripletts.N = rule->outcomes.N, "wrong size");
//  os << "r" << endl;
  uint i, j;
  // Default rule does not have an action specified...
  os << "ACTION: ";
  if (rule->action != NULL)
    rule->action->write(os, true);
  else
    os << "no_action";
  os << endl;
  os << "CONTEXT: ";
  FOR1D(rule->context, i) {
    rule->context(i)->write(os);
//     os<<context(i);
    os << " ";
  }
  os << endl;
  os << "OUTCOMES:" << endl;
  FOR1D(rule->outcomes, i) {
    os << "  " << rule->probs(i) << " ";
    FOR1D(rule->outcomes(i), j) {
      rule->outcomes(i)(j)->write(os);
//       os<<outcomes(i)(j);
      os << " ";
    }
    if (i==rule->outcomes.N-1)
      os<<rule->noise_changes;
    os<<"    ";
    os<<outcome_tripletts(i);
    os << endl;
  }
}


void RuleSetContainer_ground::writeNice(ostream& out, bool only_action) const {
  uint i;
  out<<"RULES:"<<endl;
  Atom* last_action = NULL;
  FOR1D_(rules, i) {
    if (last_action != rules.elem(i)->action) {
      last_action = rules.elem(i)->action;
      out<<"##### ";  rules.elem(i)->action->write(out);  out<<endl;
    }
    out<<"# "<<i<<"  covering "<< experiences_per_rule(i) << "  " << experiences_per_ruleOutcome(i) <<endl;
    if (only_action) {
      rules.elem(i)->action->write(out); out<<endl;
    }
    else {
      rule_write_hack_ground(rules.elem(i), experiences_per_ruleOutcome(i), out);
    }
  }
  out<<"EXAMPLES:"<<endl;
  FOR1D(nonDefaultRules_per_experience, i) {
    if (nonDefaultRules_per_experience(i).N > 0)
    out<<i<<":"<<nonDefaultRules_per_experience(i)<<"  ";
  }
  out<<endl;
}

void RuleSetContainer_ground::write_experiencesWithRules(ostream& os) const {
  uint i, k;
  FOR1D((*p_examples), i) {
    os<<"--------------"<<endl;
    os<<"EXAMPLE #"<<i<<":"<<endl;
    ((*p_examples)(i))->write(os);
    os<<"COVERING RULES: "<<nonDefaultRules_per_experience(i)<<endl;
    FOR1D(nonDefaultRules_per_experience(i), k) {
      rules.elem(nonDefaultRules_per_experience(i)(k))->write(os);
    }
  }
}

void RuleSetContainer_ground::write_rulesWithExperiences(ostream& os) const {
  uint i, k;
  FOR1D_(rules, i) {
    os<<"--------------"<<endl;
    os<<"RULE #"<<i<<":"<<endl;
    rules.elem(i)->write(os);
    os<<"COVERS "<<experiences_per_rule(i).N<<" examples."<<endl;
    FOR1D(experiences_per_rule(i), k) {
      os<<"Example #"<<experiences_per_rule(i)(k)<<endl;
      ((*p_examples)(experiences_per_rule(i)(k)))->write(os);
    }
  }
}

void RuleSetContainer_ground::sanityCheck() const {
  ruleReasoning::checkRules(rules);
  
  if (rules.num() != experiences_per_rule.N) {
    cout<<"FAILED SANITY CHECK:"<<endl;
    writeNice(cout, true);
    HALT("sanity check failed 0-A");
  }
  if (p_examples->N != nonDefaultRules_per_experience.N) {
    cout<<"FAILED SANITY CHECK:"<<endl;
    writeNice(cout, true);
    HALT("sanity check failed 0-B");
  }
  if (rules.num() != experiences_per_ruleOutcome.N) {
    cout<<"FAILED SANITY CHECK:"<<endl;
    writeNice(cout, true);
    HALT("sanity check failed 0-C");
  }
  
  uint i, k;
  // Non-default rules
  FOR1D_(rules, i) {
    if (i == 0)
      continue;
    FOR1D((*p_examples), k) {
      SubstitutionSet subs;
      bool supposed_to_cover__1 = experiences_per_rule(i).findValue(k) >= 0;
      bool supposed_to_cover__2 = nonDefaultRules_per_experience(k).findValue(i) >= 0;
      if (supposed_to_cover__1 != supposed_to_cover__2) {
        cout<<"FAILED SANITY CHECK:"<<endl;
        writeNice(cout, true);
        PRINT(i);
        PRINT(k);
        PRINT(experiences_per_rule(i));
        PRINT(nonDefaultRules_per_experience(k));
        PRINT(supposed_to_cover__1);
        PRINT(supposed_to_cover__2);
        HALT("sanity check failed 1");
      }
      bool covers = ruleReasoning::cover_groundRule_groundedAction((*p_examples)(k)->pre, (*p_examples)(k)->action, rules.elem(i));
      if (covers != supposed_to_cover__1) {
        cout<<"FAILED SANITY CHECK:"<<endl;
        writeNice(cout, false);
        cout<<"Rule i="<<i<<endl;
        cout<<"Example k="<<k<<endl;
        PRINT(supposed_to_cover__1);
        PRINT(covers);
        HALT("sanity check failed 2");
      }
    }
    uint total_examples_in_outcomes = 0;
    FOR1D(experiences_per_ruleOutcome(i), k) {
      total_examples_in_outcomes += experiences_per_ruleOutcome(i)(k).N;
      uint l;
      FOR1D(experiences_per_ruleOutcome(i)(k), l) {
        if (experiences_per_rule(i).findValue(experiences_per_ruleOutcome(i)(k)(l)) < 0) {
          cout<<"FAILED SANITY CHECK:"<<endl;
          writeNice(cout, false);
          PRINT(i);
          PRINT(k);
          PRINT(l);
          PRINT(experiences_per_ruleOutcome(i)(k)(l));
          PRINT(experiences_per_ruleOutcome(i)(k));
          PRINT(experiences_per_ruleOutcome(i));
          PRINT(experiences_per_rule(i));
          HALT("sanity check failed 3");
        }
      }
    }
    if (total_examples_in_outcomes < experiences_per_rule(i).N) {
      cout<<"FAILED SANITY CHECK:"<<endl;
      writeNice(cout, false);
      PRINT(i);
      PRINT(total_examples_in_outcomes);
      PRINT(experiences_per_ruleOutcome(i));
      PRINT(experiences_per_rule(i));
      HALT("sanity check failed 4");
    }
  }
  
  // Check that default rule does not cover examples which other rules cover
  FOR1D(experiences_per_rule(0), i) {
    FOR1D_(rules, k) {
      if (k == 0)
        continue;
      if (experiences_per_rule(k).findValue(experiences_per_rule(0)(i)) >= 0) {
        cout<<"FAILED SANITY CHECK:"<<endl;
        writeNice(cout, false);
        PRINT(k);
        PRINT(experiences_per_rule(0));
        PRINT(experiences_per_rule(k));
        PRINT(i);
        PRINT(experiences_per_rule(0)(i));
        HALT("sanity check failed 5  --  default rule A");
      }
    }
  }
  
  if (ruleReasoning::isDefaultRule(rules.elem(0))) {
    FOR1D(nonDefaultRules_per_experience, i) {
      if (nonDefaultRules_per_experience(i).findValue(0) >= 0) {
        cout<<"FAILED SANITY CHECK:"<<endl;
        writeNice(cout, false);
        PRINT(i);
        PRINT(nonDefaultRules_per_experience);
        HALT("sanity check failed 6  --  default rule B");
      }
    }
  }
  
}



/*void RuleSetContainer_ground::sort() {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"sort [START]"<<endl;}
  RuleSet new__rules;
  MT::Array< uintA > new__nonDefaultRules_per_experience;
  MT::Array< uintA > new__experiences_per_rule;
  MT::Array< MT::Array < uintA > > new__experiences_per_ruleOutcome;
  
  uintA action_ids;
  uint i, k;
  FOR1D_(rules, i) {
    action_ids.setAppend(rules.elem(i)->action->pred->id);
  }
  sort_desc(action_ids);  // descending
  CHECK_EQ(action_ids(0) , DEFAULT_ACTION_PRED__ID, "");
  
  Substitution sub;
  FOR1D(action_ids, i) {
    FOR1D_(rules, k) {
      if (rules.elem(k)->action->pred->id == action_ids(i)) {
        new__rules.append(rules.elem(k));
        sub.addSubs(k, new__rules.num()-1);
        new__experiences_per_rule.append(experiences_per_rule(k));
        new__experiences_per_ruleOutcome.append(experiences_per_ruleOutcome(k));
      }
    }
  }
  new__nonDefaultRules_per_experience = nonDefaultRules_per_experience;
  FOR1D(new__nonDefaultRules_per_experience, i) {
    FOR1D(new__nonDefaultRules_per_experience(i), k) {
      new__nonDefaultRules_per_experience(i)(k) = sub.getSubs(new__nonDefaultRules_per_experience(i)(k));
    }
  }
  
  rules = new__rules;
  experiences_per_rule = new__experiences_per_rule;
  experiences_per_ruleOutcome = new__experiences_per_ruleOutcome;
  nonDefaultRules_per_experience = new__nonDefaultRules_per_experience;
  
  sanityCheck();
  if (DEBUG>0) {cout<<"sort [END]"<<endl;}
}
*/

void RuleSetContainer_ground::sort() {
  uint max_arity = 0;
  uint r2;
  FOR1D_(rules, r2) {
    if (rules.elem(r2)->action->pred->d > max_arity)
      max_arity = rules.elem(r2)->action->pred->d;
  }
  
  if (max_arity <= 2) {
    MT::Array< uintA > new__nonDefaultRules_per_experience;
    MT::Array< uintA > new__experiences_per_rule;
    MT::Array< MT::Array < uintA > > new__experiences_per_ruleOutcome;
    RuleSet rules_sorted;
    uintA action_ids;
    uint r, i;
    uint id;
    Substitution sub;
    FOR1D_(rules, r) {
      if (rules.elem(r)->action->pred->id == DEFAULT_ACTION_PRED__ID)
        continue;
      if (rules.elem(r)->action->pred->d == 0)
        id = rules.elem(r)->action->pred->id * 100;
      else if (rules.elem(r)->action->pred->d == 1)
        id = rules.elem(r)->action->pred->id * 100  +  rules.elem(r)->action->args(0);
      else if (rules.elem(r)->action->pred->d == 2)
        id = rules.elem(r)->action->pred->id * 10000  +  100 * rules.elem(r)->action->args(0) + rules.elem(r)->action->args(1);
      else
        HALT("");
      if (action_ids.findValue(id) >= 0)
        continue;
      FOR1D(action_ids, i) {
        if (action_ids(i) > id) {
          action_ids.insert(i, id);
          break;
        }
      }
      if (action_ids.N == i)
        action_ids.append(id);
    }
    
    // DEFAULT RULE always first rule
    uint DEFAULT_ACTION_ID = DEFAULT_ACTION_PRED__ID * 100;
    action_ids.insert(0, DEFAULT_ACTION_ID);
    
    FOR1D(action_ids, i) {
      FOR1D_(rules, r) {
        if (rules.elem(r)->action->pred->d == 0) {
          if (rules.elem(r)->action->pred->id * 100 == action_ids(i)) {
            rules_sorted.append(rules.elem(r));
            new__experiences_per_rule.append(experiences_per_rule(r));
            new__experiences_per_ruleOutcome.append(experiences_per_ruleOutcome(r));
            sub.addSubs(r, rules_sorted.num()-1);
          }
        }
        else if (rules.elem(r)->action->pred->d == 1) {
          if (rules.elem(r)->action->pred->id * 100  +  rules.elem(r)->action->args(0) == action_ids(i)) {
            rules_sorted.append(rules.elem(r));
            new__experiences_per_rule.append(experiences_per_rule(r));
            new__experiences_per_ruleOutcome.append(experiences_per_ruleOutcome(r));
            sub.addSubs(r, rules_sorted.num()-1);
          }
        }
        else if (rules.elem(r)->action->pred->d == 2) {
          if (rules.elem(r)->action->pred->id * 10000  +  100 * rules.elem(r)->action->args(0) + rules.elem(r)->action->args(1) == action_ids(i)) {
            rules_sorted.append(rules.elem(r));
            new__experiences_per_rule.append(experiences_per_rule(r));
            new__experiences_per_ruleOutcome.append(experiences_per_ruleOutcome(r));
            sub.addSubs(r, rules_sorted.num()-1);
          }
        }
        else
          HALT("");
      }
    }
    rules = rules_sorted;
    
    new__nonDefaultRules_per_experience = nonDefaultRules_per_experience;
    FOR1D(new__nonDefaultRules_per_experience, i) {
      FOR1D(new__nonDefaultRules_per_experience(i), r) {
        new__nonDefaultRules_per_experience(i)(r) = sub.getSubs(new__nonDefaultRules_per_experience(i)(r));
      }
    }
    
    experiences_per_rule = new__experiences_per_rule;
    experiences_per_ruleOutcome = new__experiences_per_ruleOutcome;
    nonDefaultRules_per_experience = new__nonDefaultRules_per_experience;
  }
  // max_arity > 2
  else {
    NIY;
    write(rules, "ground_rules.dat.unsorted_backup");
    
    MT::Array< Rule* > ra_sorted;
    ra_sorted.memMove = true;
    uint r, r2, d;
    FOR1D_(rules, r) {
      bool inserted = false;
      FOR1D(ra_sorted, r2) {
        // put before action-predicate with higher id
        if (rules.elem(r)->action->pred->id > ra_sorted(r2)->action->pred->id) {
          ra_sorted.insert(r2, rules.elem(r));
          inserted = true;
        }
        // same action-predicate
        else if (rules.elem(r)->action->pred->id == ra_sorted(r2)->action->pred->id) {
          FOR1D(rules.elem(r)->action->args, d) {
            if (rules.elem(r)->action->args(d) < ra_sorted(r2)->action->args(d)) {
              ra_sorted.insert(r2, rules.elem(r));
              inserted = true;
              break;
            }
          }
        }
        if (inserted)
          break;
      }
      if (!inserted)
        ra_sorted.append(rules.elem(r));
      
//       FOR1D(ra_sorted, r3) {
//         ra_sorted(r3)->action->write(); cout<<"  ";
//       }
//       cout<<endl;
    }
    
//     FOR1D(ra_sorted, r3) {
//       ra_sorted(r3)->action->write(); cout<<"  ";
//     }
//     cout<<endl;
    
    CHECK_EQ(rules.num() , ra_sorted.N, "Some strange rule-sorting mistake.");
    RuleSet rules_sorted;
    uint i;
    FOR1D(ra_sorted, i) {
      rules_sorted.append(ra_sorted(i));
    }
    rules = rules_sorted;
  }
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    C O S T   F U N C T I O N
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


SymbolicExperienceL cf_examples_coveredByCurrentRule_ground;
boolA cf_coverage_outcome_example_ground;
double cf_sum_ground;
double cf_pos_ground;
double cf_p_min_ground;

// lower bound p_min for state transition given noise outcome
void CostFunction_ground::setNoiseStateProbability(double p_min) {
	cf_p_min_ground = p_min;
}

void CostFunction_ground::setPenaltySum(double pen_sum) {
	cf_sum_ground = pen_sum;
}

void CostFunction_ground::setPenaltyPos(double pen_pos) {
	cf_pos_ground = pen_pos;
}

void CostFunction_ground::setRuleCoveredExamples(const SymbolicExperienceL& coveredEx) {
	cf_examples_coveredByCurrentRule_ground.clear();
  cf_examples_coveredByCurrentRule_ground= coveredEx;
}

void CostFunction_ground::setOutcomesCoverage(const boolA& coverage) {
	cf_coverage_outcome_example_ground = coverage;
}


double CostFunction_ground::loglikelihood(const arr& probs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"loglikelihood [START]"<<endl;
  double loglikelihood = 0.0, innerSum;
  uint i,e;
  FOR1D(cf_examples_coveredByCurrentRule_ground, e) {
    innerSum = 0.0;
    if (DEBUG>0) cout<<"ex "<<e<<":"<<endl;
    FOR1D(probs, i) {
      if (cf_coverage_outcome_example_ground(i, e)) {
        if (i < probs.N - 1) {
          innerSum += probs(i);
          if (DEBUG>0) cout<<"  o"<<i<<" 1 * "<<probs(i)<<endl;
        }
        else { // noise outcome
          innerSum += cf_p_min_ground * probs(i);
          if (DEBUG>0) cout<<"  o"<<i<<" "<<cf_p_min_ground<<" * "<<probs(i)<<endl;
        }
      }
    }
    if (DEBUG>0) cout<<" lik="<<innerSum<<"   log(lik)="<<log(innerSum)<<endl;
    loglikelihood += log(innerSum);
  }
  if (DEBUG>0) PRINT(loglikelihood)
  if (DEBUG>0) cout<<"loglikelihood [END]"<<endl;
  return loglikelihood;
}


// see Latex-paper for mathematical details
// to minimize!!!
double CostFunction_ground::calc(const arr& in) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"CostFunction_ground::calc [START]"<<endl;
  CHECK_EQ(in.N , cf_coverage_outcome_example_ground.d0, "invalid number of arguments")
  uint i;
  // calc log-likelihood
  double loglik = loglikelihood(in);
  double sumConstraint = cf_sum_ground * pow(sum(in) - 1.0, 2);
  double posConstraint = 0.0;
  FOR1D(in, i) {
    if (in(i) < 0.0) {
      posConstraint += pow(in(i), 2);
    }
  }
  posConstraint *= cf_pos_ground;
  double cost = -loglik + sumConstraint + posConstraint;
  if (DEBUG>0) cout<<"cost="<<cost<<" (-loglik="<<-loglik<<", sumConstraint="<<sumConstraint<<", posConstraint="<<posConstraint<<")"<<endl;
  if (DEBUG>0) cout<<"CostFunction_ground::calc [END]"<<endl;
  return cost;
}


// points into direction of steepest ascent
void CostFunction_ground::calc_grad(arr& out, const arr& in) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calc_grad [START]"<<endl;
  CHECK_EQ(in.N , cf_coverage_outcome_example_ground.d0, "invalid number of arguments")
  out.resize(in.N);
  double sumIn = sum(in);
  double loglik_grad, const1_grad, const2_grad, denom_sum;
  uint i, i2, e;
  // calc gradient for each prob
  FOR1D (in, i) {
    // check for all examples
    loglik_grad = 0.0;
    if (i < in.N - 1) {
      CHECK(sum(cf_coverage_outcome_example_ground.sub(i,i,0,cf_coverage_outcome_example_ground.d1-1)), "At least one example should be covered!");
    }
    FOR1D(cf_examples_coveredByCurrentRule_ground, e) {
      // I[covers(s', o_i)]
      if (cf_coverage_outcome_example_ground(i, e)) {
        // denominator sum
        denom_sum = 0.0;
        FOR1D(in, i2) {
          if (cf_coverage_outcome_example_ground(i2, e)) {
            if (i2 < in.N - 1) // non-noise
              denom_sum += in(i2);
            else // noise outcome
              denom_sum += cf_p_min_ground * in(i2);
          }
        }
        if (i < in.N-1) // non-noise
          loglik_grad += 1. / denom_sum;
        else // noise
          loglik_grad += cf_p_min_ground / denom_sum;
      }
    }
    
    // constraint 1
    const1_grad = 2. * cf_sum_ground * (sumIn - 1.0) * in(i);
    
    // constraint 2
    if (in(i) < 0.0) {
      const2_grad = cf_pos_ground * 2. * in(i);
    }
    else
      const2_grad = 0.0;
    out(i) = - loglik_grad + const1_grad + const2_grad;

    if (DEBUG>1) {
      cout << i << ": grad=" << out(i) << " -loglik=" << (-loglik_grad) << " sumPen=" << const1_grad << " posPen=" << const2_grad << endl;
    }

  }
  if (DEBUG>0) cout<<"calc_grad [END]"<<endl;
}












// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    S E A R C H   O P E R A T O R
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


SearchOperator_ground::SearchOperator_ground(double alpha_PEN, double p_min, uint param_opt_type) {
  this->alpha_PEN = alpha_PEN;
  this->p_min = p_min;
  this->param_opt_type = param_opt_type;
  this->approximative = false;
  
  // TODO adapt these guys...
  this->pen_sum = 20.0;
  this->pen_pos = 20.0;
}








void SearchOperator_ground::calcCoverage_outcomes(const MT::Array< LitL >& potential_outcomes, const SymbolicExperienceL& covered_experiences, const Rule* old_rule, boolA& coverage) {
  // ACHTUNG: old_rule hat womoeglich andere Outcomes!!
  uint DEBUG = 0;
  if (DEBUG>0) cout << "calcOutcomesCoverage [START]" << endl;
  if (DEBUG>1) {write(old_rule->context);cout<<endl;old_rule->action->write(cout);cout<<endl;}
  coverage.resize(potential_outcomes.N, covered_experiences.N);
  uint i, e;
  FOR1D(covered_experiences, e) {
    if (DEBUG>0) {cout<<"### Ex "<<e<<":"<<endl; covered_experiences(e)->write(cout);}
    // provide the initial substitution given by action and context --> setting all vars
    SubstitutionSet subs;
    bool covered = ruleReasoning::cover_groundRule_groundedAction(covered_experiences(e)->pre, covered_experiences(e)->action, old_rule);
    if (!covered) {
      HALT("Example must be preAct_covered by rule!");
    }
    FOR1D(potential_outcomes, i) {
      if (DEBUG>1) {cout<<"+++ Potential outcome: ";write(potential_outcomes(i));cout<<endl;}
      if (i == potential_outcomes.N-1) {
        // nur falls nicht gecovert, covert das Noise outcome
        coverage(i, e) = true;
        uint l;
        for (l=0; l<potential_outcomes.N-1; l++) {
          if (coverage(l, e)) {
            coverage(i, e) = false;
            break;
          }
        }
      }
      else {
        SymbolicState successor;
        ruleReasoning::calcSuccessorState(covered_experiences(e)->pre, potential_outcomes(i), successor, false);
        if (DEBUG>2) { cout<<"\nPOST:";covered_experiences(e)->post.write(cout);cout<<endl;
          cout<<"SUCC:";successor.write(cout);cout<<endl;}
        if (covered_experiences(e)->post == successor)
          coverage(i, e) = true;
        else
          coverage(i, e) = false;
      }
      if (DEBUG>1) cout<<" "<<coverage(i,e)<<endl;
    }
  }
  if (DEBUG>0) cout << "calcOutcomesCoverage [END]" << endl;
}



void SearchOperator_ground::calcSubsumption(boolA& subsumes, const boolA& coverage) {
  CHECK_EQ(coverage.nd , 2, "invalid coverage matrix")
  subsumes.resize(coverage.d0, coverage.d0);
  subsumes.setUni(false);
  uint i, j, k;
  FOR1D(subsumes, i) {
    subsumes(i,i) = true;
    for (j=i+1; j<subsumes.d0; j++) {
      subsumes(i,j) = true;
      subsumes(j,i) = true;
      for (k=0; k<coverage.d1; k++) {
        if (coverage(i, k) < coverage(j, k))
          subsumes(i,j) = false;
        if (coverage(j, k) < coverage(i, k))
          subsumes(j,i) = false;
      }
      if (!subsumes(i,j) && !subsumes(j,i))
        break;
    }
  }
}









void SearchOperator_ground::integrateNewRules(const RuleSetContainer_ground& rulesC_old, const RuleSetContainer_ground& rulesC_2add, 
                                       const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_new) {
  uint DEBUG = 0;
  if (DEBUG > 0) {cout << "integrateNewRules [START]" << endl;}
  if (DEBUG > 1) {
    cout << "***** Really old rules *****" << endl;
    rulesC_old.writeNice(cout, true);
    cout << "***** rulesC_2add *****" << endl;
    rulesC_2add.writeNice(cout, true);
    cout << "------" << endl;
  }
  rulesC_new.clear();
  uint i, j;
  // Create a copy of the input rule-set
  rulesC_new = rulesC_old;
  
//  write(rulesC_old);
//  cout << endl;
  // For each new rule r'
  FOR1D_(rulesC_2add.rules, i) {
    // Remove rules in R' that cover any examples r' covers
    uintA& covered_new = rulesC_2add.experiences_per_rule(i);
    // Clean-up rules
    ruleReasoning::cleanup(*rulesC_2add.rules.elem(i));
    // SPECIAL: default rule (at pos 0) is never removed
    for (j=rulesC_new.rules.num()-1; j>0; j--) {
      uintA& covered_old = rulesC_new.experiences_per_rule(j);
      if (numberSharedElements(covered_new, covered_old) > 0) {
        rulesC_new.remove(j);
      }
    }
    // Add r' to R'
    rulesC_new.append(rulesC_2add.rules.elem(i), rulesC_2add.experiences_per_rule(i), rulesC_2add.experiences_per_ruleOutcome(i));
  }
  // Recompute the set of examples that the default rule in R' covers and the parameters of this default ruleLearner
  rulesC_new.recomputeDefaultRule();
  
  if (DEBUG > 0) {
    if (DEBUG>1) {
      cout << "***** Old rules *****" << endl;
      rulesC_old.writeNice(cout, true);
    }
    cout << "***** New rules *****" << endl;
    rulesC_new.writeNice(cout, true);
  }
  
//   rulesC_new.sanityCheck();
  
  if (DEBUG > 0) {cout << "integrateNewRules [END]" << endl;}
}




// Algorithm of Figure 4 in Zettlemoyer (2007)
void SearchOperator_ground::createRuleSets(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, 
        MT::Array< RuleSetContainer_ground >& set_of_new_rulesC) {
  uint DEBUG = 0;
  set_of_new_rulesC.clear();
  reset();
  while (true) {
    RuleSetContainer_ground rulesC_2add(&examples);
    // this is where the local knowledge of the individual search operators comes in
    findRules(rulesC_old, examples, rulesC_2add);
    uint i;
    if (DEBUG>1) {
      FOR1D_(rulesC_2add.rules, i) {
        rulesC_2add.rules.elem(i)->write(cout);
      }
    }
    if (rulesC_2add.rules.num() == 0)
      break;
    RuleSetContainer_ground rulesC_new(&examples);
    integrateNewRules(rulesC_old, rulesC_2add, examples, rulesC_new);
    set_of_new_rulesC.append(rulesC_new);
    if (approximative) {
      if (set_of_new_rulesC.d0 >= APPROXIMATOR__RULES_PER_ROUND)
        break;
    }
  }
}







// remove outcomes that (i) do not cover any example and (ii) have zero-probability  and (iii) sets coverage for cost function
void SearchOperator_ground::produceTrimmedOutcomes(MT::Array< LitL >& outcomes, arr& probs, boolA& coverage, const SymbolicExperienceL& coveredExamples, const Rule& rule) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"produceTrimmedOutcomes [START]"<<endl;
  uint i;
  bool removed, atleastone;
  if (DEBUG>1) {cout<<"Outcomes before:\n"; write(outcomes);}
  
  // (i) remove outcomes that do not cover any example
  if (DEBUG>1) cout<<"Removing outcomes that do not cover any example"<<endl;
  calcCoverage_outcomes(outcomes, coveredExamples, &rule, coverage);
  CostFunction_ground::setOutcomesCoverage(coverage);
  if (DEBUG>3) PRINT(coverage)
  removed = false;
  MT::Array< LitL > o_help;
  FOR1D(outcomes, i) {
    atleastone = sum(coverage.sub(i,i,0,coverage.d1-1));
    if (i < outcomes.N-1  &&  !atleastone) { // only for non-noise outcomes
      removed = true;
      if (DEBUG>2) cout << "Outcome " << i << " covers 0 examples and will be removed."<<endl;
    }
    else
      o_help.append(outcomes(i));
  }
  if (DEBUG>1) {cout<<"Outcomes now:\n"; write(o_help);}
  if (removed) {
    calcCoverage_outcomes(o_help, coveredExamples, &rule, coverage);
    CostFunction_ground::setOutcomesCoverage(coverage);
    if (DEBUG>3) PRINT(coverage)
  }

  // (ii) remove zero-probability outcomes q
  // learn params
  learnParameters(o_help, probs);
  if (DEBUG>1) cout<<"Removing zero-prob outcomes"<<endl;
  // remove zero-prob outcomes (except default outcome which can never be deleted)
  removed = false;
  outcomes.clear();
  FOR1D(o_help, i) {
    if (i!=o_help.N-1  &&  TL::isZero(probs(i))) {
      probs.remove(i);
      removed = true;
    }
    else {
      outcomes.append(o_help(i));
    }
  }
	
  if (removed) {
    calcCoverage_outcomes(outcomes, coveredExamples, &rule, coverage);
    CostFunction_ground::setOutcomesCoverage(coverage);
    if (DEBUG>3) PRINT(coverage)
  }
  if (DEBUG>1) {cout<<"Outcomes final:\n"; write(outcomes);}
  if (DEBUG>0) cout<<"produceTrimmedOutcomes [END]"<<endl;
}




// TODO list:
//  - cope with comparisons
// a la Zettlemoyer 2004
//
// Representation of NOISE outcome: empty outcome list && last element in outcomes
//
//
// Steps:
// (1) Determine basic outcomes: changes from pre to post in examples
// (2) Collapse identical outcomes
// (3) Greedily improve outcomes based on add- and remove-operator
//
// When we create new outcomes, we have to check whether there are outcomes with
// (i) no examples covered or (ii) a probability of 0. In both cases, the corresponding
// outcomes need to be deleted. --> method trimOutcomes(...), see above
void SearchOperator_ground::induceOutcomes(Rule* ground_r, MT::Array< uintA >& coveredExamples_per_outcome, const SymbolicExperienceL& coveredExamples, const uintA& covered_experiences_ids) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "induceOutcomes [START]" << endl;
  
  CHECK_EQ(covered_experiences_ids.N , coveredExamples.N, "");
  CHECK(coveredExamples.N>0, "No examples, bro!");
    
  // calc changes first per example
  uint i, j;
  if (DEBUG > 0) {ground_r->write(cout);  PRINT(coveredExamples.N);}
  
  // prepare cost function (stays like this for complete following induceOutcomes-procedure)
  CostFunction_ground::setRuleCoveredExamples(coveredExamples);
  CostFunction_ground::setPenaltySum(pen_sum * coveredExamples.N);
  CostFunction_ground::setPenaltyPos(pen_pos * coveredExamples.N);
  CostFunction_ground::setNoiseStateProbability(p_min);
      
  // (1) Determine basic outcomes = changes from pre to post = for each covered example a separate outcome
  MT::Array< LitL > outcomes_basic;
  FOR1D(coveredExamples, i) {
    if (DEBUG>4) {cout<<"====== Using ex "<<i<<":"<<endl; coveredExamples(i)->write(cout);}
    bool covered = ruleReasoning::cover_groundRule_groundedAction(coveredExamples(i)->pre, coveredExamples(i)->action, ground_r);
    if (!covered) {
      HALT("An uncovered example! Be careful when calling methods, noob!");
    }

    LitL nextOutcome;
      
    // insert postOnly-predicates
    FOR1D(coveredExamples(i)->add, j) {
      if (coveredExamples(i)->add(j)->atom->pred->type == Predicate::predicate_simple) {
        nextOutcome.append(coveredExamples(i)->add(j));
      }
    }
    // insert negations of preOnly-predicates
    FOR1D(coveredExamples(i)->del, j) {
      if (coveredExamples(i)->del(j)->atom->pred->type == Predicate::predicate_simple) {
        nextOutcome.append(logicObjectManager::getLiteralNeg(coveredExamples(i)->del(j)));
      }
    }
    outcomes_basic.append(nextOutcome);

    if (DEBUG>3) {
      cout<<"nextOutcome: "<<nextOutcome<<endl;
      cout<<"nextOutcome_pureAbstract: "<<outcomes_basic<<endl;
    }
  }
  // add noise outcomes
  LitL noiseOutcome;
  outcomes_basic.append(noiseOutcome);
  
  if (DEBUG > 0) {
    cout << "Examples outcomes (incl. noise outcome):" << endl;
    FOR1D(outcomes_basic, i) {
      cout << "(" << i << ") ";
      write(outcomes_basic(i));
      cout << endl;
    }
  }
  
  
  // (2) Collapse identical outcomes
    // ignore noisy outcome
  boolA prune(outcomes_basic.d0);
  prune.setUni(false);
  FOR1D(outcomes_basic, i) {
    if (i==outcomes_basic.N-1)
      break;
    if (prune(i))
      continue;
    for (j=i+1; j<outcomes_basic.d0-1; j++) {
      if (prune(j))
        continue;
      if (equivalent(outcomes_basic(i), outcomes_basic(j)))
        prune(j) = true;
    }
  }
  MT::Array< LitL > outcomes;
  FOR1D(outcomes_basic, i) {
    if (!prune(i)) {
      outcomes.append(outcomes_basic(i));
    }
  }
  if (DEBUG > 0) {
    cout << "Collapsed examples outcomes (copies removed)  (incl. noise outcome):" << endl;
    FOR1D(outcomes, i) {cout << "(" << i << ") "; write(outcomes(i)); cout << endl;}
  }

  // trim outcomes
  arr probs;
  boolA coverage;
  produceTrimmedOutcomes(outcomes, probs, coverage, coveredExamples, *ground_r);
  
  
  // (3) Greedily improve outcomes
  // score needs to be optimized

  double score, bestScore;
  double loglik;
  
  // evaluate and calc score
  loglik = CostFunction_ground::loglikelihood(probs);
  score = loglik - this->alpha_PEN * logicReasoning::numberLiterals(outcomes);
  if (DEBUG > 1) {
    cout << "\nBASIC OUTCOMES:" << endl;
    FOR1D(outcomes, i) {
      cout << "("<< i << ") ";
      write(outcomes(i));
      cout << " " << probs(i);
      cout << endl;
    }
    cout << " --> score=" << score << endl;
  }
  
  bestScore = score;
  bool perform_add = true;  // war urspruenglich false!!
  bool add_possible = true;
  bool remove_possible = true;
  bool change = false;
  boolA subsumes;
  arr probs_new;
  MT::Array< LitL > outcomes_new;
  boolA coverage_new;
  do {
    change = false;
    outcomes_new.clear();
    // ADD [start]
    if (perform_add) {
      if (DEBUG>4) cout<<"Outcome adding"<<endl;
      boolA unifiable(outcomes.N-1, outcomes.N-1);
      uint numUnifiables = 0;
      unifiable.setUni(false);
      // calculate which ones are unifiable
      FOR1D(unifiable, i) {
        for(j=i+1; j<unifiable.d1; j++) {
          unifiable(i,j) = logicReasoning::nonContradicting(outcomes(i), outcomes(j));
          if (unifiable(i,j))
            numUnifiables++;
        }
      }
      // randomly choose two and unify them
      if (numUnifiables>0) {
        change = true;
        int ix = rnd.num(numUnifiables);
        bool found = false;
        for(i=0; i<unifiable.d0; i++) {
          for(j=i+1; j<unifiable.d1; j++) {
            if (unifiable(i,j)) {
              if (ix-- == 0) {
                found = true; // we'll combining outcomes i and j
                break;
              }
            }
          }
          if (found)
            break;
        }
        LitL unifiedOutcome;
        unifiedOutcome.append(outcomes(i));
        unifiedOutcome.append(outcomes(j));
        logicReasoning::removeRedundant(unifiedOutcome); // ist das effizienet?
        outcomes_new.append(unifiedOutcome);
        uint o;
        FOR1D(outcomes, o) {
          if (o==i || o==j)
            continue;
          else
            outcomes_new.append(outcomes(o));
        }
        if (DEBUG>4) {
          PRINT(numUnifiables)
          PRINT(unifiable)
          cout<<"We gonna add:"<<endl;
          cout<<i<<" :";write(outcomes(i));cout<<endl;
          cout<<j<<" :";write(outcomes(j));cout<<endl;
          cout<<"  --> ";write(unifiedOutcome);cout<<endl;
        }
      }
    }
    // ADD [end]
    // REMOVE [start]
    else {
      if (DEBUG>4) cout<<"Outcome deleting"<<endl;
      // remove condition: outcome overlapping with other outcomes on every example
      // calc which outcomes overlap each other completely
      calcSubsumption(subsumes, coverage);
      // Randomly determine overlapped outcome that is to delete.
      // As usual, ignore default outcome.
      boolA overlapped(outcomes.N-1);
      overlapped.setUni(false);
      for (i=0; i<outcomes.N-1; i++) {
        for (j=0; j<outcomes.N-1; j++) {
          if (j==i)
            continue;
          if (subsumes(j,i)) {
            overlapped(i) = true;
            break;
          }
        }
      }
      uint numOverlapped = 0;
      FOR1D(overlapped,i)
        if (overlapped(i))
          numOverlapped++;
      if (DEBUG > 2) {
        PRINT(coverage_new)
        PRINT(subsumes)
        PRINT(overlapped)
        PRINT(numOverlapped)
      }
      if (numOverlapped > 0) {
        uint overlapped2delete = rnd.num(numOverlapped);
        if (DEBUG>1) {cout<<"Delete overlapped outcome #" << overlapped2delete << endl;}
        numOverlapped = 0;
        FOR1D(outcomes, i) {
          // always keep default outcome:
          if (i==outcomes.N-1) {
            outcomes_new.append(outcomes(i));
            break;
          }
          // other outcomes:
          else if (overlapped(i)) {
            if (numOverlapped != overlapped2delete) {
              outcomes_new.append(outcomes(i));
            }
            else {
              CHECK(!change, "Deleting twice is not allowed.")
              // HERE IS THE ESSENTIAL LINE WHERE WE DELETE!
              change = true;
            }
            numOverlapped++;
          }
          else
            outcomes_new.append(outcomes(i));
        }
      }
    }
    // REMOVE [end]
    
    // POSTPROCESSING OF NEW OUTCOME
    
    // If no new outcome found:
    // (1) Stop, if no more modifications possible (i.e. neither remove nor add)
    // (2) Change to remove or add
    if (!change) {
      bool stop = false;
      if (perform_add) {
        if (!remove_possible)
          stop = true;
        else {
          add_possible = false;
          perform_add = false;
          continue;
        }
      }
      else {
        if (!add_possible)
          stop = true;
        else {
          remove_possible = false;
          perform_add = true;
          continue;
        }
      }
      if (stop) {
        break;
      }
    }
    // Outcomes _have_ changed.
    else {
      if (DEBUG > 2) {
        cout << "\nNEW OUTCOMES:" << endl;
        FOR1D(outcomes_new, i) {
          cout << i << ": ";
          write(outcomes_new(i));
          cout << endl;
        }
      }
      
      // procduce trimmed outcomes
      produceTrimmedOutcomes(outcomes_new, probs_new, coverage_new, coveredExamples, *ground_r);
            
      // evaluate and calc score
      loglik = CostFunction_ground::loglikelihood(probs_new);
      score = loglik - this->alpha_PEN * logicReasoning::numberLiterals(outcomes_new);
            
      if (DEBUG > 1) {
        cout << "\nNEW OUTCOMES (now with new_probs and score):" << endl;
        FOR1D(outcomes_new, i) {
          cout << i << ": ";
          write(outcomes_new(i));
                    cout << " " << probs_new(i);
          cout << endl;
        }
        cout << " --> score=" << score << endl;
      }
      
      if (score > bestScore  /*||   (!perform_add  &&  score > bestScore)*/) {
        if (DEBUG>1) cout<<"New outcomes accepted."<<endl;
        outcomes = outcomes_new;
        coverage = coverage_new;
        probs = probs_new;
        bestScore = score;
        if (perform_add)
          remove_possible = true; // ensures that we will try the remove-operator again
        else
          add_possible = true;
      }
      else {
        if (DEBUG>1) cout<<"New outcomes rejected."<<endl;
        if (perform_add)
          add_possible = false;
        else
          remove_possible = false;
        if (!add_possible && !remove_possible)
          break;
        perform_add = !perform_add;
      }
    }
  } while (outcomes.N > 1);
  
//     CHECK(outcomes.N>1, "No non-noise outcome!")
    
  // set outcomes in rule
  ground_r->outcomes = outcomes;
  ground_r->probs = probs;
  
  // Calc examples_per_outcome
  coveredExamples_per_outcome.clear();
  coveredExamples_per_outcome.resize(coverage.d0);
  uint i_out, i_ex;
  FOR2D(coverage, i_out, i_ex) {
    if (coverage(i_out, i_ex)) {
      coveredExamples_per_outcome(i_out).append(covered_experiences_ids(i_ex));
    }
  }
  
  
  if (DEBUG > 0) {
    cout << "\nFINAL NEW OUTCOMES (now with probs and score):" << endl;
    PRINT(covered_experiences_ids);
    PRINT(coveredExamples_per_outcome);
    FOR1D(ground_r->outcomes, i) {
      cout << i << ": ";
      write(ground_r->outcomes(i));
      cout << " " << ground_r->probs(i) << " " << coveredExamples_per_outcome(i);
      cout << endl;
    }
    cout << " --> score=" << bestScore << endl;
  }
  
  // check [START]
  uint used_outcomes = 0;
  FOR1D(coveredExamples_per_outcome, i) {
    used_outcomes += coveredExamples_per_outcome(i).N;
  }
  if (used_outcomes < coveredExamples.N) {
    PRINT(used_outcomes);
    PRINT(covered_experiences_ids);
    PRINT(coveredExamples_per_outcome);
    FOR1D(ground_r->outcomes, i) {
      cout << i << ": ";
      write(ground_r->outcomes(i));
      cout << " " << ground_r->probs(i) << " " << coveredExamples_per_outcome(i);
      cout << endl;
    }
    HALT("error in examples_per_outcome calculation");
  }
  // check [END]
  
  
  if (DEBUG>0) cout << "induceOutcomes [END]" << endl;
}








// Return value needs to be MINIMIZEd.
double SearchOperator_ground::learnParameters_constrainedCostfunction(const MT::Array< LitL >& outcomes, doubleA& probs) { 
  uint DEBUG = 0;
  
  if (DEBUG > 0) {
    cout << endl;
    cout << "Learning Parameters [START]" << endl;
    PRINT(pen_sum);
    PRINT(pen_pos);
    PRINT(cf_coverage_outcome_example_ground);
  }
    
  uint i, j;
  
  // ATTENTION WE ASSUME COVERAGE HAS BEEN SET CORRECTLY BEFORE
  FOR1D(outcomes, i) {
    if (i == outcomes.N-1)  // omit noise outcome
      continue;
    CHECK(sum(cf_coverage_outcome_example_ground.sub(i,i,0,cf_coverage_outcome_example_ground.d1-1)), "At least one example should be covered for outcome i="<<i<<"!");
  }
  
  // ----------------------------
  // INITIALIZATION
  double cost, oldCost, diff_cost=10.0;
  // init probs
  probs.resize(outcomes.N);
  // default prob = 1 / 2N
  if (outcomes.N > 1)
    probs.last() = 1. / (5. * probs.N);
  else {
    probs.last() = 1.0;
    cost = CostFunction_ground::calc(probs);
    return cost;
  }
  // non-default probs = 1/(N-1) (1 - default_prob)
  for (i=0; i<outcomes.N-1; i++) {
    probs(i) = (1. / (probs.N-1.)) * (1. - probs.last());
  }
  CHECK(TL::isZero(sum(probs)-1), "Stupid probs init, Alter!")
  
  if (DEBUG > 0) {
    PRINT(cf_coverage_outcome_example_ground) // cf_coverage_outcome_example_ground == coverage
  }
  
  
  
  // ----------------------------
  // OPTIMIZATION
  
  // einen von marcs algorithmen anwerfen
  double (*f)(const arr&);
  f = CostFunction_ground::calc;
  void (*df)(arr&,const arr&);
  df = CostFunction_ground::calc_grad;
  
//  MT::checkGradient(f, df, probs, 0.05);
  
  arr gradients;
  cost = CostFunction_ground::calc(probs);
  
  if (DEBUG > 0)
  cout << "init_probs = " << probs << " C=" << cost << endl;

  double STEP_SIZE = 0.01;
  double STOPPING_THRESHOLD = 0.001;
  uint MAX_STEPS = 1000;
  if (param_opt_type == PARAM_OPT__GRAD_DESC) {
    j=0;
    if (DEBUG>0) cout<<"Gradient Descent:"<<endl;
    while(fabs(diff_cost) > STOPPING_THRESHOLD) {
      CostFunction_ground::calc_grad(gradients, probs);
      FOR1D(probs, i) {
        probs(i) -= STEP_SIZE * TL::signOf(gradients(i));
      }
      oldCost = cost;
      cost = CostFunction_ground::calc(probs);
      diff_cost = cost - oldCost;
      if (DEBUG > 0) {
        PRINT(gradients)
        cout << probs << " C=" << cost << " diff=" << diff_cost << endl;
      }
      if (j % 100 == 0) {
        cout << "."; cout << std::flush;
      }
      j++;
    }
    cout << j << endl;
  }
  else if (param_opt_type == PARAM_OPT__NEWTON) {
    NIY
  }
  else if (param_opt_type == PARAM_OPT__RPROP) {
    Rprop rp;
    rp.init(0.025);
    rp.dMin = 1e-9;
    i=0;
    if (DEBUG>0) cout<<"RProp:"<<endl;
    while(fabs(diff_cost) > STOPPING_THRESHOLD) {
      CostFunction_ground::calc_grad(gradients, probs);
      rp.step(probs, gradients);
      oldCost = cost;
      cost = CostFunction_ground::calc(probs);
      diff_cost = cost - oldCost;
      if (DEBUG > 0) {
        if (DEBUG>1)
          cout << i << ": " << probs << " C=" << cost << " diff=" << diff_cost << " sum=" << sum(probs) << endl;
        else {
          if (i>1000) {
            cout << i << ": " << probs << " C=" << cost << " diff=" << diff_cost << " sum=" << sum(probs) << endl;
          }
          else {
            if (i%10==0)
              cout << i << ": " << probs << " C=" << cost << " diff=" << diff_cost << " sum=" << sum(probs) << endl;
          }
        }
      }
      i++;
      if (i>MAX_STEPS) {
        std::cerr << "#params = " << probs.N << endl;
        HALT("Cannot learn the parameters! (No convergence.)")
      }
    }
  }
  else if (param_opt_type == PARAM_OPT__CONJGRAD) {
    NIY
  }
  else if (param_opt_type == PARAM_OPT__COND_GRAD) {
    NIY
    i=0;
    uint maxIndex;
    if (DEBUG>0) cout<<"Gradient Descent:"<<endl;
    while(fabs(diff_cost) > STOPPING_THRESHOLD) {
      CostFunction_ground::calc_grad(gradients, probs);
      maxIndex = 0;
      FOR1D(probs, j) {
        if (fabs(gradients(j)) > fabs(gradients(maxIndex))) {
          maxIndex = j;
        }
      }
      // find optimal step-size via line-search
      probs(maxIndex) -= STEP_SIZE * TL::signOf(gradients(maxIndex));

      oldCost = cost;
      cost = CostFunction_ground::calc(probs);
      diff_cost = cost - oldCost;
      
      if (DEBUG > 0) {
        PRINT(gradients)
          cout << probs << " C=" << cost << " diff=" << diff_cost << endl;
      }
      i++;
    }
  }
  else
    HALT("Unknown optimization strategy")
  
  
  // ----------------------------
  // POST-PROCESSING
  
  // In the end ensure by hand that pi >=0
  double EPSILON__PROB_NEG1 = 0.10;  // 0.01
  double EPSILON__PROB_NEG2 = 0.25;
  double negativstProb = 1.0;
  FOR1D(probs, i) {
    if (probs(i) < -EPSILON__PROB_NEG2) {
      MT::String warning;
      warning << "Param. optimization failed - Significant negative probability: " << probs(i);
      HALT(warning)
    }
    if (probs(i) < -EPSILON__PROB_NEG1) {
      MT::String warning;
      warning << "Param. optimization awkward - Significant negative probability: " << probs(i);
      MT_MSG(warning);
      pen_sum *= 1.3;
    }
    if (probs(i) < 0  &&  probs(i) < negativstProb)
      negativstProb = probs(i);
  }
  if (negativstProb < 0) {
    FOR1D(probs, i) {
      probs(i) -= negativstProb;
    }
  }
  // In the end, ensure by hand that SUM_i pi = 1
  double probSum = sum(probs);
  probs /= probSum;
  double EPSILON__PROB_SUM1 = 0.15;
  double EPSILON__PROB_SUM2 = 0.5;
  if (fabs(probSum - 1) > EPSILON__PROB_SUM2) {
    MT::String warning;
    warning << "Param. optimization failed - sum clearly different from 1: " << probSum << " found=" << (probs*probSum) << " rescaled=" << probs;
    HALT(warning);
  }
  if (fabs(probSum - 1) > EPSILON__PROB_SUM1) {
    MT::String warning;
    warning << "Param. optimization awkward - sum clearly different from 1: " << probSum << " found=" << (probs*probSum) << " rescaled=" << probs;
    MT_MSG(warning);
  }

  if (DEBUG > 0) {
    cout << "Learned params: " << probs << endl;
    cout << "Learning Parameters [END]" << endl;
  }

  // determine final score
  return cost;
}




double SearchOperator_ground::learnParameters(const MT::Array< LitL >& outcomes, doubleA& probs) {
	return learnParameters_constrainedCostfunction(outcomes, probs);
// 	Other Ideas:
// 	(a) nur in die Richtung des groessten Gradienten gehen; dort bestimmte Schritteweise; Rest anpassen
// 	(b) ohne Constraints berechnen; nach jedem Schritt Werte auf Verteilung rueckrechnen
}



const char* SearchOperator_ground::getName() {
	return name;
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    E X P L A I N   E X A M P L E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------



// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void ExplainExperiences_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& experiences, RuleSetContainer_ground& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"ExplainExperiences_ground::findRules [START]"<<endl;
  uint i;
//  PRINT(Rule::globalRuleCounter)
  for (i=nextPotentialExperience; i<experiences.N; i++) {
    uintA& covering_rules = rulesC_old.nonDefaultRules_per_experience(i);
    if (DEBUG>1) {
      write(rulesC_old.rules);
      cout<<"#Covering non-default rules for example (" << i << ") = " << covering_rules.N << endl;
    }
    if (covering_rules.N == 0) {
      // Create new rule by explaining current example (sets context and action)
      if (DEBUG>0) cout << "findRules: let's explain example " << i << endl;
      if (DEBUG>2) {experiences(i)->write(cout);}
      Rule* newRule = explainExperience_ground(experiences(i));
      if (DEBUG>0) {cout<<"New Rule:"<<endl; newRule->write(cout);}
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule, experiences);
      // Estimate new outcomes for r'
      CHECK(covered_experiences.N>0, "At least the explained example should be covered.")
      if (DEBUG>0) {cout<<"#Covering examples: "<<covered_experiences_ids<<"  "<<covered_experiences.N<<endl;}
      MT::Array< uintA > examples_per_outcome;
      induceOutcomes(newRule, examples_per_outcome, covered_experiences, covered_experiences_ids);
      rulesC_2add.append(newRule, covered_experiences_ids, examples_per_outcome);
//       rulesC_2add.sanityCheck();
      nextPotentialExperience = i+1;
      break;
    }
  }
//  PRINT(rules2_add.num());
//  PRINT(Rule::globalRuleCounter)
	if (DEBUG>0) cout<<"ExplainExperiences_ground::findRules [END]"<<endl;
}


Rule* ExplainExperiences_ground::explainExperience_ground(SymbolicExperience* ex) {
	return explainExperience_deictic_ground(ex);
// 	return explainExperience_straightforward(ex);
}


void trim_hack_ground(LitL& lits) {
  uint DEBUG=0;
  // TODO YOU MAY WANT TO FILTER SOME PREDICATE INSTANCES WHICH ARE CLEARLY NEVER TRUE.
  //  Brings in some superficial domain knowledge, but makes learning much faster.
  uint k;
  FOR1D(lits, k) {
    if (lits(k)->atom->pred->name == "table")
      break;
  }
  if (lits.N == k)
    return;
//   MT_MSG("IS TRIM_HACK UP TO DATE FOR THE PREDICATE IDs???");
  if (DEBUG>0) {cout << "trim_hack_ground before: [N="<<lits.N<<"] "; write(lits); cout<<endl;}
  // only apply for primitive predicates
  uint __TABLE_PRED_ID = logicObjectManager::getPredicate(MT::String("table"))->id;
  uint __BLOCK_PRED_ID = logicObjectManager::getPredicate(MT::String("block"))->id;
  uint __BALL_PRED_ID = logicObjectManager::getPredicate(MT::String("ball"))->id;
  uint __ON_PRED_ID = logicObjectManager::getPredicate(MT::String("on"))->id;
  uint __INHAND_PRED_ID = logicObjectManager::getPredicate(MT::String("inhand"))->id;
  uint __BOX_PRED_ID = 1000000;
  if (logicObjectManager::getPredicate(MT::String("box")) != NULL) {
    __BOX_PRED_ID = logicObjectManager::getPredicate(MT::String("box"))->id;
  }
  if (DEBUG>0) {
      cout<<"Did you adapt the predicate IDs correspondingly to logic_world_interface.h???"<<endl;
      PRINT(__ON_PRED_ID);
      PRINT(__TABLE_PRED_ID);
      PRINT(__INHAND_PRED_ID);
//         PRINT(__INHANDNIL_ID)
  }
  uint i;
  lits.memMove = true;
  
  // no self-referencing binary predicates
  // on(X,X) etc.
  FOR1D_DOWN(lits, i) {
      if (lits(i)->atom->args.N == 2) {
          if (lits(i)->atom->args(0) == lits(i)->atom->args(1)) {
              if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
              lits.remove(i);
          }
      }
  }
  if (DEBUG>1) {cout << "trim_hack_ground now: "; write(lits); cout<<endl;}
  
  // for X>__TABLE_OBJECT_ID: not table(X) [--> cares only about constants]
  FOR1D_DOWN(lits, i) {
    if (lits(i)->atom->pred->id == __TABLE_PRED_ID // table
        ||  lits(i)->atom->pred->id == __BLOCK_PRED_ID
        ||  lits(i)->atom->pred->id == __BOX_PRED_ID
        ||  lits(i)->atom->pred->id == __BALL_PRED_ID) {
      if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
      lits.remove(i);
    }
  }
  if (DEBUG>1) {cout << "trim_hack_ground now: "; write(lits); cout<<endl;}
  
  if (DEBUG>1) {cout << "trim_hack_ground now: "; write(lits); cout<<endl;}
  
  // if on(X,Y) then -on(Y,X) is redundant
  FOR1D_DOWN(lits, i) {
      if (!lits(i)->positive) {
        if (lits(i)->atom->pred->id == __ON_PRED_ID // on
              && lits(i)->atom->pred->type == 0) { // primitive
              uint k;
              bool remove = false;
              FOR1D_DOWN(lits, k) {
                  if (lits(k)->positive) {
                    if (lits(k)->atom->pred->id == __ON_PRED_ID // on
                          && lits(k)->atom->pred->type == 0) { // primitive
                          if (lits(i)->atom->args(0) == lits(k)->atom->args(1)
                              &&     lits(i)->atom->args(1) == lits(k)->atom->args(0)) {
                              remove = true;
                              break;
                          }
                      }
                  }
              }
              if (remove) {
                  if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
                  lits.remove(i);
              }
          }
      }
  }
  if (DEBUG>1) {cout << "trim_hack_ground now: "; write(lits); cout<<endl;}
  
  if (DEBUG>0) {cout << "trim_hack_ground after: [N="<<lits.N<<"] "; write(lits); cout<<endl;}
}


/*
	First, derive all complex predicates in pre-state.
	Second, calculate which predicates have become true in the post-state that didn't hold in the pre-state. Add their negations to the pre-state.
	Third, derive substitution s for (i) constants in action, and (ii) all other constants whose properties have been changed.
	Apply s to pre-state to derive partially abstracted pre-state pre_a. Keep only those predicates in pre_a which are fully abstract now. These form the context of the new rule.
	The outcomes will only be calculated in "induce_outcomes".
*/
// Rule* ExplainExperiences_ground::explainExperience_straightforward(SymbolicExperience* ex) {
// 	uint DEBUG = 0;
// 	if (DEBUG>0) cout << "explainExperience_straightforward [START]" << endl;
// 	Rule* newRule = new Rule;
// 	
// 	if (DEBUG>0) {
// 		cout << "Explaining example:" << endl;
// 		ex->pre->write(cout); cout << endl;
// 		ex->action->write(cout); cout << endl;
// 		ex->post->write(cout); cout << endl;
// 	}
// 	
// 	// Substitution
// 	Substitution invSub;
//     le->createInverseSubstitution(*(ex->action), invSub);
// 	LitL changedProperties_onlyPre_grounded;
// 	LitL changedProperties_onlyPost_grounded;
// 	uintA changedConstants;
// 	le->changes(ex->pre, ex->post, changedConstants, changedProperties_onlyPre_grounded, changedProperties_onlyPost_grounded);
// 	uint i;
// 	FOR1D(changedConstants, i) {
// 		if (!invSub.hasSubs(changedConstants(i)))
// 			invSub.addSubs2Variable(changedConstants(i));
// 	}
// 	
// 	if (DEBUG > 2) {
// 		cout << "changedProperties_onlyPre_grounded: "; write(changedProperties_onlyPre_grounded); cout << endl;
// 		cout << "changedProperties_onlyPost_grounded: "; write(changedProperties_onlyPost_grounded); cout << endl;
// 		PRINT(changedConstants)
// 		cout<<"Established inverse substitution "; invSub.write(cout); cout << endl;
// 	}
// 	
// 	// Set action
// 	newRule->action = le->applyOriginalSub(invSub, ex->action);
// 	if (DEBUG>0) {
// 		cout<< "New action: "; newRule->action->write(cout); cout << endl;
// 		
// 	}
//     
// 	// Set context
// 	LitL context_grounded;
// 	logicReasoning::derive(ex->pre);
// 	context_grounded.append(ex->pre->primitivePredTs);
// 	FOR1D(ex->pre->derivedPredTs, i)
//             context_grounded.append(ex->pre->derivedPredTs(i));
// 	
// 	if (DEBUG>1) {
// 		cout << "context_grounded: ";
// 		write(context_grounded);
// 		cout << endl;
// 	}
// 	
// 	// use also negated
// 	LitL changedProperties_onlyPost_grounded_neg;
// 	le->negate(changedProperties_onlyPost_grounded, changedProperties_onlyPost_grounded_neg);
// 	context_grounded.append(changedProperties_onlyPost_grounded_neg);
// 
// 	LitL context_abstracted;
// 	le->applyOriginalSub(invSub, context_grounded, context_abstracted);
// 	if (DEBUG>1) {
// 		cout << "context_abstracted: ";
// 		write(context_abstracted);
// 		cout << endl;
// 	}
// 	
// 	// only abstract predicates; this filters implicitely the predicates to only contain
// 	// the action constants and deictic variable constants
// 	LitL context_abstracted_pure;
// 	le->filterPurelyAbstract(context_abstracted, context_abstracted_pure);
// 	newRule->context = context_abstracted_pure;
// 	
// 	
// 	if (DEBUG>0) {
// 		cout << "newRule->context: ";
// 		write(newRule->context);
// 		cout << endl;
// 	}
// 	
// 	// trim literals
//     trim_hack_ground(newRule->context);
//     if (slimPreconditions)
//         logicReasoning::killBaseConcepts(newRule->context);
//     if (DEBUG>0) {
//       cout << "newRule->context after trimming: ";
//       write(newRule->context);
//       cout << endl;
//     }
//     
// //     std::cerr << endl;
// //     std::cerr << endl;
// //     std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
// //     std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
// //     std::cerr << endl;
// //     std::cerr << endl;
// 	
// 	
// 	// Outcome will be set somewhere else
// 	
// 	if (DEBUG>0) cout << "explainExperience_straightforward [END]" << endl;
// 	return newRule;
// }


// Algorithm Pasula et al. (2007) p. 330
Rule* ExplainExperiences_ground::explainExperience_deictic_ground(SymbolicExperience* ex) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "explainExperience_deictic [START]" << endl;
  if (DEBUG>1) ex->write(cout);
  uint i, j;
  Rule* newRule = new Rule;
  
  // ensure that all complex are derived
  logicReasoning::derive(&ex->pre);
  logicReasoning::derive(&ex->post);
  LitL precond_candidates;
  
  // Step 1.1: Create an action and context
  // create action
  newRule->action = ex->action;
  if (DEBUG>1) {cout<<"New action: ";newRule->action->write(cout);cout<<endl;}
  // create context
  // create normal literals
  // (also accounts for negations)
  logicObjectManager::getLiterals(precond_candidates, newRule->action->args);
  if (DEBUG>2) {cout<<"Precondition candidates (based on action arguments, w./o. comparisons): ";write(precond_candidates);cout<<endl;}
  FOR1D(precond_candidates, i) {
    if (logicReasoning::holds(ex->pre, precond_candidates(i)))
      newRule->context.append(precond_candidates(i));
  }
  // create comparison literals
  LitL equalityLiterals;
  logicObjectManager::getCompLiterals_constantBound(equalityLiterals, newRule->action->args, ex->pre, 0);
  FOR1D(equalityLiterals, i) {
//     cout<<"equalityLiterals(i):  ";  equalityLiterals(i)->write(cout);  cout<<endl;
    newRule->context.append(equalityLiterals(i));
  }
  // order by positives first
  logicReasoning::sort(newRule->context);
  if (DEBUG>1) {cout<<"Context (preliminary): ";write(newRule->context);cout<<endl;}
  
  // Step 1.2: Create deictic references
  if (DEBUG > 2) {
    cout << "ex->del: "; write(ex->del); cout << endl;
    cout << "ex->add: "; write(ex->add); cout << endl;
    PRINT(ex->changedConstants)
  }
  uintA used_constants;
  used_constants.setAppend(newRule->action->args);
  LitL newPreconditions;
  FOR1D(ex->changedConstants, i) {
    if (used_constants.findValue(ex->changedConstants(i)) >= 0)
      continue;
    used_constants.setAppend(ex->changedConstants(i));
    // create normal predicates (also accounts for negations)
    uintA changedConstantWrapper;
    changedConstantWrapper.append(ex->changedConstants(i));
    logicObjectManager::getLiterals(precond_candidates, used_constants, changedConstantWrapper);
    // create constant-bound comparison literals
    logicObjectManager::getCompLiterals_constantBound(equalityLiterals, changedConstantWrapper, ex->pre, 0);
    precond_candidates.append(equalityLiterals);
    if (DEBUG>1) {cout<<"Deictic candidate: "<<ex->changedConstants(i)<<endl;}
    if (DEBUG>2) {cout<<"Precondition candidates (based on deictic candidate): ";write(precond_candidates);cout<<endl;}
    // create possible newPreconditions
    newPreconditions = newRule->context;
    FOR1D(precond_candidates, j) {
      if (logicReasoning::holds(ex->pre, precond_candidates(j))) {
        newPreconditions.append(precond_candidates(j));
      }
    }
    logicReasoning::sort(newPreconditions);
    if (DEBUG>1) {cout<<"Context with deic ref (potential): ";write(newPreconditions);cout<<endl;}
    newRule->context = newPreconditions;
  }
  
  // trim literals
  trim_hack_ground(newRule->context);
  if (slimPreconditions)
    logicReasoning::killBaseConcepts(newRule->context);
  if (DEBUG>0) {
    cout << "newRule->context after trimming: ";
    write(newRule->context);
    cout << endl;
  }
  
  if (DEBUG>0) {
    newRule->write(cout);
  }
  if (DEBUG>0) cout << "explainExperience_deictic [END]" << endl;
  
  return newRule;
}



void ExplainExperiences_ground::reset() {
	nextPotentialExperience = 0;
}







// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    D R O P   C O N D I T I O N S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropPreconditions_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropPre::findRules [START]"<<endl;
  uint r, p, i;
  Rule* newRule;
  bool stop = false;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextPrecondition; p<rulesC_old.rules.elem(r)->context.N; p++) {
      newRule = new Rule;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
        if (i!=p)
          newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      newRule->action = rulesC_old.rules.elem(r)->action;
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule, examples);
      if (covered_experiences.N > 0) {
        if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" examples and will be kept."<<endl;
        if (DEBUG>3) {
          cout<<"Covered examples:"<<endl;
          uint k;
          FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
          }
        }
        MT::Array< uintA > examples_per_outcome;
        induceOutcomes(newRule, examples_per_outcome, covered_experiences, covered_experiences_ids);
        rulesC_2add.append(newRule, covered_experiences_ids, examples_per_outcome);
        stop = true;
        nextPrecondition += 1;
        break;
      }
      else {
        delete newRule;
      }
    }
    if (stop)
      break;
    
    nextRule=r+1;
    nextPrecondition=0;
  }
  if (DEBUG>0) cout<<"DropPre::findRules [END]"<<endl;
}


void DropPreconditions_ground::reset() {
	nextRule = 0;
	nextPrecondition = 0;
}






// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    D R O P   C O N D I T I O N S approximative Version
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropPreconditions_approximativeVersion_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropPre_approx::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint i, k;
  if (prepareTotalNewSearch) {
    usablePreconds.clear();
    uint num_preconds=0;
    FOR1D_(rulesC_old.rules, i) {
      num_preconds += rulesC_old.rules.elem(i)->context.N;
    }
    usablePreconds.resize(num_preconds);
    usablePreconds.setUni(1);
    prepareTotalNewSearch=false;
    if (DEBUG>0) cout<<"Prepared new usablePreconds-Array with N="<<usablePreconds.N<<endl;
  }
  uint random_precondition, id_precondition, id_rule, starting_id_for_current_rule;
  uint num_open_context;
  Rule* newRule;
  uint resamples=0;
  while (rulesC_2add.rules.num()==0) {
    num_open_context = sum(usablePreconds);
    if (num_open_context==0)
        break;
    // randomly choose which precondition to delete
    random_precondition = rnd.num(num_open_context);
    if (DEBUG>1) {cout<<"New rule-finding try:"<<endl; PRINT(num_open_context); PRINT(random_precondition); PRINT(usablePreconds);}
    id_precondition=0;
    id_rule=0;
    starting_id_for_current_rule=0;
    // find the correct rule and precondition for the random_precondition
    FOR1D(usablePreconds, k) {
      // If target surely not in current rule "rulesC_old.rules.elem(id_rule)"...
      // Counter for id_rule
      if (k >= rulesC_old.rules.elem(id_rule)->context.N+starting_id_for_current_rule) {
        id_rule++;
        while (rulesC_old.rules.elem(id_rule)->context.N == 0)  // account for empty rules thereafter
          id_rule++;
        starting_id_for_current_rule=k;
      }
      // Counter for id_precondition
      //    found
      if (usablePreconds(k) && id_precondition==random_precondition) {
        usablePreconds(k)=0;
        id_precondition=k-starting_id_for_current_rule;
        break;
      }
      //   searching
      if (usablePreconds(k))
        id_precondition++;
    }
    if (DEBUG>1) {
      cout<<"Thinking about deleting in rule "<<id_rule<<" precondition "<<id_precondition<<endl;
      rulesC_old.rules.elem(id_rule)->write(cout);
      cout<<"Precondition of interest:  "; rulesC_old.rules.elem(id_rule)->context(id_precondition)->write(cout); cout<<endl;
    }
    
    // bias for keeping positive
    if (rulesC_old.rules.elem(id_rule)->context(id_precondition)->positive && resamples < DROP_NEGATIVE_BIAS) {
      if (DEBUG>1) {cout<<" (Don't wanna delete positive pt: "; rulesC_old.rules.elem(id_rule)->context(id_precondition)->write(cout); cout<<")"<<endl;}
      usablePreconds(k)=1; // set back
      resamples++;
      continue;
    }
    else
      resamples=0;
    if (DEBUG>1) {cout<<"Deletion executed."<<endl;}
    newRule = new Rule;
    FOR1D(rulesC_old.rules.elem(id_rule)->context, k) {
      if (k!=id_precondition)
        newRule->context.append(rulesC_old.rules.elem(id_rule)->context(k));
    }
    newRule->action = rulesC_old.rules.elem(id_rule)->action;
    SymbolicExperienceL covered_experiences;
    uintA covered_experiences_ids;
    calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule, examples);
    if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
    if (DEBUG>1) cout<<"+++++ Covers "<<covered_experiences.N<<" examples " << covered_experiences_ids << endl;
    if (covered_experiences.N > 0) {
      if (DEBUG>1) cout<<"Thus, new rule will be kept."<<endl;
      if (DEBUG>3) {
        cout<<"Covered examples:"<<endl;
        uint k;
        FOR1D(covered_experiences, k) {
          covered_experiences(k)->write(cout);
        }
      }
      MT::Array< uintA > examples_per_outcome;
      induceOutcomes(newRule, examples_per_outcome, covered_experiences, covered_experiences_ids);
      rulesC_2add.append(newRule, covered_experiences_ids, examples_per_outcome);
    }
    else {
      delete newRule;
    }
  }
  
  if (DEBUG>1) cout<<"# rules found = "<<rulesC_2add.rules.num()<<endl;
  if (DEBUG>0) cout<<"DropPre_approx::findRules [END]"<<endl;
}


void DropPreconditions_approximativeVersion_ground::reset() {
}


void DropPreconditions_approximativeVersion_ground::reset_total_approximator() {
  prepareTotalNewSearch = true;
}






// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    D R O P   R E F E R E N C E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropReferences_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {
	uint DEBUG = 0;
	if (DEBUG>0) cout<<"DropReferences_ground::findRules [START]"<<endl;
	uint r, p, i;
  bool stop = false;
	Rule* newRule = NULL;
	for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    uintA terms;
    ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), terms);
    uintA non_arg_terms;
    FOR1D(terms, i) {
      if (rulesC_old.rules.elem(r)->action->args.findValue(terms(i)) < 0)
        non_arg_terms.setAppend(terms(i));
    }
    for (p=nextReference; p<terms.N; p++) {
      if (DEBUG>0) {
        cout<<"Removing non-arg reference "<<terms(nextReference)<<" in rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
      }
      newRule = new Rule;
      newRule->action = rulesC_old.rules.elem(r)->action;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
          if (DEBUG>4) {PRINT(rulesC_old.rules.elem(r)->context(i)->atom->args);}
          if (rulesC_old.rules.elem(r)->context(i)->atom->args.findValue(terms(nextReference))<0)
              newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      if (DEBUG>0) {cout<<"Yielding the new context: "; write(newRule->context); cout<<endl;}
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule, examples);
      if (covered_experiences.N > 0) {
        if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" examples and will be kept."<<endl;
        if (DEBUG>3) {
          cout<<"Covered examples:"<<endl;
          uint k;
          FOR1D(covered_experiences, k) {
            covered_experiences(k)->write(cout);
          }
        }
        MT::Array< uintA > examples_per_outcome;
        induceOutcomes(newRule, examples_per_outcome, covered_experiences, covered_experiences_ids);
        rulesC_2add.append(newRule, covered_experiences_ids, examples_per_outcome);
        stop = true;
        nextReference += 1;
        break;
      }
      else {
        delete newRule;
      }
    }
    if (stop)
      break;

    nextRule=r+1;
    nextReference=0;
	}
	if (DEBUG>0) {if (newRule!=NULL) {cout<<"Found rule: "; newRule->write(cout); cout<<endl;}}
	
	if (DEBUG>0) cout<<"DropReferences_ground::findRules [END]"<<endl;
}


void DropReferences_ground::reset() {
	nextRule=0;
	nextReference = 0;
}









// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    D R O P   R U L E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void DropRules_ground::createRuleSets(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, 
          MT::Array< RuleSetContainer_ground >& one_rulesC_new) {
  uint DEBUG = 0;
  one_rulesC_new.clear();
  uint i, j;
  if (DEBUG>0) {cout<<"Old rule-set:"<<endl;  rulesC_old.writeNice(cout, true); cout<<endl<<endl;}
  for (i=1; i<rulesC_old.rules.num(); i++) { // default rule must always be in
    if (DEBUG>0) {cout<<"Dropping rule #" << i << endl;}
    RuleSetContainer_ground rulesC_new(&examples);
//     rulesC_new = rulesC_old;
    FOR1D_(rulesC_old.rules, j) {
      if (i!=j) {
        if (DEBUG>0) {cout<<"j="<<j<<":  "; PRINT(rulesC_old.experiences_per_rule(j));}
        rulesC_new.append(rulesC_old.rules.elem(j), rulesC_old.experiences_per_rule(j), rulesC_old.experiences_per_ruleOutcome(j));
      }
    }
    rulesC_new.recomputeDefaultRule();
//     rulesC_new.sanityCheck();
    one_rulesC_new.append(rulesC_new);
  }
}

void DropRules_ground::reset() {}
void DropRules_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {}











// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    S P L I T   O N   L I T E R A L S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void SplitOnLiterals_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnLiterals_ground::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint r;
  Rule* newRule_pos = NULL;
  Rule* newRule_neg = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      ruleReasoning::calcAbsentLiterals(*rulesC_old.rules.elem(r), absentLiterals, true);
      if (DEBUG>2) {
        cout << "Calculated absent literals for rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Absent literals: ";write(absentLiterals);cout<<endl;
      }
    }
    while (nextLiteral<absentLiterals.N) {
      // positive version
      LitL wrapper;
      wrapper.append(absentLiterals(nextLiteral));
      if (DEBUG>1) {
        cout<<"Trying to insert ";absentLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && !logicReasoning::containsLiteral(rulesC_old.rules.elem(r)->context, *absentLiterals(nextLiteral))) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_pos = new Rule;
        newRule_pos->action = rulesC_old.rules.elem(r)->action;
        newRule_pos->context = rulesC_old.rules.elem(r)->context;
        ruleReasoning::insert(*newRule_pos, *absentLiterals(nextLiteral));
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule_pos, examples);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" examples and will be kept."<<endl;
          MT::Array< uintA > examples_per_outcome;
          induceOutcomes(newRule_pos, examples_per_outcome, covered_experiences, covered_experiences_ids);
          rulesC_2add.append(newRule_pos, covered_experiences_ids, examples_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 examples and will be dropped."<<endl;
          delete newRule_pos;
        }
      }
      else {
        if (DEBUG>1) cout<<" --> Impossible."<<endl;
      }
      // negative version
      Literal* nextLiteral_neg = logicObjectManager::getLiteralNeg(absentLiterals(nextLiteral));
      wrapper.clear();
      wrapper.append(nextLiteral_neg);
      if (DEBUG>1) {
        cout<<"Trying to insert ";nextLiteral_neg->write(cout);cout<<"   into   ";
        write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && !logicReasoning::containsLiteral(rulesC_old.rules.elem(r)->context, *nextLiteral_neg)) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_neg = new Rule;
        newRule_neg->action = rulesC_old.rules.elem(r)->action;
        newRule_neg->context = rulesC_old.rules.elem(r)->context;
        ruleReasoning::insert(*newRule_neg, *nextLiteral_neg);
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule_neg, examples);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) {cout<<"Covers "<<covered_experiences.N<<" examples and will be kept."<<endl;}
          if (DEBUG>3) {
            cout<<"Covered examples:"<<endl;
            uint k;
            FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
            }
          }
          MT::Array< uintA > examples_per_outcome;
          induceOutcomes(newRule_neg, examples_per_outcome, covered_experiences, covered_experiences_ids);
          rulesC_2add.append(newRule_neg, covered_experiences_ids, examples_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 examples and will be dropped."<<endl;
          delete newRule_neg;
        }
      }
      else {
        if (DEBUG>1) cout<<" --> Impossible."<<endl;
      }
      
      nextLiteral++;
      if (rulesC_2add.rules.num() > 0)
        break;
    }
    if (rulesC_2add.rules.num() > 0)
      break;
    else {
      nextRule = r+1;
      nextLiteral = 0;
      absentLiterals.clear();
    }
  }
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {write(rulesC_2add.rules);}}
  if (DEBUG>0) cout<<"SplitOnLiterals_ground::findRules [END]"<<endl;
}

void SplitOnLiterals_ground::reset() {
  nextRule = 1;
  nextLiteral = 0;
}








// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    A D D   L I T E R A L S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void AddLiterals_ground::findRules(const RuleSetContainer_ground& rulesC_old, const SymbolicExperienceL& examples, RuleSetContainer_ground& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"AddLits::findRules [START]"<<endl;
  uint r;
  Rule* newRule = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      ruleReasoning::calcAbsentLiterals(*rulesC_old.rules.elem(r), absentLiterals);
      if (DEBUG>2) {
        cout << "Calculated absent literals for rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Absent literals: ";write(absentLiterals);cout<<endl;
      }
    }
    while (nextLiteral<absentLiterals.N) {
      LitL wrapper;
      wrapper.append(absentLiterals(nextLiteral));
      if (DEBUG>1) {
        cout<<"Trying to insert ";absentLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule = new Rule;
        newRule->action = rulesC_old.rules.elem(r)->action;
        newRule->context = rulesC_old.rules.elem(r)->context;
        ruleReasoning::insert(*newRule, *absentLiterals(nextLiteral));
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage_ground(covered_experiences, covered_experiences_ids, newRule, examples);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" examples and will be kept."<<endl;
          if (DEBUG>3) {
            cout<<"Covered examples:"<<endl;
            uint k;
            FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
            }
          }
          MT::Array< uintA > examples_per_outcome;
          induceOutcomes(newRule, examples_per_outcome, covered_experiences, covered_experiences_ids);
          rulesC_2add.append(newRule, covered_experiences_ids, examples_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 examples and will be dropped."<<endl;
          delete newRule;
        }
      }
      else {
        if (DEBUG>1) cout<<" --> Impossible."<<endl;
      } 
      nextLiteral++;
      if (rulesC_2add.rules.num() > 0)
        break;  
    }
    if (rulesC_2add.rules.num() > 0)
      break;
    else {
      nextRule = r+1;
      nextLiteral = 0;
      absentLiterals.clear();
    }
  }
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {newRule->write(cout);}}
  if (DEBUG>0) cout<<"AddLits::findRules [END]"<<endl;
}


void AddLiterals_ground::reset() {
	nextRule = 1;
	nextLiteral = 0;
}




















// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    R U L E   L E A R N E R
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


RuleLearner_ground::RuleLearner_ground(double alpha, double p_min, double p_min_noisyDefaultRule, uint param_opt_type, uint so_choice_type) {
  this->alpha_PEN = alpha;
  this->p_min = p_min;
  this->p_min_noisyDefaultRule = p_min_noisyDefaultRule;
  this->so_choice_type = so_choice_type;
    
  // SUCH A SEARCHER
  
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES>0.0) {
      ExplainExperiences_ground* explainEx = new ExplainExperiences_ground(false, false, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES);
  }
  
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM>0.0) {
      ExplainExperiences_ground* explainEx_slim = new ExplainExperiences_ground(true, false, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx_slim);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM);
  }
  
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM>0.0) {
      ExplainExperiences_ground* explainEx_slim_compare = new ExplainExperiences_ground(true, true, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx_slim_compare);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM_AND_COMPARING);
  }

  if (SO_WEIGHT__DROP_PRECONDS>0.0) {
      DropPreconditions_ground* dropPre = new DropPreconditions_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropPre);
      so_priorWeights.append(SO_WEIGHT__DROP_PRECONDS);
  }
  
  if (SO_WEIGHT__DROP_PRECONDS_APPROX>0.0) {
      DropPreconditions_approximativeVersion_ground* dropPre_approx = new DropPreconditions_approximativeVersion_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropPre_approx);
      so_priorWeights.append(SO_WEIGHT__DROP_PRECONDS_APPROX);
  }

  if (SO_WEIGHT__DROP_REFS>0.0) {
      DropReferences_ground* dropRef = new DropReferences_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropRef);
      so_priorWeights.append(SO_WEIGHT__DROP_REFS);
  }
  
  if (SO_WEIGHT__DROP_RULES>0.0) {
      DropRules_ground* dropRules = new DropRules_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropRules);
      so_priorWeights.append(SO_WEIGHT__DROP_RULES);
  }

  if (SO_WEIGHT__SPLIT_ON_LITS>0.0) {
      SplitOnLiterals_ground* splitOnLits = new SplitOnLiterals_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(splitOnLits);
      so_priorWeights.append(SO_WEIGHT__SPLIT_ON_LITS);
  }

  if (SO_WEIGHT__ADD_LITS>0.0) {
      AddLiterals_ground* addLit = new AddLiterals_ground(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(addLit);
      so_priorWeights.append(SO_WEIGHT__ADD_LITS);
  }

  num_so_applied.resize(searchOperators.N);
  num_so_applied.setUni(0);
  num_so_improvements.resize(searchOperators.N);
  num_so_improvements.setUni(0);
  so_improvements.resize(searchOperators.N);
  so_improvements.setUni(0.0);
}




RuleLearner_ground::~RuleLearner_ground() {
  uint i;
  FOR1D(searchOperators, i) {
    delete searchOperators(i);
  }
}



double RuleLearner_ground::score(RuleSetContainer_ground& rulesC, SymbolicExperienceL& experiences, double cutting_threshold) {
  arr experience_weights(experiences.N);
  experience_weights.setUni(1.0);
  return score(rulesC, experiences, cutting_threshold, experience_weights);
}

double RuleLearner_ground::score(RuleSetContainer_ground& rulesC, SymbolicExperienceL& experiences, double cutting_threshold, arr& experience_weights) {
  uint DEBUG = 0;
  if (DEBUG > 0) {cout << "SCORE [start]" << endl;}
  if (DEBUG > 1) {cout<<"RULES:"<<endl;  rulesC.writeNice(); }
  uint i;
  
  // (1) Penalty
  double penalty = 0.0;
  FOR1D_(rulesC.rules, i) {
    penalty += ruleReasoning::numLiterals(*rulesC.rules.elem(i));
  }
  penalty *= alpha_PEN;
  if (DEBUG > 0) {PRINT(penalty);}
  
  // (2) Log-Likelihood
  double loglikelihood = 0.0, exLik;
  FOR1D(experiences, i) {
    if (DEBUG > 1) {cout << "+++ ex " << i << ": ";  experiences(i)->action->write(cout);  cout<<endl;}
    const uintA& covering_rules = rulesC.nonDefaultRules_per_experience(i);
    if (DEBUG>1) {PRINT(covering_rules);}
    // only one non-default rule covers
    if (covering_rules.N == 1) {
      Rule* rule = rulesC.rules.elem(covering_rules(0));
      if (DEBUG > 2) {
        cout << "Use rule #" << covering_rules(0) << "  " << rulesC.experiences_per_rule(covering_rules(0)) << endl;
        rule->write(cout);
      }
      #if 0
      exLik = ruleReasoning::probability_abstractRule(rule, *examples(i)->pre, examples(i)->action, *examples(i)->post, p_min);
      #else
      uint o;
      MT::Array< uintA >& exs_per_out = rulesC.experiences_per_ruleOutcome(covering_rules(0));
      if (DEBUG>2) {PRINT(exs_per_out);}
      exLik = 0.0;
      FOR1D(exs_per_out, o) {
        // Non-noise outcome
        if (o<exs_per_out.N-1  &&  exs_per_out(o).findValue(i) >= 0) {
          exLik += rule->probs(o);
        }
        // Noise-outcome:  always covers
        if (o == exs_per_out.N-1) {
          exLik += p_min * rule->probs(o);
        }
      }
      CHECK(!TL::isZero(exLik), "bad referencing  exLik="<<exLik);
      #endif
    }
    // only default covers or more than one non-default covers
    // --> apply default rule
    else {
      if (DEBUG > 2) {
        cout << " Using default rule." << endl;
      }
      Rule* default_rule = rulesC.rules.elem(0);
//       exLik = ruleReasoning::probability_defaultRule(default_rule, *examples(i)->pre, *examples(i)->post, p_min_noisyDefaultRule);
      if (rulesC.experiences_per_ruleOutcome(0)(0).findValue(i) >= 0)
        exLik = default_rule->probs(0) * (100.0 * p_min_noisyDefaultRule);
      else 
        exLik = default_rule->probs(1) * p_min_noisyDefaultRule;
      // TODO wieder wegmachen
//       exLik *= 0.01;// cerr<<"SGE"<<std::flush;
    }
    loglikelihood += experience_weights(i) * log(exLik); // weight the examples differently
    if (DEBUG>1) cout<<" --> lik=" << exLik<<endl;
    
    if (loglikelihood - penalty  <  cutting_threshold) {
      if (DEBUG>0) {
        PRINT(penalty);
        PRINT(loglikelihood);
        PRINT(loglikelihood - penalty);
        PRINT(cutting_threshold);
        cout<<"Score becomes too small... giving up."<<endl;
        cout << "SCORE [end]" << endl;
      }
      return TL::TL_DOUBLE_MIN;
    }
  }
  if (DEBUG > 0) {
    PRINT(loglikelihood);
  }
  
  
  if (DEBUG > 0) {
    PRINT(loglikelihood - penalty);
    cout << "SCORE [end]" << endl;
  }
  
  return loglikelihood - penalty;
}




int RuleLearner_ground::chooseNextOperator(boolA& op_applicable) {
  if (sum(op_applicable) == 0)
    return -1;
  uint op = -1;
  // LINEAR VARIANT
  if (so_choice_type == RULE_LEARNER__OP_CHOICE__LINEAR) {
    static int op_linear = -1;
    op_linear++;
    op_linear %= searchOperators.N;
    op = op_linear;
  }
  // RANDOM VARIANT
  else if (so_choice_type == RULE_LEARNER__OP_CHOICE__RANDOM) {
    arr so_weights;
    so_weights = so_priorWeights;
    uint i;
    FOR1D_DOWN(so_successfulUsageHistory, i) {
      if (so_successfulUsageHistory.d0-SEARCH_OP_CHOICE__PAST_HORIZON>i)
        break;
      so_weights(so_successfulUsageHistory(i)) += SEARCH_OP_CHOICE__PAST_WEIGHT;
    }
    FOR1D(op_applicable, i) {
      if (!op_applicable(i))
        so_weights(i) = 0.0;
    }
    op = TL::basic_sample(so_weights);
  }
  else {
    HALT("Undefined search operator choice procedure:  so_choice_type="<<so_choice_type)
  }
  // FINAL
  return op;
}



void RuleLearner_ground::setAlphaPEN(double alpha_PEN) {
  this->alpha_PEN = alpha_PEN;
}


void RuleLearner_ground::learn_rules(RuleSetContainer_ground& rulesC, SymbolicExperienceL& experiences, const char* logfile) {
  arr experience_weights(experiences.N);
  experience_weights.setUni(1.0);
  learn_rules(rulesC, experiences, experience_weights, logfile);
}

// Algorithm in Zettlemoyer's Figure 2
void RuleLearner_ground::learn_rules(RuleSetContainer_ground& rulesC, SymbolicExperienceL& experiences, arr& experience_weights, const char* logfile) {
  uint DEBUG = 0; //  2 ist gut
  rulesC.clear();
  rulesC.init(&experiences);
  uint i, j;
  // Set penalties for probabilities learning in search operators
//   double pen_pos_scaled = probabilitiesLearning__pen_pos * (examples.N+1);
//   double pen_sum_scaled = probabilitiesLearning__pen_sum * (examples.N+1);
//   FOR1D(searchOperators, i) {
//     searchOperators(i)->setProbabilitiesLearningPenalty_pos(pen_pos_scaled);
//     searchOperators(i)->setProbabilitiesLearningPenalty_sum(pen_sum_scaled);
//   }
  
  // Init default rule
  rulesC.rules.append(ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();
  bool betterRulesFound = true;
  uint round = 0;
  double bestscore = score(rulesC, experiences, TL::TL_DOUBLE_MIN, experience_weights);
  scores.append(bestscore);
  if (DEBUG > 0) {
    cout << "learnRuleSet [START]" << endl;
    cout << "Number of training examples " << experiences.N << endl;
    cout << "Default rule:" << endl;
//     rulesC.rules.elem(0)->write(cout);
    rulesC.writeNice();
    cout << "SCORE = " << bestscore << endl;
  }
  if (DEBUG > 2) {
    cout << "Examples:" << endl;
    FOR1D(experiences, i) {
      cout << "--- [" << i << "] ---" << endl;
      experiences(i)->write();  cout<<endl;
//       cout<<"PRE:  "; examples(i)->pre->write(cout); cout<<endl;
//       cout<<"ACTION:  "; examples(i)->action->write(cout); cout<<endl;
//       cout<<"POST:  "; examples(i)->post->write(cout); cout<<endl;
    }
  }
  // log writing
  std::ofstream log;
  MT::open(log, logfile);
  log<<"# Search Operators:"<<endl;
  FOR1D(searchOperators, i) {
    log<<"# "<<i<<" "<<searchOperators(i)->getName()<<endl;
  }
  log<<"#"<<endl;
  log << "# round  op  bestscore  #newRulesets  improvement"<<endl;
  
  uint MAX_STEPS = 10000;
  boolA op_applicable;
  op_applicable.resize(searchOperators.d0);
  op_applicable.setUni(true);
  bool so_useAgain=false;
  int op = 0;
  while (round++ < MAX_STEPS) {
    if (!so_useAgain) {
      op = chooseNextOperator(op_applicable);
      if (op < 0)
          break;
      if(searchOperators(op)->isApproximator())
          searchOperators(op)->reset_total_approximator();
    }
    
    if (op < 0)
      break;
    if (DEBUG > 0) {cout << "========== LEARN RULE-SET ROUND " << round << " ==========" << endl;}
    betterRulesFound = false;
    if (DEBUG > 1) {cout<<">>> Search operator ***"<<searchOperators(op)->getName()<<"*** gives it a try. <<<"<<endl;}
    if (DEBUG > 1) {if(so_useAgain) cout<<"Using op again."<<endl; else cout<<"Using fresh op."<<endl;}
    so_UsageHistory.append(op);
    num_so_applied(op)++;
    MT::Array< RuleSetContainer_ground > set_of__rulesC_new;
    searchOperators(op)->createRuleSets(rulesC, experiences, set_of__rulesC_new);
    if (set_of__rulesC_new.N == 0) {
      op_applicable(op) = false;
      if (so_useAgain) {
        so_useAgain = false;
        if (DEBUG>1) cout<<"Turning off search operator."<<endl;
      }
      //write log
      log << round << " " << op << " " << bestscore << " 0" << endl;
      if (DEBUG>1) {cout << "No new rules found." << endl;}
      continue;
    }
//     if (DEBUG>1) {cout<<"Sanity checks"<<endl;}
//     FOR1D(set_of__rulesC_new, j) {
//       if (DEBUG>2) {cout<<"Sanity check for new candidate rule-set #"<<j<<endl;}
//       set_of__rulesC_new(j).sanityCheck();
//     }
    arr new_scores;
    FOR1D(set_of__rulesC_new, j) {
      new_scores.append(score(set_of__rulesC_new(j), experiences, bestscore, experience_weights));
    }
    if (DEBUG > 2) {
      cout << "Search operator found the following " << set_of__rulesC_new.N << " new rule-sets:" << endl;
      FOR1D(set_of__rulesC_new, j) {
        cout << "+++ New rule-set " << j << ": +++" << endl;
        set_of__rulesC_new(j).writeNice(cout);
        cout << " --> score=" << new_scores(j);
        if (TL::areEqual(new_scores(j), TL::TL_DOUBLE_MIN))
          cout <<" (calculation had been aborted as it is really bad...)";
        cout << endl;
      }
      cout << "Old best score: " << bestscore << endl;
      cout << "Scores of the new rule-sets: " << new_scores << endl;
    }
    uint maxIdx = new_scores.maxIndex();
    bool betterRulesFound_thisSearchOperator_ground = false;
    // write log
    log << round << " " << op << " " << (new_scores(maxIdx) > bestscore ? new_scores(maxIdx) : bestscore) << " " << new_scores.N << " " << (new_scores(maxIdx) > bestscore ? (new_scores(maxIdx)-bestscore) : 0) << endl;
    // We did improve!
    if (new_scores(maxIdx) > bestscore) {
      // some statistic
      num_so_improvements(op)++;
      so_improvements(op) += new_scores(maxIdx)-bestscore;
      // algorithmic part
      rulesC = set_of__rulesC_new(maxIdx);
      bestscore = new_scores(maxIdx);
      betterRulesFound = true;
      betterRulesFound_thisSearchOperator_ground = true;
      op_applicable.setUni(true); // now, all sos are applicable again since we have a rule-set change
      so_successfulUsageHistory.append(op);
      so_useAgain = false; // here we go, efficiency! we don't need to use this SO again.
    }
    // Found rule-sets but not better ones
    else {
      if (!searchOperators(op)->isApproximator())
        op_applicable(op) = false;
      else
        so_useAgain = true; // might produce nice results again
    }
    if (DEBUG > 0) {
      if (betterRulesFound_thisSearchOperator_ground) {
        cout << "A new best rule-set was found:" << endl;
        rulesC.writeNice();
        cout << "SCORE = " << bestscore << endl;
      }
      else {cout << "No better rule-set was found (although I did my very best)." << endl;}
      if (round%10==0) {
        uintA covered_experiences_num;
        MT::Array< uintA > covered_experiences;
        arr responsibilities;
        rulesC.getResponsibilities(responsibilities, covered_experiences, covered_experiences_num);
        cout<<"Responsibilities after round "<<round<<":"<<endl;
        FOR1D(responsibilities, j) {
          cout << "[" << j << "] " << covered_experiences_num(j) << " " << responsibilities(j) << " " << covered_experiences(j) << endl;
        }
        cout << "-> Non-default responsibility: " << (1.0 - responsibilities(0)) << endl;
      }
    }
    scores.append(bestscore);
  }
  rulesC.recomputeDefaultRule();
  
  rulesC.sort();
 
	if (DEBUG>0) cout<<"Puh, that was it, now I can't find any better rules."<<endl;
	
  // LOGFILE WRITING [start]
  std::ofstream log_info;
  MT::String logfile_info;
  logfile_info << logfile << ".info";
  open(log_info, logfile_info);
  
  log_info<<"--- VOCABULARY ---"<<endl;
  log_info<<"*Primitive predicates*"<<endl;
  writeNice(logicObjectManager::p_prim, log_info);
  log_info<<"*Derived predicates*"<<endl;
  writeNice(logicObjectManager::p_derived, log_info);
  log_info<<"*Primitive functions*"<<endl;
  writeNice(logicObjectManager::f_prim, log_info);
  log_info<<"*Derived functions*"<<endl;
  writeNice(logicObjectManager::f_derived, log_info);
  log_info<<endl;
  log_info<<"--- RULES ---"<<endl;
  rulesC.rules.write(log_info);
  log_info<<endl;
  // responsibilities
  uintA covered_experiences_num;
  MT::Array< uintA > covered_experiences;
  arr responsibilities;
  rulesC.getResponsibilities(responsibilities, covered_experiences, covered_experiences_num);
  log_info<<"Responsibilities:"<<endl;
  FOR1D(responsibilities, j) {
      log_info << "[" << j << "] " << covered_experiences_num(j) << " " << responsibilities(j) << endl;
  }
  log_info << "-> Non-default responsibility: " << (1.0 - responsibilities(0)) << endl;
  log_info<<endl;
  log_info <<"--- STATISTICS ---"<<endl;
  log_info << "#rounds = " << (round-1) << endl;
//     log_info << "Scores: "<<scores<<endl;
  log_info <<"SEARCH OPERATORS:  (#applied  #improve  ratio     improveTotal  improveStep)"<<endl;
  FOR1D(searchOperators, i) {
      log_info<<"["<<i<<"] "<<"  "<<num_so_applied(i)<<"  "<<num_so_improvements(i) 
          << "  " <<  (num_so_improvements(i)/num_so_applied(i))
          <<"  "<<so_improvements(i);
      if (so_improvements(i)>0.0)log_info<<"  "<<(so_improvements(i)/num_so_improvements(i));
      log_info<<"  "<<searchOperators(i)->getName();
      log_info<<endl;
  }
  log_info << "History of successful SO applications: "<<endl<<so_successfulUsageHistory<<endl;
  log_info << "History of SO applications: "<<endl<<so_UsageHistory<<endl;
  // LOGFILE WRITING [end]
    
  if (DEBUG>0) {
      cout<<"==================================================="<<endl;
      cout<<"BEST RULE-SET:"<<endl;
      rulesC.writeNice(cout);
      cout<<"Responsibilities:"<<endl;
      FOR1D(responsibilities, j) {
          cout << "[" << j << "] " << covered_experiences_num(j) << " " << responsibilities(j) << " " << covered_experiences(j) << endl;
      }
      cout << "-> Non-default responsibility: " << (1.0 - responsibilities(0)) << endl;
      cout<<"STATISTICS:"<<endl;
      cout << "#rounds = " << (round-1) << endl;
      cout << "Scores: "<<scores<<endl;
      cout<<"SEARCH OPERATORS:  (name  #applied  #improve  improveTotal  improveStep)"<<endl;
      FOR1D(searchOperators, i) {
          cout<<"["<<i<<"] "<<"  "<<num_so_applied(i)<<"  "<<num_so_improvements(i)<<"  "<<so_improvements(i);
          if (so_improvements(i)>0.0) cout<<"  "<<(so_improvements(i)/num_so_improvements(i));
          cout<<"  "<<searchOperators(i)->getName()<<endl;
      }
      cout << "History of successful SO applications: "<<so_successfulUsageHistory<<endl;
      cout << "History of SO applications: "<<so_UsageHistory<<endl;
  }

  if (DEBUG > 0) cout << "learnRuleSet [END]" << endl;
}


}

#endif