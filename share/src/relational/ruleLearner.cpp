#include "ruleLearner.h"
#include <math.h>
#include <MT/algos.h>
#include "ruleReasoning.h"
#include "logicReasoning.h"

namespace TL {



void calcCoverage(SymbolicExperienceL& covered_experiences, uintA& covered_experiences_ids, const TL::Rule* r, const SymbolicExperienceL& experiences) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcCoverage [START]"<<endl;
  if (DEBUG>0) r->write(cout);
  covered_experiences.clear();
  covered_experiences_ids.clear();
  uint i;
  FOR1D(experiences, i) {
    if (DEBUG>0) cout<<"ex"<<i<< " " << endl;
    if (DEBUG>1) experiences(i)->write(cout);
    TL::SubstitutionSet subs;
    if (TL::ruleReasoning::cover_rule_groundedAction(experiences(i)->pre, experiences(i)->action, r, subs)) {
      covered_experiences.append(experiences(i));
      covered_experiences_ids.append(i);
      if (DEBUG>0) cout<<" --> 1"<<endl;
    }
    else {
      if (DEBUG>0) cout<<" --> 0"<<endl;
    }
  }
  if (DEBUG>0) cout<<"calcCoverage [END]"<<endl;
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    RuleSetContainer
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

TL::RuleSetContainer::RuleSetContainer() {
  init(NULL);
}

TL::RuleSetContainer::RuleSetContainer(const SymbolicExperienceL* _p_experiences) {
  init(_p_experiences);
}

void TL::RuleSetContainer::init(const SymbolicExperienceL* _p_experiences) {
  this->p_experiences = _p_experiences;
  if (this->p_experiences != NULL) {
    nonDefaultRules_per_experience.resize(this->p_experiences->N);
  }
}

void TL::RuleSetContainer::append(TL::Rule* rule, uintA& experiences_of_this_rule, MT::Array< uintA >& experiences_per_outcome_of_this_rule) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RuleSetContainer::append [START]"<<endl;}
  if (DEBUG>0) {rule->write(cout);  PRINT(experiences_of_this_rule);  PRINT(experiences_per_outcome_of_this_rule);}
  if (TL::ruleReasoning::isDefaultRule(rule)  &&   rules.num() > 0) {
    HALT("don't append default rule");
  }
  // (1) update rules
  rules.append(rule);
    
  if (!TL::ruleReasoning::isDefaultRule(rule)) {
    // (2 - A) update experiences_per_rule
    experiences_per_rule.append(experiences_of_this_rule);
    // (3) update nonDefaultRules_per_experience
    uint i;
    FOR1D(experiences_of_this_rule, i) {
      nonDefaultRules_per_experience(experiences_of_this_rule(i)).append(rules.num()-1);
    }
    // (4) update experiences_per_ruleOutcome
    experiences_per_ruleOutcome.append(experiences_per_outcome_of_this_rule);
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
  if (DEBUG>0) {cout<<"RuleSetContainer::append [END]"<<endl;}
}


void TL::RuleSetContainer::remove(uint id) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RuleSetContainer::remove [START]"<<endl;}
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
  if (DEBUG>0) {cout<<"RuleSetContainer::remove [END]"<<endl;}
}

void TL::RuleSetContainer::clear() {
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

void TL::RuleSetContainer::recomputeDefaultRule() {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"recomputeDefault [START]"<<endl;
  CHECK(TL::ruleReasoning::isDefaultRule(rules.elem(0)), "First rule should be default rule, digger");
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
  FOR1D((*p_experiences), i) {
    if (nonDefaultRules_per_experience(i).N != 1) {
      experiences_per_rule(0).append(i);
      if (((*p_experiences)(i))->noChange())
        experiences_per_ruleOutcome(0)(0).append(i);
      else
        experiences_per_ruleOutcome(0)(1).append(i);
    }
  }
  if (DEBUG>1) {PRINT(experiences_per_rule(0));}
  // finalize rule
  if ((*p_experiences).N == 0   ||   experiences_per_rule(0).N == 0) {
    double DEFAULT_CHANGE_PROB = 0.5;
    rules.overwrite(0, TL::ruleReasoning::generateDefaultRule(DEFAULT_CHANGE_PROB));
  }
  else {
    uint changes = 0;
    FOR1D(experiences_per_rule(0), i) {
      if ((*p_experiences)(experiences_per_rule(0)(i))->pre != (*p_experiences)(experiences_per_rule(0)(i))->post)
        changes++;
    }
    double prob_change = ((double) changes) / experiences_per_rule(0).N;
    if (DEBUG>1) {PRINT(prob_change);}
    TL::Rule* new_default_rule = TL::ruleReasoning::generateDefaultRule(prob_change, 0.05);
    rules.overwrite(0, new_default_rule);
  }
  if (DEBUG>1) {rules.elem(0)->write(cout);}
  if (DEBUG>0) {cout<<"recomputeDefault [END]"<<endl;}
}




void TL::RuleSetContainer::getResponsibilities(arr& responsibilities, MT::Array< uintA >& covered_experiences, uintA& covered_experiences_num) const {
  responsibilities.clear();
  covered_experiences.clear();
  covered_experiences_num.clear();
  responsibilities.resize(rules.num());
  covered_experiences.resize(rules.num());
  covered_experiences_num.resize(rules.num());
  covered_experiences_num.setUni(0);
  uint i;
  FOR1D((*p_experiences), i) {
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
    responsibilities(i) = (1.0 * covered_experiences_num(i)) / p_experiences->N;
  }
}



void TL::RuleSetContainer::getPartitionsForAction(MT::Array< uintA >& partitions, TL::Atom* action) const {
  partitions.clear();
  uint i;
  FOR1D_(rules, i) {
    if (rules.elem(i)->action == action) {
      partitions.append(experiences_per_rule(i));
    }
  }
}


// only for visualisation...
void rule_write_hack(TL::Rule* rule, MT::Array< uintA >& outcome_tripletts, bool with_action, ostream& os) {
  CHECK(outcome_tripletts.N = rule->outcomes.N, "wrong size");
//  os << "r" << endl;
  uint i, j;
  // Default rule does not have an action specified...
  if (with_action) {
    os << "ACTION: ";
    if (rule->action != NULL)
      rule->action->write(os, true);
    else
      os << "no_action";
    os << endl;
  }
  os << "CONTEXT: ";
  FOR1D(rule->context, i) {
    os << *rule->context(i) << " ";
  }
  os << endl;
  os << "OUT:" << endl;
  uint total_num_experiences = 0;
  FOR1D(rule->outcomes, i) {total_num_experiences += outcome_tripletts(i).N;}
  FOR1D(rule->outcomes, i) {
    os.precision(2);
    os << "  " << rule->probs(i) << " ";
    FOR1D(rule->outcomes(i), j) {
      rule->outcomes(i)(j)->write(os);
//       os<<outcomes(i)(j);
      os << " ";
    }
    if (i==rule->outcomes.N-1)
//       os<<rule->noise_changes;
      os<<"noise";
    os<<"    [";
    FOR1D(outcome_tripletts(i), j) {
      if (j > 10) {
        os<<"...";
        break;
      }
      os<<outcome_tripletts(i)(j)<<" ";
    }
    os <<"]";
    os << " (" << outcome_tripletts(i).N << "/" << total_num_experiences << " = ";
    if (outcome_tripletts(i).N == total_num_experiences) os<<"100";
    else os << ((uint) 100 * (outcome_tripletts(i).N * 1.0 / total_num_experiences));
    os << "%)" << endl;
  }
  if (TL::ruleReasoning::isDefaultRule(rule) &&  outcome_tripletts(0).N > 0) {os<<"ACHTUNG!!! Noise rule used to model!!"<<endl;  MT_MSG("ACHTUNG!!! Noise rule used to model!!");}
//   if (outcome_tripletts(rule->outcomes.N-1).N > 0) {os<<"ACHTUNG!!! Using noise-outcome!"<<endl;  MT_MSG("ACHTUNG!!! Using noise-outcome!");}
}

void TL::RuleSetContainer::writeNice(ostream& out, bool only_action) const {
  uint i, k;
  out<<"RULES:"<<endl;
  TL::Atom* last_action = NULL;
  FOR1D_(rules, i) {
    if (last_action != rules.elem(i)->action) {
      last_action = rules.elem(i)->action;
      out<<"##### ";  rules.elem(i)->action->write(out, true);  out<<endl;
    }
    out<<"# "<<i<<"  covering "<<experiences_per_rule(i).N<<" experiences [";
    FOR1D(experiences_per_rule(i), k) {
      out<<experiences_per_rule(i)(k)<<" ";
      if (k > 10) {
        out<<"...";
        break;
      }
    }
    out << "]" <<endl;
    if (only_action) {
      rules.elem(i)->action->write(out); out<<endl;
    }
    else {
      rule_write_hack(rules.elem(i), experiences_per_ruleOutcome(i), false, out);
    }
  }
  out<<"EXPERIENCES:"<<endl;
  FOR1D(nonDefaultRules_per_experience, i) {
    if (nonDefaultRules_per_experience(i).N > 0)
    out<<i<<":"<<nonDefaultRules_per_experience(i)<<"  ";
  }
  out<<endl;
}

void TL::RuleSetContainer::write_experiencesWithRules(ostream& os) const {
  uint i, k;
  FOR1D((*p_experiences), i) {
    os<<"--------------"<<endl;
    os<<"EXAMPLE #"<<i<<":"<<endl;
    ((*p_experiences)(i))->write(os);
    os<<"  ---> COVERING RULES: "<<nonDefaultRules_per_experience(i)<<endl;
    FOR1D(nonDefaultRules_per_experience(i), k) {
      rules.elem(nonDefaultRules_per_experience(i)(k))->write(os);
    }
  }
}

void TL::RuleSetContainer::write_rulesWithExperiences(ostream& os) const {
  uint i, k;
  FOR1D_(rules, i) {
    os<<"--------------"<<endl;
    os<<"RULE #"<<i<<":"<<endl;
    rules.elem(i)->write(os);
    os<<"COVERS "<<experiences_per_rule(i).N<<" experiences."<<endl;
    FOR1D(experiences_per_rule(i), k) {
      os<<"Example #"<<experiences_per_rule(i)(k)<<endl;
      ((*p_experiences)(experiences_per_rule(i)(k)))->write(os);
    }
  }
}

void TL::RuleSetContainer::sanityCheck(bool ignore_default_rule) const {
  TL::ruleReasoning::checkRules(rules);
  
  if (rules.num() != experiences_per_rule.N) {
    cout<<"FAILED SANITY CHECK:"<<endl;
    writeNice(cout, true);
    HALT("sanity check failed 0-A");
  }
  if (p_experiences->N != nonDefaultRules_per_experience.N) {
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
  
  // check for neg free DRs
  FOR1D_(rules, i) {
    uintA negFreeDRs;
    TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, *rules.elem(i));
    if (negFreeDRs.N > 0) {
      cout<<endl<<endl<<endl;
      cout<<"FAILED SANITY CHECK  -  NEGATIVE FREE DEICTIC REFERNCES"<<endl;
      cout<<"RULE:  "<<endl;  rules.elem(i)->write();
      PRINT(i);
      PRINT(negFreeDRs);
      HALT("FAILED SANITY CHECK  -  NEGATIVE FREE DEICTIC REFERNCES");
    }
  }

  // Non-default rules
  FOR1D_(rules, i) {
    if (i == 0)
      continue;
    FOR1D((*p_experiences), k) {
      TL::SubstitutionSet subs;
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
        HALT("sanity check failed 1:  supposed_to_cover__1 != supposed_to_cover__2");
      }
      bool covers = TL::ruleReasoning::cover_rule_groundedAction((*p_experiences)(k)->pre, (*p_experiences)(k)->action, rules.elem(i), subs);
      if (covers != supposed_to_cover__1) {
        cout<<"FAILED SANITY CHECK 2:  covers != supposed_to_cover__1 "<<endl;
        if (covers) {
          cout<<"Rule covers experience although it is supposed to NOT cover experience according to rulesC-information."<<endl;
        }
        else {
          cout<<"Rule does NOT cover experience although it is supposed to cover experience according to rulesC-information."<<endl;
        }
        writeNice(cout, true);
        cout<<"rule #i="<<i<<endl;
        cout<<"experience #k="<<k<<endl;
        PRINT(supposed_to_cover__1);
        PRINT(covers);
        cout<<"Rule:"<<endl;  rules.elem(i)->write();  cout<<endl;
        cout<<"SymbolicExperience:"<<endl;  (*p_experiences)(k)->write();  cout<<endl;
        HALT("sanity check failed 2");
      }
    }
    uint total_experiences_in_outcomes = 0;
    FOR1D(experiences_per_ruleOutcome(i), k) {
      total_experiences_in_outcomes += experiences_per_ruleOutcome(i)(k).N;
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
    if (total_experiences_in_outcomes < experiences_per_rule(i).N) {
      cout<<"FAILED SANITY CHECK:"<<endl;
      writeNice(cout, false);
      PRINT(i);
      PRINT(total_experiences_in_outcomes);
      PRINT(experiences_per_ruleOutcome(i));
      PRINT(experiences_per_rule(i));
      HALT("sanity check failed 4");
    }
  }
  
  if (!ignore_default_rule) {
    // Check that default rule does not cover experiences which other rules cover
    FOR1D(experiences_per_rule(0), i) {
      FOR1D_(rules, k) {
        if (k == 0)
          continue;
        if (experiences_per_rule(k).findValue(experiences_per_rule(0)(i)) >= 0) {
          cout<<"FAILED SANITY CHECK:  default rule covers experience which also other rules covers"<<endl;
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
}



void TL::RuleSetContainer::sort() {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"sort [START]"<<endl;}
  TL::RuleSet new__rules;
  MT::Array< uintA > new__nonDefaultRules_per_experience;
  MT::Array< uintA > new__experiences_per_rule;
  MT::Array< MT::Array < uintA > > new__experiences_per_ruleOutcome;
  
  uintA action_ids;
  uint i, k;
  FOR1D_(rules, i) {
    action_ids.setAppend(rules.elem(i)->action->pred->id);
  }
  TL::sort_asc(action_ids);  // descending
  CHECK(action_ids(0) == TL::DEFAULT_ACTION_PRED__ID, "");
  
  TL::Substitution sub;
  FOR1D(action_ids, i) {
    if (DEBUG>0) {cout<<"Dealing with action_ids(i="<<i<<")="<<action_ids(i)<<endl;}
    uintA rules_ids_for_this_action;
    uintA num_experiences;
    FOR1D_(rules, k) {
      if (rules.elem(k)->action->pred->id == action_ids(i)) {
        rules_ids_for_this_action.append(k);
        num_experiences.append(experiences_per_rule(k).N);
        // hack for better sorting [start]
        num_experiences.last() = num_experiences.last() * 1000;
        if (experiences_per_rule(k).N > 0)  num_experiences.last() += experiences_per_rule(k)(0);
        // hack for better sorting [end]
      }
    }
    uintA sorted_indices;
    TL::sort_desc_keys(sorted_indices, num_experiences);
//     PRINT(sorted_indices);
    FOR1D(sorted_indices, k) {
      new__rules.append(rules.elem(rules_ids_for_this_action(sorted_indices(k))));
      sub.addSubs(rules_ids_for_this_action(sorted_indices(k)), new__rules.num()-1);
      new__experiences_per_rule.append(experiences_per_rule(rules_ids_for_this_action(sorted_indices(k))));
      new__experiences_per_ruleOutcome.append(experiences_per_ruleOutcome(rules_ids_for_this_action(sorted_indices(k))));
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
  
//   sanityCheck();
  if (DEBUG>0) {cout<<"sort [END]"<<endl;}
}





// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    C O S T   F U N C T I O N
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


SymbolicExperienceL cf_experiences_coveredByCurrentRule;
boolA cf_coverage_outcome_example;
double cf_sum;
double cf_pos;
double cf_p_min;

// lower bound p_min for state transition given noise outcome
void CostFunction::setNoiseStateProbability(double p_min) {
  cf_p_min = p_min;
}

void CostFunction::setPenaltySum(double pen_sum) {
  cf_sum = pen_sum;
}

void CostFunction::setPenaltyPos(double pen_pos) {
  cf_pos = pen_pos;
}

void CostFunction::setRuleCoveredExperiences(const SymbolicExperienceL& coveredEx) {
  cf_experiences_coveredByCurrentRule.clear();
  cf_experiences_coveredByCurrentRule= coveredEx;
}

void CostFunction::setOutcomesCoverage(const boolA& coverage) {
  cf_coverage_outcome_example = coverage;
}


double CostFunction::loglikelihood(const arr& probs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"loglikelihood [START]"<<endl;
  double loglikelihood = 0.0, innerSum;
  uint i,e;
  FOR1D(cf_experiences_coveredByCurrentRule, e) {
    innerSum = 0.0;
    if (DEBUG>0) cout<<"ex "<<e<<":"<<endl;
    FOR1D(probs, i) {
      if (cf_coverage_outcome_example(i, e)) {
        if (i < probs.N - 1) {
          innerSum += probs(i);
          if (DEBUG>0) cout<<"  o"<<i<<" 1 * "<<probs(i)<<endl;
        }
        else { // noise outcome
          innerSum += cf_p_min * probs(i);
          if (DEBUG>0) cout<<"  o"<<i<<" "<<cf_p_min<<" * "<<probs(i)<<endl;
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
double CostFunction::calc(const arr& in) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"CostFunction::calc [START]"<<endl;
  CHECK(in.N == cf_coverage_outcome_example.d0, "invalid number of arguments")
  uint i;
  // calc log-likelihood
  double loglik = loglikelihood(in);
  double sumConstraint = cf_sum * pow(sum(in) - 1.0, 2);
  double posConstraint = 0.0;
  FOR1D(in, i) {
    if (in(i) < 0.0) {
      posConstraint += pow(in(i), 2);
    }
  }
  posConstraint *= cf_pos;
  double cost = -loglik + sumConstraint + posConstraint;
  if (DEBUG>0) cout<<"cost="<<cost<<" (-loglik="<<-loglik<<", sumConstraint="<<sumConstraint<<", posConstraint="<<posConstraint<<")"<<endl;
  if (DEBUG>0) cout<<"CostFunction::calc [END]"<<endl;
  return cost;
}


// points into direction of steepest ascent
void CostFunction::calc_grad(arr& out, const arr& in) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calc_grad [START]"<<endl;
  CHECK(in.N == cf_coverage_outcome_example.d0, "invalid number of arguments")
  out.resize(in.N);
  double sumIn = sum(in);
  double loglik_grad, const1_grad, const2_grad, denom_sum;
  uint i, i2, e;
  // calc gradient for each prob
  FOR1D (in, i) {
    // check for all experiences
    loglik_grad = 0.0;
    if (i < in.N - 1) {
      CHECK(sum(cf_coverage_outcome_example.sub(i,i,0,cf_coverage_outcome_example.d1-1)), "At least one example should be covered!");
    }
    FOR1D(cf_experiences_coveredByCurrentRule, e) {
      // I[covers(s', o_i)]
      if (cf_coverage_outcome_example(i, e)) {
        // denominator sum
        denom_sum = 0.0;
        FOR1D(in, i2) {
          if (cf_coverage_outcome_example(i2, e)) {
            if (i2 < in.N - 1) // non-noise
              denom_sum += in(i2);
            else // noise outcome
              denom_sum += cf_p_min * in(i2);
          }
        }
        if (i < in.N-1) // non-noise
          loglik_grad += 1. / denom_sum;
        else // noise
          loglik_grad += cf_p_min / denom_sum;
      }
    }
    
    // constraint 1
    const1_grad = 2. * cf_sum * (sumIn - 1.0) * in(i);
    
    // constraint 2
    if (in(i) < 0.0) {
      const2_grad = cf_pos * 2. * in(i);
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


SearchOperator::SearchOperator(double alpha_PEN, double p_min, ProbabilityOptimizationType param_opt_type) {
  this->alpha_PEN = alpha_PEN;
  this->p_min = p_min;
  this->param_opt_type = param_opt_type;
  this->approximative = false;
  
  // TODO adapt these guys...
  this->pen_sum = 20.0;
  this->pen_pos = 20.0;
}


void SearchOperator::set_p_min(double p_min) {
  this->p_min = p_min;
}



void SearchOperator::calcCoverage_outcomes(const MT::Array< LitL >& potential_outcomes, const SymbolicExperienceL& covered_experiences, const TL::Rule* old_rule, boolA& coverage) {
  // ACHTUNG: old_rule hat womoeglich andere Outcomes!!
  uint DEBUG = 0;
  if (DEBUG>0) cout << "calcOutcomesCoverage [START]" << endl;
  if (DEBUG>1) {
    cout<<"Old rule context:  "<<old_rule->context<<endl;
    cout<<"Old rule action:  "<<*old_rule->action<<endl;
    cout<<"Potential outcomes (#="<<potential_outcomes.N<<"):  "<<endl;
    uint o; FOR1D(potential_outcomes, o) {cout<<o<<": "<<potential_outcomes(o)<<endl;}
  }
  coverage.resize(potential_outcomes.N, covered_experiences.N);
  uint i, e;
  FOR1D(covered_experiences, e) {
    if (DEBUG>0) {cout<<"### Ex "<<e<<":"<<endl; covered_experiences(e)->write(cout);}
    // provide the initial substitution given by action and context --> setting all vars
    TL::SubstitutionSet subs;
    bool covered = TL::ruleReasoning::cover_rule_groundedAction(covered_experiences(e)->pre, covered_experiences(e)->action, old_rule, subs);
    CHECK(covered, "Example must be preAct_covered by rule!");
    if (DEBUG>0) {cout<<"Substitution:  "; subs.elem(0)->write(cout);cout<<endl;}
    FOR1D(potential_outcomes, i) {
      if (DEBUG>1) {cout<<"--- Potential outcome #i="<<i<<": "<<potential_outcomes(i)<<endl;}
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
        LitL outcome_grounded;
        logicReasoning::applyOriginalSub(*subs.elem(0), potential_outcomes(i), outcome_grounded);
        TL::State successor;
        TL::ruleReasoning::calcSuccessorState(covered_experiences(e)->pre, outcome_grounded, successor, false);
        if (DEBUG>1) {cout<<"[Grounded potential outcome: ";TL::write(outcome_grounded);cout<<"]";}
        if (DEBUG>2) {
          cout<<"\nTRUE POST:   "; covered_experiences(e)->post.write(cout); cout<<endl;
          cout<<"PRED POST:   "; successor.write(cout); cout<<endl;
        }
        if (covered_experiences(e)->post == successor)
          coverage(i, e) = true;
        else
          coverage(i, e) = false;
      }
      if (DEBUG>1) cout<<"---> coverage="<<coverage(i,e)<<endl;
    }
  }
  if (DEBUG>0) cout << "calcOutcomesCoverage [END]" << endl;
}



void SearchOperator::calcSubsumption(boolA& subsumes, const boolA& coverage) {
  CHECK(coverage.nd == 2, "invalid coverage matrix")
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









void SearchOperator::integrateNewRules(const TL::RuleSetContainer& rulesC_old, const TL::RuleSetContainer& rulesC_2add, 
                                       const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_new) {
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
  
  // For each new rule r'
  FOR1D_(rulesC_2add.rules, i) {
    // Remove rules in R' that cover any experiences r' covers
    uintA& covered_new = rulesC_2add.experiences_per_rule(i);
    // Clean-up rules
    TL::ruleReasoning::cleanup(*rulesC_2add.rules.elem(i));
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
  // Recompute the set of experiences that the default rule in R' covers and the parameters of this default ruleLearner
  rulesC_new.recomputeDefaultRule();
  
  if (DEBUG > 1) {
    if (DEBUG>2) {
      cout << "***** Old rules *****" << endl;
      rulesC_old.writeNice(cout, true);
    }
    cout << "***** New rules *****" << endl;
    rulesC_new.writeNice(cout, true);
  }
  
  if (DEBUG > 0) {cout << "integrateNewRules [END]" << endl;}
}


// Algorithm of Figure 4 in Pasula et al. (2007)
void SearchOperator::createRuleSets(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, 
        MT::Array< TL::RuleSetContainer >& set_of_new_rulesC) {
  uint DEBUG = 0;
  set_of_new_rulesC.clear();
  reset();
  uint i;
  while (true) {
    TL::RuleSetContainer rulesC_2add(&experiences);
    // this is where the local knowledge of the individual search operators comes in
    findRules(rulesC_old, experiences, rulesC_2add);
    if (DEBUG>1) {
      FOR1D_(rulesC_2add.rules, i) {
        rulesC_2add.rules.elem(i)->write(cout);
      }
    }
    if (rulesC_2add.rules.num() == 0)
      break;
    TL::RuleSetContainer rulesC_new(&experiences);
    integrateNewRules(rulesC_old, rulesC_2add, experiences, rulesC_new);
    set_of_new_rulesC.append(rulesC_new);
    if (approximative) {
      if (set_of_new_rulesC.d0 >= APPROXIMATOR__RULES_PER_ROUND)
        break;
    }
  }
}







// remove outcomes that (i) do not cover any example and (ii) have zero-probability  and (iii) sets coverage for cost function
void SearchOperator::produceTrimmedOutcomes(MT::Array< LitL >& outcomes, arr& probs, boolA& coverage, const SymbolicExperienceL& coveredExperiences, const TL::Rule& rule,
                                            double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"produceTrimmedOutcomes [START]"<<endl;
  uint i;
  bool removed, atleastone;
  if (DEBUG>1) {cout<<"Outcomes before:\n"; TL::write(outcomes); PRINT(coveredExperiences);}
  
  // (i) remove outcomes that do not cover any example
  if (DEBUG>1) cout<<"Removing outcomes that do not cover any example"<<endl;
  calcCoverage_outcomes(outcomes, coveredExperiences, &rule, coverage);
  CostFunction::setOutcomesCoverage(coverage);
  if (DEBUG>3) {PRINT(coverage);}
  removed = false;
  MT::Array< LitL > o_help;
  FOR1D(outcomes, i) {
    atleastone = sum(coverage.sub(i,i,0,coverage.d1-1));
    if (i < outcomes.N-1  &&  !atleastone) { // only for non-noise outcomes
      removed = true;
      if (DEBUG>2) cout << "Outcome " << i << " covers 0 experiences and will be removed."<<endl;
    }
    else
      o_help.append(outcomes(i));
  }
  if (DEBUG>1) {cout<<"Outcomes now:\n"; TL::write(o_help);}
  if (removed) {
    calcCoverage_outcomes(o_help, coveredExperiences, &rule, coverage);
    CostFunction::setOutcomesCoverage(coverage);
    if (DEBUG>3) PRINT(coverage)
  }

  // (ii) remove zero-probability outcomes
  if (DEBUG>1) cout<<"Removing zero-prob outcomes"<<endl;
  // learn params
  learnParameters(o_help, probs, pen_sum, pen_pos, param_opt_type);
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
    calcCoverage_outcomes(outcomes, coveredExperiences, &rule, coverage);
    CostFunction::setOutcomesCoverage(coverage);
    if (DEBUG>3) PRINT(coverage)
  }
  if (DEBUG>1) {cout<<"Outcomes final:\n"; TL::write(outcomes);}
  if (DEBUG>0) cout<<"produceTrimmedOutcomes [END]"<<endl;
}



// Steps:
// (1) Determine basic outcomes: changes from pre to post in experiences
// (2) Collapse identical outcomes
// (3) Greedily improve outcomes based on add- and remove-operator
//
// When we create new outcomes, we have to check whether there are outcomes with
// (i) no experiences covered or (ii) a probability of 0. In both cases, the corresponding
// outcomes need to be deleted. --> method produceTrimmedOutcomes(...), see above
//
// TODO cope with comparisons (a la Zettlemoyer et al., 2004)
void SearchOperator::induceOutcomes(TL::Rule* r, MT::Array< uintA >& coveredExperiences_per_outcome, const SymbolicExperienceL& coveredExperiences, const uintA& covered_experiences_ids,
                                    double alpha_PEN, double p_min, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "induceOutcomes [START]" << endl;
  
  CHECK(covered_experiences_ids.N == coveredExperiences.N, "");
  CHECK(coveredExperiences.N>0, "No experiences, bro!");
    
  // calc changes first per example
  uint i, j;
  if (DEBUG > 0) {r->write(cout);  PRINT(coveredExperiences.N);}
  
  // prepare cost function (stays like this for complete following induceOutcomes-procedure)
  CostFunction::setRuleCoveredExperiences(coveredExperiences);
  CostFunction::setPenaltySum(pen_sum * coveredExperiences.N);
  CostFunction::setPenaltyPos(pen_pos * coveredExperiences.N);
  CostFunction::setNoiseStateProbability(p_min);

    
  // (1) Determine basic outcomes = changes from pre to post = for each covered example a separate outcome
  MT::Array< LitL > outcomes_basic;
  FOR1D(coveredExperiences, i) {
    if (DEBUG>4) {cout<<"====== Using ex "<<i<<":"<<endl; coveredExperiences(i)->write(cout);}
    TL::SubstitutionSet subs;
    bool covered = TL::ruleReasoning::cover_rule_groundedAction(coveredExperiences(i)->pre, coveredExperiences(i)->action, r, subs);
    CHECK(covered, "An uncovered example! Be careful when calling methods, noob!");
    if (subs.num() != 1) {
      cout << "FAILING: TOO " << (subs.num() > 1 ? "MANY" : "FEW") << "SUBS"<<endl;
      cout<<"State: ";coveredExperiences(i)->pre.write(cout);cout<<endl;
      cout<<"Action: ";coveredExperiences(i)->action->write(cout);cout<<endl;
      cout<<"Rule: "<<endl;r->write(cout);cout<<endl;
      cout << "Substitutions: ";
      uint s;
      FOR1D_(subs, s) {
        subs.elem(s)->write(cout); cout << endl;
      }
    }
    CHECK(subs.num()==1, "Cannot be deictic variant! (Should've been taken care for somewhere else.)")

    LitL nextOutcome;
    // just take first subs --> DEICTIC
    uint subsId = 0;
    
    TL::Substitution invSub;
    subs.elem(subsId)->getInverse(invSub);
    if (DEBUG > 4) {
      cout<<"Substitution: "; subs.elem(subsId)->write(cout); cout << endl;
      cout<<"Inverse Substitution: "; invSub.write(cout);cout << endl;
    }
      
    // insert add-predicates
    FOR1D(coveredExperiences(i)->add, j) {
      if (coveredExperiences(i)->add(j)->atom->pred->type == TL::Predicate::predicate_simple) {
        nextOutcome.setAppend(logicReasoning::applyOriginalSub(invSub, coveredExperiences(i)->add(j)));
      }
    }
    // insert negations of del-predicates
    FOR1D(coveredExperiences(i)->del, j) {
      if (coveredExperiences(i)->del(j)->atom->pred->type == TL::Predicate::predicate_simple) {
        TL::Literal* lit = logicObjectManager::getLiteralNeg(coveredExperiences(i)->del(j));
        nextOutcome.setAppend(logicReasoning::applyOriginalSub(invSub, lit));
      }
    }
    LitL nextOutcome_pureAbstract;
    logicReasoning::filterPurelyAbstract(nextOutcome, nextOutcome_pureAbstract);
    outcomes_basic.append(nextOutcome_pureAbstract);

    if (DEBUG>3) {
      cout<<"nextOutcome: "; TL::write(nextOutcome); cout<<endl;
      cout<<"nextOutcome_pureAbstract: "; TL::write(nextOutcome_pureAbstract); cout<<endl;
    }
  }
  // add noise outcomes
  LitL noiseOutcome;
  outcomes_basic.append(noiseOutcome);
  
  if (DEBUG > 0) {
    cout << "SymbolicExperience outcomes (incl. noise outcome):" << endl;
    FOR1D(outcomes_basic, i) {
      cout << "(" << i << ") ";
      TL::write(outcomes_basic(i));
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
      if (TL::equivalent(outcomes_basic(i), outcomes_basic(j)))
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
    cout << "Collapsed experiences outcomes (copies removed)  (incl. noise outcome):" << endl;
    FOR1D(outcomes, i) {cout << "(" << i << ") "; TL::write(outcomes(i)); cout << endl;}
  }

  // trim outcomes
  arr probs;
  boolA coverage;
  produceTrimmedOutcomes(outcomes, probs, coverage, coveredExperiences, *r, pen_sum, pen_pos, param_opt_type);
  
  // (3) Greedily improve outcomes
  // score needs to be optimized

  double score, bestScore;
  double loglik;
  
  // evaluate and calc score
  loglik = CostFunction::loglikelihood(probs);
  score = loglik - alpha_PEN * TL::logicReasoning::numberLiterals(outcomes);
  if (DEBUG > 1) {
    cout << "\nBASIC OUTCOMES:" << endl;
    FOR1D(outcomes, i) {
      cout << "("<< i << ") ";
      TL::write(outcomes(i));
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
          cout<<i<<" :";TL::write(outcomes(i));cout<<endl;
          cout<<j<<" :";TL::write(outcomes(j));cout<<endl;
          cout<<"  --> ";TL::write(unifiedOutcome);cout<<endl;
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
          TL::write(outcomes_new(i));
          cout << endl;
        }
      }
      
      // procduce trimmed outcomes
      produceTrimmedOutcomes(outcomes_new, probs_new, coverage_new, coveredExperiences, *r, pen_sum, pen_pos, param_opt_type);
            
      // evaluate and calc score
      loglik = CostFunction::loglikelihood(probs_new);
      score = loglik - alpha_PEN * TL::logicReasoning::numberLiterals(outcomes_new);
            
      if (DEBUG > 1) {
        cout << "\nNEW OUTCOMES (now with new_probs and score):" << endl;
        FOR1D(outcomes_new, i) {
          cout << i << ": ";
          TL::write(outcomes_new(i));
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
  r->outcomes = outcomes;
  r->probs = probs;
  
  // Calc experiences_per_outcome
  coveredExperiences_per_outcome.clear();
  coveredExperiences_per_outcome.resize(coverage.d0);
  uint i_out, i_ex;
  FOR2D(coverage, i_out, i_ex) {
    if (coverage(i_out, i_ex)) {
      coveredExperiences_per_outcome(i_out).append(covered_experiences_ids(i_ex));
    }
  }
  
  
  // sort for probabilities
  MT::Array< LitL > sorted__outcomes(r->outcomes.N);
  arr sorted__probs(r->outcomes.N);
  MT::Array< uintA > sorted__coveredExperiences_per_outcome(r->outcomes.N);
  uintA sorted_indices;
  TL::sort_desc_keys(sorted_indices, probs);
  uint current_i = 0;
  FOR1D(sorted_indices, i) {
    if (sorted_indices(i) == r->outcomes.N-1)   // skip noise outcome
      continue;
    sorted__outcomes(current_i) = r->outcomes(sorted_indices(i));
    sorted__probs(current_i) = r->probs(sorted_indices(i));
    sorted__coveredExperiences_per_outcome(current_i) = coveredExperiences_per_outcome(sorted_indices(i));
    current_i++;
  }
  CHECK(current_i == r->outcomes.N-1, "");
  sorted__probs.last() = r->probs.last();   // noise outcome
  sorted__coveredExperiences_per_outcome.last() = coveredExperiences_per_outcome.last();   // noise outcome
  r->outcomes = sorted__outcomes;
  r->probs = sorted__probs;
  coveredExperiences_per_outcome = sorted__coveredExperiences_per_outcome;
  
  if (DEBUG > 0) {
    cout << "\nFINAL NEW OUTCOMES (now with probs and score):" << endl;
    PRINT(covered_experiences_ids);
    PRINT(coveredExperiences_per_outcome);
    FOR1D(r->outcomes, i) {
      cout << i << ": ";
      TL::write(r->outcomes(i));
      cout << " " << r->probs(i) << " " << coveredExperiences_per_outcome(i);
      cout << endl;
    }
    cout << " --> score=" << bestScore << endl;
  }
  
  // check [START]
  uint used_outcomes = 0;
  FOR1D(coveredExperiences_per_outcome, i) {
    used_outcomes += coveredExperiences_per_outcome(i).N;
  }
  if (used_outcomes < coveredExperiences.N) {
    PRINT(used_outcomes);
    PRINT(covered_experiences_ids);
    PRINT(coveredExperiences_per_outcome);
    FOR1D(r->outcomes, i) {
      cout << i << ": ";
      TL::write(r->outcomes(i));
      cout << " " << r->probs(i) << " " << coveredExperiences_per_outcome(i);
      cout << endl;
    }
    HALT("error in experiences_per_outcome calculation");
  }
  // check [END]
  
  
  if (DEBUG>0) cout << "induceOutcomes [END]" << endl;
}








// Return value needs to be MINIMIZEd.
double SearchOperator::learnParameters_constrainedCostfunction(const MT::Array< LitL >& outcomes, doubleA& probs, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type) { 
  uint DEBUG = 0;
  
  if (DEBUG > 0) {
    cout << endl;
    cout << "Learning Parameters [START]" << endl;
    PRINT(pen_sum)
    PRINT(pen_pos)
  }
    
  uint i, j;
  
  // ATTENTION WE ASSUME COVERAGE HAS BEEN SET CORRECTLY BEFORE
  FOR1D(outcomes, i) {
    if (i == outcomes.N-1)  // omit noise outcome
      continue;
    CHECK(sum(cf_coverage_outcome_example.sub(i,i,0,cf_coverage_outcome_example.d1-1)), "At least one example should be covered for outcome i="<<i<<"!");
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
    cost = CostFunction::calc(probs);
    return cost;
  }
  // non-default probs = 1/(N-1) (1 - default_prob)
  for (i=0; i<outcomes.N-1; i++) {
    probs(i) = (1. / (probs.N-1.)) * (1. - probs.last());
  }
  CHECK(TL::isZero(sum(probs)-1), "Bad init probs!")
  
  if (DEBUG > 0) {
    PRINT(cf_coverage_outcome_example) // cf_coverage_outcome_example == coverage
  }
  
  
  
  // ----------------------------
  // OPTIMIZATION
  
  // prepare optimization algorithm
  double (*f)(const arr&);
  f = CostFunction::calc;
  void (*df)(arr&,const arr&);
  df = CostFunction::calc_grad;
  
//  MT::checkGradient(f, df, probs, 0.05);
  
  arr gradients;
  cost = CostFunction::calc(probs);
  
  if (DEBUG > 0)
    cout << "init_probs = " << probs << " C=" << cost << endl;

  double STEP_SIZE = 0.01;
  double STOPPING_THRESHOLD = 0.001;
  uint MAX_STEPS = 1000;
  if (param_opt_type == grad_desc) {
    j=0;
    if (DEBUG>0) cout<<"Gradient Descent:"<<endl;
    while(fabs(diff_cost) > STOPPING_THRESHOLD) {
      CostFunction::calc_grad(gradients, probs);
      FOR1D(probs, i) {
        probs(i) -= STEP_SIZE * TL::signOf(gradients(i));
      }
      oldCost = cost;
      cost = CostFunction::calc(probs);
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
  else if (param_opt_type == rprop) {
    Rprop rp;
    rp.init(0.025);
    rp.dMin = 1e-9;
    i=0;
    if (DEBUG>0) cout<<"RProp:"<<endl;
    while(fabs(diff_cost) > STOPPING_THRESHOLD) {
      CostFunction::calc_grad(gradients, probs);
      rp.step(probs, gradients);
      oldCost = cost;
      cost = CostFunction::calc(probs);
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
        std::cerr <<endl<<endl<<endl<< "probs = " << probs << endl;
        MT_MSG("WARNING!!! Cannot learn rule probabilities! (No convergence.)");
      }
    }
  }
  else
    HALT("Unknown optimization strategy")
  
  
  // ----------------------------
  // POST-PROCESSING
  
  // In the end ensure by hand that pi >=0
  double EPSILON__PROB_NEG1 = 0.10;  // 0.01
  double EPSILON__PROB_NEG2 = 0.15;
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
      pen_pos *= 1.3;
      MT_MSG(warning)
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
  double EPSILON__PROB_SUM2 = 0.25;
  if (fabs(probSum - 1) > EPSILON__PROB_SUM2) {
    MT::String warning;
    warning << "Param. optimization failed - sum clearly different from 1: " << probSum << " found=" << (probs*probSum) << " rescaled=" << probs;
    HALT(warning);
  }
  if (fabs(probSum - 1) > EPSILON__PROB_SUM1) {
    MT::String warning;
    warning << "Param. optimization awkward - sum clearly different from 1: " << probSum << " found=" << (probs*probSum) << " rescaled=" << probs;
    MT_MSG(warning);
    pen_sum *= 1.3;
  }
  
  // In the end, ensure that noise outcome (= last outcome) has non-zero prob.
  probs.last() += 1e-5;
  probs(0) -= 1e-5;

  if (DEBUG > 0) {
    cout << "Learned params: " << probs << endl;
    cout << "Learning Parameters [END]" << endl;
  }

  // determine final score
  return cost;
}




double SearchOperator::learnParameters(const MT::Array< LitL >& outcomes, doubleA& probs, double pen_sum, double pen_pos, ProbabilityOptimizationType param_opt_type) {
  return learnParameters_constrainedCostfunction(outcomes, probs, pen_sum, pen_pos, param_opt_type);
// 	Other Ideas:
// 	(a) nur in die Richtung des groessten Gradienten gehen; dort bestimmte Schritteweise; Rest anpassen
// 	(b) ohne Constraints berechnen; nach jedem Schritt Werte auf Verteilung rueckrechnen
}



const char* SearchOperator::getName() {
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
void ExplainExperiences::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"ExplainExperiences::findRules [START]"<<endl;
  uint i;
//  PRINT(TL::Rule::globalRuleCounter)
  for (i=nextPotentialExperience; i<experiences.N; i++) {
    uintA& covering_rules = rulesC_old.nonDefaultRules_per_experience(i);
    if (DEBUG>1) {
      TL::write(rulesC_old.rules);
      cout<<"#Covering non-default rules for example (" << i << ") = " << covering_rules.N << endl;
    }
    if (covering_rules.N == 0) {
      // Create new rule by explaining current example (sets context and action)
      if (DEBUG>0) cout << "findRules: let's explain #" << i << endl;
      if (DEBUG>2) {experiences(i)->write(cout);}
      TL::Rule* newRule = explainExperience(experiences(i));
      // calc experience coverage
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
      // Estimate new outcomes for r'
      CHECK(covered_experiences.N>0, "At least the explained example should be covered.")
      MT::Array< uintA > experiences_per_outcome;
      induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
      if (DEBUG>0) {cout<<"New Rule:"<<endl; newRule->write(cout);}
      if (DEBUG>0) {cout<<"#Covering experiences: "<<covered_experiences.N<<endl;}
      rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
//       rulesC_2add.sanityCheck();
      nextPotentialExperience = i+1;
      break;
    }
  }
//  PRINT(rules2_add.num());
//  PRINT(TL::Rule::globalRuleCounter)
	if (DEBUG>0) cout<<"ExplainExperiences::findRules [END]"<<endl;
}


TL::Rule* ExplainExperiences::explainExperience(SymbolicExperience* ex) {
//   return explainExperience_deictic(ex);
  return explainExperience_deictic_ALL_DRs(ex);
}


// TODO Should be replaced by an explicit vocabulary that shall be used for the rules...
void filter_language(LitL& lits) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout << "filter_language before: [N="<<lits.N<<"] "<<lits<<endl;}
  // determine predicates to be filtered
  uintA filtered_predicate_ids;
  uint i;
  FOR1D(lits, i) {
    if (lits(i)->atom->pred->name == "above") {
      filtered_predicate_ids.append(lits(i)->atom->pred->id);
    }
  }
  if (DEBUG>0) {PRINT(filtered_predicate_ids);}
  
  lits.memMove = true;
  FOR1D_DOWN(lits, i) {
    if (lits(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      ComparisonLiteral* clit = (ComparisonLiteral*) lits(i);
      if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived) {
        if (DEBUG>0) {cout<<"Removing "<<*lits(i)<<endl;}
        lits.remove(i);
      }
    }
    else if (filtered_predicate_ids.findValue(lits(i)->atom->pred->id) >= 0) {
      if (DEBUG>0) {cout<<"Removing "<<*lits(i)<<endl;}
      lits.remove(i);
    }
  }
  
  if (DEBUG>0) {cout << "filter_language after: [N="<<lits.N<<"] "<<lits<<endl;}
}


// TODO YOU MAY WANT TO FILTER SOME LITERALS WHICH ARE CLEARLY NEVER TRUE.
//  Brings in some domain knowledge to make learning faster.
//  Should be replaced by an approach of principled language constraints...
void trim_hack(LitL& lits) {
  uint DEBUG = 0;
  uint k;
  FOR1D(lits, k) {
    if (lits(k)->atom->pred->name == "table")
      break;
  }
  if (lits.N == k) // --> only for desktop world
    return;
  if (DEBUG>0) {cout << "trim_hack before: [N="<<lits.N<<"] "; TL::write(lits); cout<<endl;}
  
  uint __TABLE_PRED_ID = 100;
  uint __ON_PRED_ID = 100;
  uint __TABLE_OBJECT_ID = 60; // ATTENTION: this has to be ensured!
  
  FOR1D(lits, k) {
    if (lits(k)->atom->pred->name == "table") {
      __TABLE_PRED_ID = lits(k)->atom->pred->id;
    }
    else if (lits(k)->atom->pred->name == "on") {
      __ON_PRED_ID = lits(k)->atom->pred->id;
    }
  }
  if (DEBUG>0) {
    cout<<"Did you adapt the predicate IDs correspondingly to RobotManipulationDomain.h???"<<endl;
    PRINT(__ON_PRED_ID);
    PRINT(__TABLE_PRED_ID);
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
  if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  
  // for X>__TABLE_OBJECT_ID: not table(X) [--> cares only about constants]
  if (__TABLE_PRED_ID < 100) {
    FOR1D_DOWN(lits, i) {
      if (lits(i)->atom->pred->id == __TABLE_PRED_ID) { // table
        if (lits(i)->atom->args(0) > __TABLE_OBJECT_ID) {
          if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
            lits.remove(i);
        }
      }
    }
    if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  }
  
  // if on(X,Y) then -on(Y,X) is redundant
  if (__ON_PRED_ID < 100) {
    FOR1D_DOWN(lits, i) {
      if (!lits(i)->positive) {
        if (lits(i)->atom->pred->id == __ON_PRED_ID // on
            && lits(i)->atom->pred->type == TL::Predicate::predicate_simple) {
          uint k;
          bool remove = false;
          FOR1D_DOWN(lits, k) {
            if (lits(k)->positive) {
              if (lits(k)->atom->pred->id == __ON_PRED_ID // on
                    && lits(k)->atom->pred->type == TL::Predicate::predicate_simple) { // primitive
                if (lits(i)->atom->args(0) == lits(k)->atom->args(1)
                    &&     lits(i)->atom->args(1) == lits(k)->atom->args(0)) {
                  remove = true;
                  break;
                }
              }
            }
          }
          if (remove) {
            if (DEBUG>0) {cout<<"Removing ";  lits(i)->write(cout);cout<<endl;}
            lits.remove(i);
          }
        }
      }
    }
    if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  }
  
  // sort out specific concepts
  filter_language(lits);
  if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  
  if (DEBUG>0) {cout << "trim_hack after: [N="<<lits.N<<"] "; TL::write(lits); cout<<endl;}
}


// Algorithm Pasula et al. (2007) p. 330
TL::Rule* ExplainExperiences::explainExperience_deictic(SymbolicExperience* ex) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "explainExperience_deictic [START]" << endl;
  if (DEBUG>1) ex->write(cout);
  uint i, k;
  TL::Rule* newRule = new TL::Rule;
  
  // ensure that all complex are derived
  logicReasoning::derive(&ex->pre);
  logicReasoning::derive(&ex->post);
  LitL context_candidates;
  uintA arguments, arguments_mustBeContained;
  
  // Step 1.1: Create an action and context
  // create action
  TL::Substitution invSub;
  logicReasoning::createInverseSubstitution(*(ex->action), invSub);
  newRule->action = logicReasoning::applyOriginalSub(invSub, ex->action);
  if (DEBUG>1) {cout<<"New action: ";newRule->action->write(cout);cout<<endl;}
  // create context
  // create normal literals; first only with action arguments
  invSub.getIns(arguments);
  // (also accounts for negations)
  logicObjectManager::getLiterals(context_candidates, arguments);
  
  // hack -- don't use complex reward-concepts [START]
  context_candidates.memMove = true;
  FOR1D_DOWN(context_candidates, k) {
    if ( context_candidates(k)->atom->pred->id > 43
      || context_candidates(k)->atom->pred->id == 18  // homies
      )
      context_candidates.remove(k);
  }
  // hack -- don't use complex reward-concepts [END]
  
  if (DEBUG>2) {cout<<"Context literal candidates (based on action arguments, w./o. comparisons): ";TL::write(context_candidates);cout<<endl;}
  FOR1D(context_candidates, i) {
    if (logicReasoning::holds(ex->pre, context_candidates(i))) {
      if (DEBUG>3) {cout<<"Accepting context_candidates(i="<<i<<")="<<*context_candidates(i)<<endl;}
      newRule->context.append(logicReasoning::applyOriginalSub(invSub, context_candidates(i)));
    }
    else {
      if (DEBUG>3) {cout<<"Rejecting context_candidates(i="<<i<<")="<<*context_candidates(i)<<endl;}
    }
  }
  
  // create comparison literals
  LitL equalityLiterals;
  logicObjectManager::getCompLiterals_constantBound(equalityLiterals, arguments, ex->pre, 0);
  
  // HACK -- don't use complex reward-concepts [START]
  equalityLiterals.memMove = true;
  FOR1D_DOWN(equalityLiterals, k) {
    ComparisonLiteral* clit = (ComparisonLiteral*) equalityLiterals(k);
    if (((ComparisonAtom*)clit->atom)->fa1->f->category >= category_derived)
      equalityLiterals.remove(k);
  }
  // HACK -- don't use complex reward-concepts [END]
  
  FOR1D(equalityLiterals, i) {
      newRule->context.append(logicReasoning::applyOriginalSub(invSub, equalityLiterals(i)));
  }

  // order by positives first
  TL::logicReasoning::sort(newRule->context);
  if (DEBUG>1) {cout<<"Context (preliminary): ";TL::write(newRule->context);cout<<endl;}
  
  // Step 1.2: Create deictic references and their literals
  if (DEBUG > 2) {
    cout << "ex->del: "; TL::write(ex->del); cout << endl;
    cout << "ex->add: "; TL::write(ex->add); cout << endl;
    PRINT(ex->changedConstants)
  }
  LitL newContext;
  FOR1D(ex->changedConstants, i) {
    if (!invSub.hasSubs(ex->changedConstants(i))) {
      if (DEBUG>1) {cout<<"Deictic candidate: "<<ex->changedConstants(i)<<endl;}
      TL::Substitution newInvSub = invSub;
      TL::Substitution sub;
      invSub.getInverse(sub);
      newInvSub.addSubs2Variable(ex->changedConstants(i));
      newInvSub.getIns(arguments);
      arguments_mustBeContained.clear();
      arguments_mustBeContained.append(ex->changedConstants(i));
      // create normal predicates (also accounts for negations)
      logicObjectManager::getLiterals(context_candidates, arguments, arguments_mustBeContained);
      
      // hack -- don't use complex reward-concepts [START]
      context_candidates.memMove = true;
      FOR1D_DOWN(context_candidates, k) {
        if ( context_candidates(k)->atom->pred->id > 43
          || context_candidates(k)->atom->pred->id == 18  // homies
          )
          context_candidates.remove(k);
      }
      // hack -- don't use complex reward-concepts [END]
      
      // create constant-bound comparison literals
      LitL equalityLiterals;
      uintA changedConstantWrapper;
      changedConstantWrapper.append(ex->changedConstants(i));
      logicObjectManager::getCompLiterals_constantBound(equalityLiterals, changedConstantWrapper, ex->pre, 0);
      
      // hack -- don't use complex reward-concepts [START]
      equalityLiterals.memMove = true;
      FOR1D_DOWN(equalityLiterals, k) {
        ComparisonLiteral* clit = (ComparisonLiteral*) equalityLiterals(k);
        if (((ComparisonAtom*)clit->atom)->fa1->f->category >= category_derived)
          equalityLiterals.remove(k);
      }
      // hack -- don't use complex reward-concepts [END]
      
      context_candidates.append(equalityLiterals);
      if (comparingValues) {
        // create dynamic-bound comparison literals
        uintA vars_sofar;
        newInvSub.getIns(vars_sofar);
        LitL dynamic_comparisons;
        logicObjectManager::getCompLiterals_dynamicBound(dynamic_comparisons, vars_sofar, ex->pre, 0);
        if (DEBUG>3) {cout<<"Dynamic-bound comparison literals:  ";  TL::write(dynamic_comparisons); cout<<endl;}
        FOR1D(dynamic_comparisons, k) {
          // hack -- don't use complex reward-concepts [START]
          if (((ComparisonAtom*) dynamic_comparisons(k)->atom)->fa1->f->category >= category_derived)
            continue;
          if (dynamic_comparisons(k)->atom->args.findValue(ex->changedConstants(i)) >= 0) {
            context_candidates.append(dynamic_comparisons(k));
          }
        }
      }
      if (DEBUG>2) {cout<<"Context literal candidates (based on deictic candidate): ";TL::write(context_candidates);cout<<endl;}
      // create possible newContext
      newContext = newRule->context;
      FOR1D(context_candidates, k) {
        if (DEBUG>3) {cout<<"context_candidates(k)="<<*context_candidates(k)<<" accepted? ";}
        if (logicReasoning::holds(ex->pre, context_candidates(k))) {
          newContext.append(logicReasoning::applyOriginalSub(newInvSub, context_candidates(k)));
          if (DEBUG>3) {cout<<" yes"<<endl;}
        }
        else {
          if (DEBUG>3) {cout<<" no"<<endl;}
        }
      }
      TL::logicReasoning::sort(newContext);
      if (DEBUG>1) {cout<<"Context with deic ref (potential): ";TL::write(newContext);cout<<endl;}
      // trim literals
      trim_hack(newContext);
      if (slimContext)
        logicReasoning::killBaseConcepts(newContext);
      if (DEBUG>0) {
        cout << "newContext after trimming: ";
        TL::write(newContext);
        cout << endl;
      }
      // check whether new variables refers uniquely to s
      TL::SubstitutionSet subs;
      // check whether truly deictic ref (only one sub)
      bool covers = logicReasoning::cover(ex->pre, newContext, subs, true, &sub);
      if (covers && subs.num()==1) {
        // check for neg free DRs
        TL::Rule helper_rule;
        helper_rule.action = newRule->action;
        helper_rule.context = newContext;
        uintA negFreeDRs;
        TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, helper_rule);
        if (negFreeDRs.N == 0) {
          invSub = newInvSub;
          newRule->context = newContext;
          if (DEBUG>1) {cout<<"Accepted ("<<ex->changedConstants(i)<<")"<<endl;}
        }
        else {
          if (DEBUG>1) {cout << "Not accepted ("<<ex->changedConstants(i)<<"):  negFreeDRs=" << negFreeDRs << endl;}
        }
      }
      else {
        if (DEBUG>1) {cout << "Not accepted ("<<ex->changedConstants(i)<<"):  covers="<<covers<<"  subs.num()=" << (subs.num()) << "=/=1"<< endl;}
      }
    }
  }

  if (DEBUG>0) {
    newRule->write(cout);
  }
  if (DEBUG>0) cout << "explainExperience_deictic [END]" << endl;
  
  return newRule;
}




TL::Rule* ExplainExperiences::explainExperience_deictic_ALL_DRs(SymbolicExperience* ex) {
  uint DEBUG = 0;
  if (DEBUG>0) cout << "explainExperience_deictic_ALL_DRs [START]" << endl;
  if (DEBUG>1) ex->write(cout);
  uint i, k;
  TL::Rule* newRule = new TL::Rule;
  
  // ensure that all complex are derived
  logicReasoning::derive(&ex->pre);
  logicReasoning::derive(&ex->post);
  
  // Step 1.1: Create an action and context
  // create action
  TL::Substitution invSub;
  logicReasoning::createInverseSubstitution(*(ex->action), invSub);
  newRule->action = logicReasoning::applyOriginalSub(invSub, ex->action);
  if (DEBUG>1) {cout<<"New action: ";newRule->action->write(cout);cout<<endl;}
  // create context
 
  // Step 1.2: Create deictic references and their literals
  if (DEBUG > 2) {
    cout << "ex->del: "; TL::write(ex->del); cout << endl;
    cout << "ex->add: "; TL::write(ex->add); cout << endl;
    PRINT(ex->changedConstants)
  }
//   
  TL::Substitution newInvSub = invSub;
  TL::Substitution sub_action;
  invSub.getInverse(sub_action);
  if (DEBUG>0) {
    cout<<"invSub:  "; invSub.write();  cout<<endl;
    cout<<"newInvSub: "; newInvSub.write();  cout<<endl;
    cout<<"sub_action: "; sub_action.write();  cout<<endl;
  }
  
  FOR1D(ex->changedConstants, i) {
    if (newInvSub.hasSubs(ex->changedConstants(i))) continue;
    newInvSub.addSubs2Variable(ex->changedConstants(i));
//     cout<<"newInvSub: "; newInvSub.write();  cout<<endl;
  }
  
  uintA context_vars;  context_vars.setAppend(ex->action->args);  context_vars.setAppend(ex->changedConstants);
  TL::sort_asc(context_vars);
  if (DEBUG>1) {PRINT(context_vars);}
  LitL grounded_context_candidates;
  logicObjectManager::getLiterals(grounded_context_candidates, context_vars);
  LitL newContext;
  LitL newContext_grounded;
  FOR1D(grounded_context_candidates, k) {
    if (DEBUG>3) {cout<<"grounded_context_candidates(k)="<<*grounded_context_candidates(k)<<" accepted? ";}
    if (logicReasoning::holds(ex->pre, grounded_context_candidates(k))) {
      newContext.append(logicReasoning::applyOriginalSub(newInvSub, grounded_context_candidates(k)));
      newContext_grounded.append(grounded_context_candidates(k));
      if (DEBUG>3) {cout<<" yes"<<endl;}
    }
    else {
      if (DEBUG>3) {cout<<" no"<<endl;}
    }
  }
  TL::logicReasoning::sort(newContext_grounded);
  TL::logicReasoning::sort(newContext);
      
  if (DEBUG>0) {PRINT(newContext_grounded);  PRINT(newContext);}

  // check whether new variables refers uniquely to s
  TL::SubstitutionSet subs;
  // check whether truly deictic ref (only one sub)
  bool covers = logicReasoning::cover(ex->pre, newContext, subs, true, &sub_action);
  if (covers && subs.num()==1) {
    // check for neg free DRs
    TL::Rule helper_rule;
    helper_rule.action = newRule->action;
    helper_rule.context = newContext;
    uintA negFreeDRs;
    TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, helper_rule);
    if (negFreeDRs.N == 0) {
      invSub = newInvSub;
      newRule->context = newContext;
      if (DEBUG>1) {cout<<"Rule accepted"<<endl;}
    }
    else {
      if (DEBUG>1) {cout << "Rule not accepted:  negFreeDRs=" << negFreeDRs << endl;}
    }
  }
  else {
    if (DEBUG>1) {
      cout << "Rule not accepted:  covers="<<covers<<"  subs.num()=" << (subs.num()) << "=/=1"<< endl;
      FOR1D_(subs, i) {
        cout<<"["<<i<<"] ";  subs.elem(i)->write();  cout<<endl;
      }
    }
  }
    
  if (DEBUG>0) {newRule->write(cout);}
  if (DEBUG>0) cout << "explainExperience_deictic_ALL_DRs [END]" << endl;
  
  return newRule;
}



void ExplainExperiences::reset() {
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
void DropContextLiterals::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropContextLiterals::findRules [START]"<<endl;
  uint r, p, i;
  TL::Rule* newRule;
  bool stop = false;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextContextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      newRule = new TL::Rule;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
        if (i!=p)
          newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      newRule->action = rulesC_old.rules.elem(r)->action;
      // check for neg free DRs
      uintA negFreeDRs;
      TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, *newRule);
      if (negFreeDRs.N > 0) {
        delete newRule;
        continue;
      }
      // Ensure that new rule covers experiences
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
      if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
        if (DEBUG>3) {
          cout<<"Covered experiences:"<<endl;
          uint k;
          FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
          }
        }
        MT::Array< uintA > experiences_per_outcome;
        induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
        rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
        stop = true;
        nextContextLiteral += 1;
        break;
      }
      else {
        delete newRule;
      }
    }
    if (stop)
      break;
    
    nextRule=r+1;
    nextContextLiteral=0;
  }
  if (DEBUG>0) cout<<"DropContextLiterals::findRules [END]"<<endl;
}


void DropContextLiterals::reset() {
	nextRule = 0;
	nextContextLiteral = 0;
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
void DropContextLiterals_approximativeVersion::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropPre_approx::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint i, k;
  if (prepareTotalNewSearch) {
    usableContextLiterals.clear();
    uint numContextLiterals=0;
    FOR1D_(rulesC_old.rules, i) {
      numContextLiterals += rulesC_old.rules.elem(i)->context.N;
    }
    usableContextLiterals.resize(numContextLiterals);
    usableContextLiterals.setUni(1);
    prepareTotalNewSearch=false;
    if (DEBUG>0) cout<<"Prepared new usableContextLiterals-Array with N="<<usableContextLiterals.N<<endl;
  }
  uint random_contextLiteral, id_contextLiteral, id_rule, starting_id_for_current_rule;
  uint num_open_context;
  TL::Rule* newRule;
  uint resamples=0;
  while (rulesC_2add.rules.num()==0) {
    num_open_context = sum(usableContextLiterals);
    if (num_open_context==0)
        break;
    // randomly choose which context literal to delete
    random_contextLiteral = rnd.num(num_open_context);
    if (DEBUG>1) {cout<<"New rule-finding try:"<<endl; PRINT(num_open_context); PRINT(random_contextLiteral); PRINT(usableContextLiterals);}
    id_contextLiteral=0;
    id_rule=0;
    starting_id_for_current_rule=0;
    // find the correct rule and context for the random_contextLiteral
    FOR1D(usableContextLiterals, k) {
      // If target surely not in current rule "rulesC_old.rules.elem(id_rule)"...
      // Counter for id_rule
      if (k >= rulesC_old.rules.elem(id_rule)->context.N+starting_id_for_current_rule) {
        id_rule++;
        while (rulesC_old.rules.elem(id_rule)->context.N == 0)  // account for empty rules thereafter
          id_rule++;
        starting_id_for_current_rule=k;
      }
      // Counter for id_contextLiteral
      //    found
      if (usableContextLiterals(k) && id_contextLiteral==random_contextLiteral) {
        usableContextLiterals(k)=0;
        id_contextLiteral=k-starting_id_for_current_rule;
        break;
      }
      //   searching
      if (usableContextLiterals(k))
        id_contextLiteral++;
    }
    if (DEBUG>1) {
      cout<<"Thinking about deleting in rule "<<id_rule<<" context literal "<<id_contextLiteral<<endl;
      rulesC_old.rules.elem(id_rule)->write(cout);
      cout<<"Context literal of interest:  "; rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)->write(cout); cout<<endl;
    }
    
    // bias for positive
    if (rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)->positive && resamples < DROP_NEGATIVE_BIAS) {
      if (DEBUG>1) {cout<<" (Don't wanna delete positive lit: "; rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)->write(cout); cout<<")"<<endl;}
      usableContextLiterals(k)=1; // set back
      resamples++;
      continue;
    }
    else
      resamples=0;
    newRule = new TL::Rule;
    FOR1D(rulesC_old.rules.elem(id_rule)->context, k) {
      if (k!=id_contextLiteral)
        newRule->context.append(rulesC_old.rules.elem(id_rule)->context(k));
    }
    if (DEBUG>1) {cout<<"Deletion of "<<*rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)<<" executed."<<endl;}
    newRule->action = rulesC_old.rules.elem(id_rule)->action;
    // check for neg free DRs
    uintA negFreeDRs;
    TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, *newRule);
    if (negFreeDRs.N > 0) {
      delete newRule;
      continue;
    }
    // calc experience coverage
    SymbolicExperienceL covered_experiences;
    uintA covered_experiences_ids;
    calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
    if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
    if (covered_experiences.N > 0) {
      if (DEBUG>1) cout<<"+++++ Covers "<<covered_experiences.N<<" experiences " << covered_experiences_ids << " and will be kept."<<endl;
      if (DEBUG>3) {
        cout<<"Covered experiences:"<<endl;
        uint k;
        FOR1D(covered_experiences, k) {
          covered_experiences(k)->write(cout);
        }
      }
      MT::Array< uintA > experiences_per_outcome;
      induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
      rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
    }
    else {
      delete newRule;
    }
  }
  
  if (DEBUG>1) cout<<"# rules found = "<<rulesC_2add.rules.num()<<endl;
  if (DEBUG>0) cout<<"DropPre_approx::findRules [END]"<<endl;
}


void DropContextLiterals_approximativeVersion::reset() {
}


void DropContextLiterals_approximativeVersion::reset_total_approximator() {
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
void DropReferences::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropReferences::findRules [START]"<<endl;
  uint r, p, i;
  bool stop = false;
  TL::Rule* newRule = NULL;
  uintA drefs;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    TL::ruleReasoning::calcDeicticRefs(*rulesC_old.rules.elem(r), drefs);
    for (p=nextReference; p<drefs.N; p++) {
      if (DEBUG>0) {
        cout<<"Removing deictic reference "<<drefs(nextReference)<<" in rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
      }
      newRule = new TL::Rule;
      newRule->action = rulesC_old.rules.elem(r)->action;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
        TL::Literal* lit = rulesC_old.rules.elem(r)->context(i);
        if (DEBUG>4) {PRINT(lit->atom->args);}
        if (lit->atom->pred->type == TL::Predicate::predicate_comparison) {
          ComparisonAtom* ca = (ComparisonAtom*) lit->atom;
          if (ca->fa1->args.findValue(drefs(nextReference)) < 0 ) {
            if (ca->fa2 == NULL  ||  ca->fa2->args.findValue(drefs(nextReference)) < 0 ) {
              newRule->context.append(rulesC_old.rules.elem(r)->context(i));
            }
          }
        }
        else if (lit->atom->args.findValue(drefs(nextReference))<0)
          newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      if (DEBUG>0) {cout<<"Yielding the new context: "; TL::write(newRule->context); cout<<endl;}
      // check for neg free DRs
      uintA negFreeDRs;
      TL::ruleReasoning::getNegFreeDeicticRefs(negFreeDRs, *newRule);
      if (negFreeDRs.N > 0) {
        delete newRule;
        continue;
      }
      // calc experience coverage
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
      if (covered_experiences.N > 0) {
        if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
        if (DEBUG>3) {
          cout<<"Covered experiences:"<<endl;
          uint k;
          FOR1D(covered_experiences, k) {
            covered_experiences(k)->write(cout);
          }
        }
        MT::Array< uintA > experiences_per_outcome;
        induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
        rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
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
  if (DEBUG>0) {if (newRule!=NULL) {cout<<"Found rule: ";newRule->write(cout);cout<<endl;}}
  
  if (DEBUG>0) cout<<"DropReferences::findRules [END]"<<endl;
}


void DropReferences::reset() {
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

void DropRules::createRuleSets(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, 
          MT::Array< TL::RuleSetContainer >& one_rulesC_new) {
  uint DEBUG = 0;
  one_rulesC_new.clear();
  uint i, j;
  if (DEBUG>0) {cout<<"Old rule-set:"<<endl;  rulesC_old.writeNice(cout, true); cout<<endl<<endl;}
  for (i=1; i<rulesC_old.rules.num(); i++) { // default rule must always be in
    if (DEBUG>0) {cout<<"Dropping rule #" << i << endl;}
    TL::RuleSetContainer rulesC_new(&experiences);
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

void DropRules::reset() {}
void DropRules::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {}











// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    S P L I T   O N   L I T E R A L S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void SplitOnLiterals::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnLiterals::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint r, k;
  TL::Rule* newRule_pos = NULL;
  TL::Rule* newRule_neg = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      TL::ruleReasoning::calcAbsentLiterals(*rulesC_old.rules.elem(r), absentLiterals, true);
      
#if 0
      // TODO - use refs [START]
      uintA terms;
      TL::ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), terms);
      uint newVar = 0;
      FOR1D(terms, k)
        if (terms(k) == newVar)
          newVar++;
      uintA arguments;
      arguments.append(terms);
      arguments.append(newVar);
      uintA wrapper;
      wrapper.append(newVar);
      LitL lits_dr;
      logicObjectManager::getLiterals(lits_dr, arguments, wrapper, true);
      // hack -- don't use complex reward-concepts [START]
      lits_dr.memMove = true;
      FOR1D_DOWN(lits_dr, k) {
        if ( lits_dr(k)->atom->pred->id > 43
          || lits_dr(k)->atom->pred->id == 18  // homies
          )
          lits_dr.remove(k);
      }
      FOR1D_DOWN(lits_dr, k) {
        if (lits_dr(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
          ComparisonLiteral* clit = (ComparisonLiteral*) lits_dr(k);
          if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived)
            lits_dr.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated restriction literals for rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Restriction literals: ";TL::write(lits_dr);cout<<endl;
      }
      
      absentLiterals.append(lits_dr);
      // TODO - use refs [END]
#endif
      
      // hack -- don't use complex reward-concepts [START]
      absentLiterals.memMove = true;
      TL::Predicate* p_HOMIES = logicObjectManager::getPredicate(MT::String("homies"));
      if (p_HOMIES != NULL) {
        FOR1D_DOWN(absentLiterals, k) {
          if (absentLiterals(k)->atom->pred == p_HOMIES)
            absentLiterals.remove(k);
        }
      }
      FOR1D_DOWN(absentLiterals, k) {
        if (absentLiterals(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
          ComparisonLiteral* clit = (ComparisonLiteral*) absentLiterals(k);
          if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived)
            absentLiterals.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated absent literals for rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Absent literals: ";TL::write(absentLiterals);cout<<endl;
      }
    }
    while (nextLiteral<absentLiterals.N) {
      // positive version
      LitL wrapper;
      wrapper.append(absentLiterals(nextLiteral));
      if (DEBUG>1) {
        cout<<"Trying to insert ";absentLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        TL::write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && !logicReasoning::containsLiteral(rulesC_old.rules.elem(r)->context, *absentLiterals(nextLiteral))) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_pos = new TL::Rule;
        newRule_pos->action = rulesC_old.rules.elem(r)->action;
        newRule_pos->context = rulesC_old.rules.elem(r)->context;
        TL::ruleReasoning::insert(*newRule_pos, *absentLiterals(nextLiteral));
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage(covered_experiences, covered_experiences_ids, newRule_pos, experiences);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
          MT::Array< uintA > experiences_per_outcome;
          induceOutcomes(newRule_pos, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
          rulesC_2add.append(newRule_pos, covered_experiences_ids, experiences_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 experiences and will be dropped."<<endl;
          delete newRule_pos;
        }
      }
      else {
        if (DEBUG>1) cout<<" --> Impossible."<<endl;
      }
      // negative version
      TL::Literal* nextLiteral_neg = logicObjectManager::getLiteralNeg(absentLiterals(nextLiteral));
      wrapper.clear();
      wrapper.append(nextLiteral_neg);
      if (DEBUG>1) {
        cout<<"Trying to insert ";nextLiteral_neg->write(cout);cout<<"   into   ";
        TL::write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && !logicReasoning::containsLiteral(rulesC_old.rules.elem(r)->context, *nextLiteral_neg)) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_neg = new TL::Rule;
        newRule_neg->action = rulesC_old.rules.elem(r)->action;
        newRule_neg->context = rulesC_old.rules.elem(r)->context;
        TL::ruleReasoning::insert(*newRule_neg, *nextLiteral_neg);
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage(covered_experiences, covered_experiences_ids, newRule_neg, experiences);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) {cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;}
          if (DEBUG>3) {
            cout<<"Covered experiences:"<<endl;
            uint k;
            FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
            }
          }
          MT::Array< uintA > experiences_per_outcome;
          induceOutcomes(newRule_neg, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
          rulesC_2add.append(newRule_neg, covered_experiences_ids, experiences_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 experiences and will be dropped."<<endl;
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
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {TL::write(rulesC_2add.rules);}}
  if (DEBUG>0) cout<<"SplitOnLiterals::findRules [END]"<<endl;
}

void SplitOnLiterals::reset() {
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

void AddLiterals::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"AddLits::findRules [START]"<<endl;
  uint r;
  TL::Rule* newRule = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      TL::ruleReasoning::calcAbsentLiterals(*rulesC_old.rules.elem(r), absentLiterals);
      // hack -- don't use complex reward-concepts [START]
      absentLiterals.memMove = true;
      uint k;
      FOR1D_DOWN(absentLiterals, k) {
        if ( absentLiterals(k)->atom->pred->id > 43
          || absentLiterals(k)->atom->pred->id == 18  // homies
          )
          absentLiterals.remove(k);
      }
      FOR1D_DOWN(absentLiterals, k) {
        if (absentLiterals(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
          ComparisonLiteral* clit = (ComparisonLiteral*) absentLiterals(k);
          if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived)
            absentLiterals.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated absent literals for rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Absent literals: ";TL::write(absentLiterals);cout<<endl;
      }
    }
    while (nextLiteral<absentLiterals.N) {
      LitL wrapper;
      wrapper.append(absentLiterals(nextLiteral));
      if (DEBUG>1) {
        cout<<"Trying to insert ";absentLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        TL::write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (logicReasoning::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule = new TL::Rule;
        newRule->action = rulesC_old.rules.elem(r)->action;
        newRule->context = rulesC_old.rules.elem(r)->context;
        TL::ruleReasoning::insert(*newRule, *absentLiterals(nextLiteral));
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
          if (DEBUG>3) {
            cout<<"Covered experiences:"<<endl;
            uint k;
            FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
            }
          }
          MT::Array< uintA > experiences_per_outcome;
          induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
          rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
        }
        else {
          if (DEBUG>1) cout<<"Covers 0 experiences and will be dropped."<<endl;
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


void AddLiterals::reset() {
	nextRule = 1;
	nextLiteral = 0;
}









// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    A D D   R E F E R E N C E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void AddReferences::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"AddReferences::findRules [START]"<<endl;
  uint r, i;
  TL::Rule* newRule = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (restrictionLiterals.N == 0) { // first round
      uintA terms;
      TL::ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), terms);
      uint newVar = 0;
      FOR1D(terms, i)
        if (terms(i) == newVar)
          newVar++;
      uintA arguments;
      arguments.append(terms);
      arguments.append(newVar);
      uintA wrapper;
      wrapper.append(newVar);
      logicObjectManager::getLiterals(restrictionLiterals, arguments, wrapper, true);
      // hack -- don't use complex reward-concepts [START]
      restrictionLiterals.memMove = true;
      uint k;
      FOR1D_DOWN(restrictionLiterals, k) {
        if ( restrictionLiterals(k)->atom->pred->id > 43
          || restrictionLiterals(k)->atom->pred->id == 18  // homies
          )
          restrictionLiterals.remove(k);
      }
      FOR1D_DOWN(restrictionLiterals, k) {
        if (restrictionLiterals(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
          ComparisonLiteral* clit = (ComparisonLiteral*) restrictionLiterals(k);
          if (((ComparisonAtom*)clit->atom)->fa1->f->category == category_derived)
            restrictionLiterals.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated restriction literals for rule "<<rulesC_old.experiences_per_rule(r)<<":"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Restriction literals: ";TL::write(restrictionLiterals);cout<<endl;
      }
    }
    while (nextLiteral<restrictionLiterals.N) {
      if (DEBUG>1) {
        cout<<"Inserting ";restrictionLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        TL::write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      newRule = new TL::Rule;
      newRule->action = rulesC_old.rules.elem(r)->action;
      newRule->context = rulesC_old.rules.elem(r)->context;
      TL::ruleReasoning::insert(*newRule, *restrictionLiterals(nextLiteral));
      SymbolicExperienceL covered_experiences;
      uintA covered_experiences_ids;
      calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
      if (covered_experiences.N > 0) {
        if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences "<<covered_experiences_ids<<" and will be kept."<<endl;
        if (DEBUG>3) {
          cout<<"Covered experiences:"<<endl;
          uint k;
          FOR1D(covered_experiences, k) {
            covered_experiences(k)->write(cout);
          }
        }
        MT::Array< uintA > experiences_per_outcome;
        induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
        rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
      }
      else {
        if (DEBUG>1) cout<<"Covers 0 experiences and will be dropped."<<endl;
        delete newRule;
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
      restrictionLiterals.clear();
    }
  }
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {newRule->write(cout);}}
  if (DEBUG>0) cout<<"AddReferences::findRules [END]"<<endl;
}

void AddReferences::reset() {
  nextRule = 1;
  nextLiteral = 0;
}















// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    G E N E R A L I Z E   E Q U A L I T Y
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void GeneralizeEquality::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"GeneralizeEquality::findRules [START]"<<endl;
  uint r, p, i;
  TL::Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (logicReasoning::isEqualityLiteral(rulesC_old.rules.elem(r)->context(p))) {
        if (DEBUG>1) {cout<<"Generalizing literal #"<<p<<": ";rulesC_old.rules.elem(r)->context(p)->write(cout);cout<<endl;}
        newRule = new TL::Rule;
        newRule->action = rulesC_old.rules.elem(r)->action;
        FOR1D(rulesC_old.rules.elem(r)->context, i) {
          if (i!=p) {
            newRule->context.append(rulesC_old.rules.elem(r)->context(i));
          }
          else {
            // create new context literal
            TL::ComparisonLiteral* clit_old = (TL::ComparisonLiteral*) rulesC_old.rules.elem(r)->context(p);
            TL::ComparisonAtom* ca_old = (ComparisonAtom*) clit_old->atom;
            TL::ComparisonLiteral* clit_new = NULL;
            if (!doneLess) {
              doneLess = true; // try greater-equal thereafter
              nextLiteral = p;
              if (clit_old->hasConstantBound()) {
                clit_new = logicObjectManager::getCompLiteral_constant(ca_old->fa1->f, comparison_lessEqual, ca_old->bound, ca_old->fa1->args);
              }
              else {
                clit_new = logicObjectManager::getCompLiteral_dynamic(ca_old->fa1->f, comparison_lessEqual, ca_old->fa1->args, ca_old->fa2->args);
              }
            }
            else {
              doneLess=false;
              nextLiteral = p+1; // try next equality literal thereafter
              if (clit_old->hasConstantBound()) {
                clit_new = logicObjectManager::getCompLiteral_constant(ca_old->fa1->f, comparison_greaterEqual, ca_old->bound, ca_old->fa1->args);
              }
              else {
                clit_new = logicObjectManager::getCompLiteral_dynamic(ca_old->fa1->f, comparison_greaterEqual, ca_old->fa1->args, ca_old->fa2->args);
              }
            }
            if (DEBUG>1) {cout<<"Generalized literal: "<<*clit_new<<endl;}
            newRule->context.append(clit_new);
          }
        }
        TL::logicReasoning::sort(newRule->context);
        if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
        SymbolicExperienceL covered_experiences;
        uintA covered_experiences_ids;
        calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
          if (DEBUG>3) {
            cout<<"Covered experiences:"<<endl;
            uint k;
            FOR1D(covered_experiences, k) {
              covered_experiences(k)->write(cout);
            }
          }
          MT::Array< uintA > experiences_per_outcome;
          induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
          rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
          break;
        }
        else {
          delete newRule;
        }
      }
    }
    if (rulesC_2add.rules.num()>0) {
      break;
    }
    else {
      nextRule=r+1;
      nextLiteral=0;
      doneLess=false;
    }
  }
  if (DEBUG>0) cout<<"GeneralizeEquality::findRules [END]"<<endl;
}


void GeneralizeEquality::reset() {
    nextRule = 1;
    nextLiteral = 0;
    doneLess= false;
}












// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    S P L I T   O N   E Q U A L I T I E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

void SplitOnEqualities::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnEqualities::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint r, v, f, i;
  
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (vars.N == 0) { // first round
      // determine vars
      TL::ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), vars);
      if (DEBUG>2) {
        cout << "Inspecting rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"findValue the following vars: "<<vars<<endl;
      }
    }
    for (v=nextVar; v<vars.d0; v++) {
      for (f=nextFunc; f<usedFunctions.d0; f++) {
        if (usedFunctions(f)->d != 1)  // nur fuer unary functions!!
          continue;
        if (DEBUG>1) {cout<<"Checking var "<<v<<" with function "<<usedFunctions(f)->name<<endl;}
        // check whether function has not been used with this variable
        bool alreadyUsed = false;
        FOR1D(rulesC_old.rules.elem(r)->context, i) {
          if (rulesC_old.rules.elem(r)->context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
            TL::ComparisonLiteral* clit = dynamic_cast<TL::ComparisonLiteral*>(rulesC_old.rules.elem(r)->context(i));
            if (clit==NULL) {HALT("cast failed: clit="<<*rulesC_old.rules.elem(r)->context(i));}
            if (((ComparisonAtom*)clit->atom)->fa1->f->id == usedFunctions(f)->id) {
              if (((ComparisonAtom*)clit->atom)->fa1->args.findValue(vars(v)) >= 0)
                alreadyUsed = true;
            }
          }
        }
        if (DEBUG>1) PRINT(alreadyUsed)
        if (!alreadyUsed) {
          arr usedValues;
          logicReasoning::usedValues(*(usedFunctions(f)), experiences(0)->pre, usedValues); // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          if (DEBUG>2) {cout<<"Using function values: "<<usedValues<<endl;}
          FOR1D(usedValues, i) {
            TL::Rule* newRule = new TL::Rule;
            newRule->action = rulesC_old.rules.elem(r)->action;
            newRule->context = rulesC_old.rules.elem(r)->context;
            uintA eq_args;
            eq_args.append(vars(v));
            TL::ComparisonLiteral* eq = logicObjectManager::getCompLiteral_constant(usedFunctions(f), comparison_equal, usedValues(i), eq_args);
            TL::ruleReasoning::insert(*newRule, *eq);
            if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
            SymbolicExperienceL covered_experiences;
            uintA covered_experiences_ids;
            calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
            if (covered_experiences.N > 0) {
              if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
              MT::Array< uintA > experiences_per_outcome;
              induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
              rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
            }
            else {
              if (DEBUG>1) cout<<"Covers 0 experiences and will be dropped."<<endl;
              delete newRule;
            }
          }
        }
        if (rulesC_2add.rules.num() > 0) {
            nextFunc = f+1;
            break;
        }
      }
      if (rulesC_2add.rules.num() > 0) {
        break;
      }
      else {
        nextFunc = 0;
        nextVar = v+1;
      }
    }
    
    if (rulesC_2add.rules.num() > 0)
      break;
    else {
      nextRule = r+1;
      nextVar = 0;
      nextFunc = 0;
      vars.clear();
    }
  }

  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {TL::write(rulesC_2add.rules);}}
  if (DEBUG>0) cout<<"SplitOnEqualities::findRules [END]"<<endl;
}

void SplitOnEqualities::reset() {
  nextRule = 1;
  nextVar = 0;
  nextFunc = 0;
}







// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    C H A N G E   R A N G E
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
// only for CONSTANT-bound comparison predicates
void ChangeRange::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"ChangeRange::findRules [START]"<<endl;
  uint r, p, v;
  TL::Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (rulesC_old.rules.elem(r)->context(p)->atom->pred->type == TL::Predicate::predicate_comparison) {
        TL::ComparisonLiteral* clit = dynamic_cast<TL::ComparisonLiteral*>(rulesC_old.rules.elem(r)->context(p));
        if (!((ComparisonAtom*)clit->atom)->hasConstantBound()) break;
        CHECK(clit!=NULL, "Cast failed");
        if (DEBUG>1) {cout<<"Changing range of literal #"<<p<<": ";clit->write(cout);cout<<endl;}
        if (nextPossibleValue == 0) {
          // collect possibleValues 
          logicReasoning::usedValues(*(((ComparisonAtom*)clit->atom)->fa1->f), experiences(0)->pre, possibleValues); // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          possibleValues.removeValueSafe(((ComparisonAtom*)clit->atom)->bound);
        }
        if (DEBUG>2) {cout<<"Using values: "<<possibleValues<<endl;}
        
        for (v=nextPossibleValue; v<possibleValues.d0; v++) {
          newRule = new TL::Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          uint n;
          FOR1D(rulesC_old.rules.elem(r)->context, n) {
            if (n!=p) {
              newRule->context.append(rulesC_old.rules.elem(r)->context(n));
            }
            else {
              // create new context literal
              TL::ComparisonLiteral* eq = logicObjectManager::getCompLiteral_constant(((ComparisonAtom*)clit->atom)->fa1->f, ((ComparisonAtom*)clit->atom)->comparisonType, possibleValues(v), ((ComparisonAtom*)clit->atom)->fa1->args);
              TL::ruleReasoning::insert(*newRule, *eq);
            }
          }
          if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          SymbolicExperienceL covered_experiences;
          uintA covered_experiences_ids;
          calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
          if (covered_experiences.N > 0) {
            if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
            if (DEBUG>3) {
              cout<<"Covered experiences:"<<endl;
              uint k;
              FOR1D(covered_experiences, k) {
                covered_experiences(k)->write(cout);
              }
            }
            MT::Array< uintA > experiences_per_outcome;
            induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
            rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
            nextPossibleValue = v+1;
            break;
          }
          else {
            if (DEBUG>1) cout<<"Covers 0 experiences and will be discarded."<<endl;
            delete newRule;
          }
        } // for values
      }
      if (rulesC_2add.rules.num()>0) {
        break;
      }
      else {
        nextLiteral=p+1;
        nextPossibleValue = 0;
      }
    } // for literals
    if (rulesC_2add.rules.num()>0) {
      nextLiteral = p;
      break;
    }
    else {
      nextRule=r+1;
      nextLiteral=0;
    }
  } // for rules
  if (DEBUG>0) cout<<"ChangeRange::findRules [END]"<<endl;
}


void ChangeRange::reset() {
  nextRule=1; // ignore default rule
  nextLiteral=0;
  nextPossibleValue=0;
}





// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    M A K E   I N T E R V A L
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void MakeInterval::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"MakeInterval::findRules [START]"<<endl;
  uint r, p, v;
  TL::Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (rulesC_old.rules.elem(r)->context(p)->atom->pred->type == TL::Predicate::predicate_comparison) {
        TL::ComparisonLiteral* clit = dynamic_cast<TL::ComparisonLiteral*>(rulesC_old.rules.elem(r)->context(p));
        if (!((ComparisonAtom*)clit->atom)->hasConstantBound()) { // only defined for constant bounds
          continue;
        }
        CHECK(clit!=NULL, "Cast failed")
        // TODO CHECK FOR DOUBLE INTERVALS!!!!!!!!!!!!
        // omit equality literals
        if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_equal)
          continue;
        if (DEBUG>1) {cout<<"Making interval for literal #"<<p<<": ";clit->write(cout);cout<<endl;}
        if (nextPossibleValue == 0) {
          // collect possibleValues 
          logicReasoning::usedValues(*(((ComparisonAtom*)clit->atom)->fa1->f), experiences(0)->pre, possibleValues); // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_less 
              || ((ComparisonAtom*)clit->atom)->comparisonType == comparison_lessEqual) {
            uint pv;
            FOR1D_DOWN(possibleValues, pv) {
              if (possibleValues(pv) >= ((ComparisonAtom*)clit->atom)->bound)
                possibleValues.remove(pv);
            }
          }
          else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_greater 
                    || ((ComparisonAtom*)clit->atom)->comparisonType == comparison_greaterEqual) {
            uint pv;
            FOR1D_DOWN(possibleValues, pv) {
              if (possibleValues(pv) <= ((ComparisonAtom*)clit->atom)->bound)
                possibleValues.remove(pv);
            }
          }
          else {HALT("Don't know this comparison type, digger.");}
        }
        if (DEBUG>2) {cout<<"Using values: "<<possibleValues<<endl;}
        
        for (v=nextPossibleValue; v<possibleValues.d0; v++) {
          newRule = new TL::Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          
          // create new context literal
          ComparisonType newCompType;
          if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_less 
              || ((ComparisonAtom*)clit->atom)->comparisonType == comparison_lessEqual) {
            newCompType = comparison_greaterEqual;
          }
          else if (((ComparisonAtom*)clit->atom)->comparisonType == comparison_greater 
                      || ((ComparisonAtom*)clit->atom)->comparisonType == comparison_greaterEqual) {
            newCompType = comparison_lessEqual;
          }
          else
              HALT("Don't know this comparison type, digger.")
          TL::ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_constant(((ComparisonAtom*)clit->atom)->fa1->f, newCompType, possibleValues(v), ((ComparisonAtom*)clit->atom)->fa1->args);
          TL::ruleReasoning::insert(*newRule, *new_clit);

          if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          SymbolicExperienceL covered_experiences;
          uintA covered_experiences_ids;
          calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
          if (covered_experiences.N > 0) {
            if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
            if (DEBUG>3) {
                cout<<"Covered experiences:"<<endl;
                uint k;
                FOR1D(covered_experiences, k) {
                    covered_experiences(k)->write(cout);
                }
            }
            MT::Array< uintA > experiences_per_outcome;
            induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
            rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
            nextPossibleValue = v+1;
            break;
          }
          else {
            if (DEBUG>1) cout<<"Covers 0 experiences and will be discarded."<<endl;
            delete newRule;
          }
        } // for values
      }
      if (rulesC_2add.rules.num()>0) {
          break;
      }
      else {
          nextLiteral=p+1;
          nextPossibleValue = 0;
      }
    } // for literals
    if (rulesC_2add.rules.num()>0) {
      nextLiteral = p;
      break;
    }
    else {
      nextRule=r+1;
      nextLiteral=0;
    }
  } // for rules
  if (DEBUG>0) cout<<"MakeInterval::findRules [END]"<<endl;
}


void MakeInterval::reset() {
  nextRule=1; // ignore default rule
  nextLiteral=0;
  nextPossibleValue=0;
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    CompareFunctionValues
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void CompareFunctionValues::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"CompareFunctionValues::findRules [START]"<<endl;
  uint r, f, c, t;
  TL::Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (f=nextFunction; f<usedFunctions.d0; f++) {
      // calc new term combos if needed
      if (nextTermCombination==0) {
        termCombos.clear();
        uintA terms;
        TL::ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), terms);
        MT::Array< uintA > termCombos_unfiltered;
        TL::allPossibleLists(termCombos_unfiltered, terms, 2 * usedFunctions(f)->d, false, true);
        // filter such that only terms are only combined once (i.e. [X,Y] and not also [Y,X]
        uint i,j;
        FOR1D(termCombos_unfiltered,i) {
          FOR1D(termCombos_unfiltered(i), j) {
            if (j==0) continue;
            if (termCombos_unfiltered(i)(j-1) > termCombos_unfiltered(i)(j))
              break;
          }
          if (termCombos_unfiltered(i).N == j)
            termCombos.append(termCombos_unfiltered(i));
        }
        if (DEBUG>2) cout<<"termCombos for function "<<usedFunctions(f)->name<<": "<<termCombos<<"   (unfiltered: "<<termCombos_unfiltered<<")"<<endl;
      }
      for (t=nextTermCombination; t<termCombos.d0; t++) {
        for (c=nextComparisonType; c<comparisonTypes.d0; c++) {
          if (c==0) {
              coveredExIDsPerComparisonType.clear();
              if (DEBUG>1) {cout<<"Extending OLD RULE:"<<endl;rulesC_old.rules.elem(r)->write(cout);}
          }
          // FILTER FOR SENSIBLE COMPARISONS THAT DO NOT CONTRADICT OTHER STUFF?
          // add comparison to rule
          newRule = new TL::Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          uintA args1, args2;
          uint u;
          for (u=0; u<termCombos(t).N/2; u++) args1.append(termCombos(t)(u));
          for (u=termCombos(t).N/2; u<termCombos(t).N; u++) args2.append(termCombos(t)(u));
          TL::ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_dynamic(usedFunctions(f), comparisonTypes(c), args1, args2);
          TL::ruleReasoning::insert(*newRule, *new_clit);
          if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          SymbolicExperienceL covered_experiences;
          uintA covered_experiences_ids;
          calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
          if (covered_experiences.N > 0) {
            if (DEBUG>1) {
//               cout<<"Covers "<<coveredExperiences.N<<" experiences and will be kept."<<endl;
//               uintA coveredExperiencesIDs;
//               calcCoverage(newRule, experiences, coveredExperiencesIDs);
//               coveredExIDsPerComparisonType.append(coveredExperiencesIDs);
//               cout<<"Example IDs: "<<coveredExperiencesIDs<<endl;
//               if (DEBUG>3) {
//                 cout<<"Covered experiences:"<<endl;
//                 uint k;
//                 FOR1D(coveredExperiences, k) {
//                   cout<<coveredExperiencesIDs(k)<<":"<<endl;
//                   coveredExperiences(k)->write(cout);
//                 }
//               }
            }
            MT::Array< uintA > experiences_per_outcome;
            induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
            rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
            break;
          }
          else {
            delete newRule;
            if (DEBUG>1) cout<<"Covers 0 experiences and will be discarded."<<endl;
          }
        } // for comparison types
        if (rulesC_2add.rules.num()>0) {
          nextComparisonType = c+1;
          break;
        }
        else {
          nextComparisonType=0;
          if (DEBUG>2) {
//             cout<<"Recap on coverages:"<<endl;
//             SymbolicExperienceL coveredExperiences;
//             calcCoverage(rulesC_old.rules.elem(r), experiences, coveredExperiences);
//             cout<<"Old rule: "<<endl;rulesC_old.rules.elem(r)->write(cout);cout<<" covers "<<coveredExperiences.N<<endl;
//             cout <<"while the new rules with comparisons cover: "<<endl;
//             uint xx;
//             FOR1D(coveredExIDsPerComparisonType, xx) {
//               cout<<"comptype #"<<xx<<"="<<comparisonTypes(xx)<<": "<<coveredExIDsPerComparisonType(xx).d0<<" "<<coveredExIDsPerComparisonType(xx)<<endl;
//             }
            // several tests for correct calculation
            // These tests are intuitive, but wrong!!
//                         CHECK(coveredExIDsPerComparisonType.d0==comparisonTypes.N, "Too many new rules!!!")
//                         CHECK(coveredExIDsPerComparisonType(0).d0+coveredExIDsPerComparisonType(1).d0==coveredExIDsPerComparisonType(2).d0, "equal + less != less-equal!!")
//                         CHECK(coveredExIDsPerComparisonType(0).d0+coveredExIDsPerComparisonType(3).d0==coveredExIDsPerComparisonType(4).d0, "equal + less != less-equal!!")
//                         CHECK(numberSharedElements(coveredExIDsPerComparisonType(0), coveredExIDsPerComparisonType(2)) == coveredExIDsPerComparisonType(0).N, "<= does not subsume ==!")
//                         CHECK(numberSharedElements(coveredExIDsPerComparisonType(0), coveredExIDsPerComparisonType(4)) == coveredExIDsPerComparisonType(0).N, ">= does not subsume ==!")
//                         CHECK(numberSharedElements(coveredExIDsPerComparisonType(1), coveredExIDsPerComparisonType(3)) == 0, "< and > share elements!")
          }
        }
      } // for term combos
      if (rulesC_2add.rules.num()>0) {
        nextTermCombination = t;
        break;
      }
      else {
        nextTermCombination=0;
      }
    } // for functions
    if (rulesC_2add.rules.num()>0) {
        nextFunction = f;
        break;
    }
    else {
        nextRule=r+1;
        nextFunction=0;
    }
  }
  
  if (DEBUG>0) cout<<"CompareFunctionValues::findRules [END]"<<endl;
}


void CompareFunctionValues::reset() {
  nextRule=1; // ignore default rule
  nextFunction=0;
  nextComparisonType=0;
  nextTermCombination=0;
}




// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    SplitOnCompareFunctionValues
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void SplitOnCompareFunctionValues::findRules(const TL::RuleSetContainer& rulesC_old, const SymbolicExperienceL& experiences, TL::RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnCompareFunctionValues::findRules [START]"<<endl;
  uint r, f, c, t;
  TL::Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (f=nextFunction; f<usedFunctions.d0; f++) {
      // calc new term combos if needed
      if (nextTermCombination==0) {
        termCombos.clear();
        uintA terms;
        TL::ruleReasoning::calcTerms(*rulesC_old.rules.elem(r), terms);
        MT::Array< uintA > termCombos_unfiltered;
        TL::allPossibleLists(termCombos_unfiltered, terms, 2 * usedFunctions(f)->d, false, true);
        // filter such that only terms are only combined once (i.e. [X,Y] and not also [Y,X]
        uint i,j;
        FOR1D(termCombos_unfiltered,i) {
          FOR1D(termCombos_unfiltered(i), j) {
            if (j==0) continue;
            if (termCombos_unfiltered(i)(j-1) > termCombos_unfiltered(i)(j))
              break;
          }
          if (termCombos_unfiltered(i).N == j)
            termCombos.append(termCombos_unfiltered(i));
        }
        if (DEBUG>2) cout<<"termCombos for function "<<usedFunctions(f)->name<<": "<<termCombos<<"   (unfiltered: "<<termCombos_unfiltered<<")"<<endl;
      }
      for (t=nextTermCombination; t<termCombos.d0; t++) {
        if (DEBUG>1) {cout<<"Extending OLD RULE:"<<endl;rulesC_old.rules.elem(r)->write(cout);coveredExIDsPerComparisonType.clear();}
        FOR1D(comparisonTypes, c) {
          // add comparison to rule
          newRule = new TL::Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          uintA args1, args2;
          uint u;
          for (u=0; u<termCombos(t).N/2; u++) args1.append(termCombos(t)(u));
          for (u=termCombos(t).N/2; u<termCombos(t).N; u++) args2.append(termCombos(t)(u));
          TL::ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_dynamic(usedFunctions(f), comparisonTypes(c), args1, args2);
          TL::ruleReasoning::insert(*newRule, *new_clit);
          if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          SymbolicExperienceL covered_experiences;
          uintA covered_experiences_ids;
          calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
          if (covered_experiences.N > 0) {
            if (DEBUG>1) {
              cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
              coveredExIDsPerComparisonType.append(covered_experiences_ids);
              cout<<"Example IDs: "<<covered_experiences_ids<<endl;
              if (DEBUG>3) {
                cout<<"Covered experiences:"<<endl;
                uint k;
                FOR1D(covered_experiences, k) {
                  cout<<covered_experiences_ids(k)<<":"<<endl;
                  covered_experiences(k)->write(cout);
                }
              }
            }
            MT::Array< uintA > experiences_per_outcome;
            induceOutcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos, param_opt_type);
            rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
          }
          else {
            delete newRule;
            if (DEBUG>1) cout<<"Covers 0 experiences and will be discarded."<<endl;
          }
        } // for comparison types
        if (rulesC_2add.rules.num()>0) {
          break;
        }
      } // for term combos
      if (rulesC_2add.rules.num()>0) {
          nextTermCombination = t+1;
          break;
      }
      else {
          nextTermCombination=0;
      }
    } // for functions
    if (rulesC_2add.rules.num()>0) {
      nextFunction = f;
      break;
    }
    else {
      nextRule=r+1;
      nextFunction=0;
    }
  }
  
  if (DEBUG>0) cout<<"SplitOnCompareFunctionValues::findRules [END]"<<endl;
}


void SplitOnCompareFunctionValues::reset() {
  nextRule=1; // ignore default rule
  nextFunction=0;
  nextTermCombination=0;
}
































// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    R U L E   L E A R N E R
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


RuleLearner::RuleLearner(double alpha, double p_min, double p_min_noisyDefaultRule, uint so_choice_type) {
  TL::SearchOperator::ProbabilityOptimizationType param_opt_type = TL::SearchOperator::rprop; // fix it to RProp
  
  this->alpha_PEN = alpha;
  this->p_min = p_min;
  this->p_min_noisyDefaultRule = p_min_noisyDefaultRule;
  this->so_choice_type = so_choice_type;
    
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES>0.0) {
      ExplainExperiences* explainEx = new ExplainExperiences(false, false, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES);
  }
  
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM>0.0) {
      ExplainExperiences* explainEx_slim = new ExplainExperiences(true, false, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx_slim);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM);
  }
  
  if (SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM>0.0) {
      ExplainExperiences* explainEx_slim_compare = new ExplainExperiences(true, true, this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(explainEx_slim_compare);
      so_priorWeights.append(SO_WEIGHT__EXPLAIN_EXPERIENCES_SLIM_AND_COMPARING);
  }

  if (SO_WEIGHT__DROP_CONTEXT_LITERALS>0.0) {
      DropContextLiterals* dropPre = new DropContextLiterals(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropPre);
      so_priorWeights.append(SO_WEIGHT__DROP_CONTEXT_LITERALS);
  }
  
  if (SO_WEIGHT__DROP_CONTEXT_LITERALS_APPROX>0.0) {
      DropContextLiterals_approximativeVersion* dropPre_approx = new DropContextLiterals_approximativeVersion(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropPre_approx);
      so_priorWeights.append(SO_WEIGHT__DROP_CONTEXT_LITERALS_APPROX);
  }

  if (SO_WEIGHT__DROP_REFS>0.0) {
      DropReferences* dropRef = new DropReferences(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropRef);
      so_priorWeights.append(SO_WEIGHT__DROP_REFS);
  }
  
  if (SO_WEIGHT__DROP_RULES>0.0) {
      DropRules* dropRules = new DropRules(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(dropRules);
      so_priorWeights.append(SO_WEIGHT__DROP_RULES);
  }

  if (SO_WEIGHT__SPLIT_ON_LITS>0.0) {
      SplitOnLiterals* splitOnLits = new SplitOnLiterals(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(splitOnLits);
      so_priorWeights.append(SO_WEIGHT__SPLIT_ON_LITS);
  }

  if (SO_WEIGHT__ADD_LITS>0.0) {
      AddLiterals* addLit = new AddLiterals(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(addLit);
      so_priorWeights.append(SO_WEIGHT__ADD_LITS);
  }

  if (SO_WEIGHT__ADD_REFS>0.0) {
      AddReferences* addRef = new AddReferences(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(addRef);
      so_priorWeights.append(SO_WEIGHT__ADD_REFS);
  }
  
  if (SO_WEIGHT__SPLIT_ON_EQS>0.0) {
      SplitOnEqualities* splitOnEqs = new SplitOnEqualities(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(splitOnEqs);
      so_priorWeights.append(SO_WEIGHT__SPLIT_ON_EQS);
  }
  
  if (SO_WEIGHT__GENERALIZE_EQS>0.0) {
      GeneralizeEquality* generalizeEq = new GeneralizeEquality(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(generalizeEq);
      so_priorWeights.append(SO_WEIGHT__GENERALIZE_EQS);
  }
  
  if (SO_WEIGHT__CHANGE_RANGE>0.0) {
      ChangeRange* changeRange = new ChangeRange(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(changeRange);
      so_priorWeights.append(SO_WEIGHT__CHANGE_RANGE);
  }
  
  if (SO_WEIGHT__MAKE_INTVL>0.0) {
      MakeInterval* makeIntvl = new MakeInterval(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(makeIntvl);
      so_priorWeights.append(SO_WEIGHT__MAKE_INTVL);
  }
  
  if (SO_WEIGHT__COMPARE_FUNCTIONVALUES>0.0) {
      CompareFunctionValues* compFVs = new CompareFunctionValues(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(compFVs);
      so_priorWeights.append(SO_WEIGHT__COMPARE_FUNCTIONVALUES);
  }
  
  if (SO_WEIGHT__SPLIT_ON_COMPARE_FUNCTIONVALUES>0.0) {
      SplitOnCompareFunctionValues* split_compFVs = new SplitOnCompareFunctionValues(this->alpha_PEN, this->p_min, param_opt_type);
      searchOperators.append(split_compFVs);
      so_priorWeights.append(SO_WEIGHT__SPLIT_ON_COMPARE_FUNCTIONVALUES);
  }
  
  num_so_applied.resize(searchOperators.N);
  num_so_applied.setUni(0);
  num_so_improvements.resize(searchOperators.N);
  num_so_improvements.setUni(0);
  so_improvements.resize(searchOperators.N);
  so_improvements.setUni(0.0);
}




RuleLearner::~RuleLearner() {
  uint i;
  FOR1D(searchOperators, i) {
    delete searchOperators(i);
  }
}



double RuleLearner::score(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, double cutting_threshold) {
  arr experience_weights(experiences.N);
  experience_weights.setUni(1.0);
  return score(rulesC, experiences, cutting_threshold, experience_weights);
}


double RuleLearner::score(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, double cutting_threshold, arr& experience_weights) {
  uint DEBUG = 0;
  if (DEBUG > 0) {cout << "SCORE [start]" << endl;}
  if (DEBUG > 1) {cout<<"RULES:"<<endl;  rulesC.writeNice(); }
  uint i;
  
  // (1) Penalty
  double penalty = 0.0;
  FOR1D_(rulesC.rules, i) {
    penalty += TL::ruleReasoning::numLiterals(*rulesC.rules.elem(i));
  }
  penalty *= alpha_PEN;
  if (DEBUG > 0) {PRINT(alpha_PEN); PRINT(penalty);}
  
  // (2) Log-Likelihood
  double loglikelihood = 0.0, exLik;
  FOR1D(experiences, i) {
    if (DEBUG > 1) {cout << "+++ ex " << i << ": ";  experiences(i)->action->write(cout);  cout<<endl;}
    const uintA& covering_rules = rulesC.nonDefaultRules_per_experience(i);
    if (DEBUG>1) {PRINT(covering_rules);}
    // only one non-default rule covers
    if (covering_rules.N == 1) {
      TL::Rule* rule = rulesC.rules.elem(covering_rules(0));
      if (DEBUG > 2) {
        cout << "Use rule #" << covering_rules(0) << "  " << rulesC.experiences_per_rule(covering_rules(0)) << endl;
        rule->write(cout);
      }
      uint o;
      MT::Array< uintA >& exs_per_out = rulesC.experiences_per_ruleOutcome(covering_rules(0));
      if (DEBUG>2) {PRINT(exs_per_out);}
      exLik = 0.0;
      FOR1D(exs_per_out, o) {
        // Non-noise outcome
        if (o<exs_per_out.N-1  &&  exs_per_out(o).findValue(i) >= 0) {
          exLik += rule->probs(o);
          if (DEBUG>2) {printf("o=%i:  %10.5f\n", o, exLik);}
        }
        // Noise-outcome:  always covers
        if (o == exs_per_out.N-1) {
          exLik += p_min * rule->probs(o);
          if (DEBUG>2) {printf("o=%i:  %10.15f\n", o, exLik);}
        }
      }
      CHECK(exLik>0., "bad referencing  exLik="<<exLik);
    }
    // only default covers or more than one non-default covers
    // --> apply default rule
    else {
      if (DEBUG > 2) {
        cout << " Using default rule." << endl;
      }
      TL::Rule* default_rule = rulesC.rules.elem(0);
      if (rulesC.experiences_per_ruleOutcome(0)(0).findValue(i) >= 0)
        exLik = default_rule->probs(0) * (100.0 * p_min_noisyDefaultRule);   // Avoid modeling persistence by means of noise default rule.
      else 
        exLik = default_rule->probs(1) * p_min_noisyDefaultRule;
    }
    loglikelihood += experience_weights(i) * log(exLik); // weight the experiences differently
    if (DEBUG>1) cout<<" --> lik=" << exLik<<endl;
    if (DEBUG > 0) {PRINT(loglikelihood);  }
  
    
    if (loglikelihood - penalty  <  cutting_threshold) {
      if (DEBUG>0) {
        PRINT(loglikelihood - penalty);
        PRINT(cutting_threshold);
        cout<<"Score becomes too small... giving up."<<endl;
        cout << "SCORE [end]" << endl;
      }
      return TL::TL_DOUBLE_MIN;
    }
  }


  
  if (DEBUG > 0) {
    PRINT(loglikelihood - penalty)
    cout << "SCORE [end]" << endl;
  }
  
  return loglikelihood - penalty;
}



int RuleLearner::chooseNextOperator(boolA& op_applicable) {
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
    HALT("Undefined search operator choice procedure")
  }
  // FINAL
  return op;
}



void RuleLearner::setAlphaPEN(double alpha_PEN) {
  this->alpha_PEN = alpha_PEN;
}


void RuleLearner::set_p_mins(double p_min, double p_min_noisyDefaultRule) {
  this->p_min = p_min;
  this->p_min_noisyDefaultRule = p_min_noisyDefaultRule;
  uint i;
  FOR1D(searchOperators, i) {
    searchOperators(i)->set_p_min(p_min);
  }
}



void RuleLearner::learn_rules(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, const char* logfile) {
  arr experience_weights(experiences.N);
  experience_weights.setUni(1.);
  learn_rules(rulesC, experiences, experience_weights, logfile);
}


// Algorithm in Zettlemoyer's Figure 2
void RuleLearner::learn_rules(TL::RuleSetContainer& rulesC, SymbolicExperienceL& experiences, arr& experience_weights, const char* logfile) {
  uint DEBUG = 0; //  2 ist gut
  rulesC.clear();
  uint i, j;
  FOR1D(experiences, i) {
    TL::logicReasoning::sort(experiences(i)->pre.lits_prim);
  }
  TL::logicReasoning::sort(experiences.last()->post.lits_prim);
  
  rulesC.init(&experiences);
  
  // Init default rule
  rulesC.rules.append(TL::ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();
  bool betterRulesFound = true;
  uint round = 0;
  double bestscore = score(rulesC, experiences, TL::TL_DOUBLE_MIN, experience_weights);
  scores.append(bestscore);
  if (DEBUG > 0) {
    cout << "RuleLearner::learn_rules [START]" << endl;
    PRINT(alpha_PEN);
    PRINT(p_min);
    PRINT(p_min_noisyDefaultRule);
    cout << "Number of training experiences " << experiences.N << endl;
    cout << "Default rule:" << endl;
//     rulesC.rules.elem(0)->write(cout);
    rulesC.writeNice();
    cout << "SCORE = " << bestscore << endl;
  }
  if (DEBUG > 2) {
    cout << "Examples:" << endl;
    FOR1D(experiences, i) {
      cout << "--- [" << i << "] ---" << endl;
      cout<<"PRE-STATE:  "; experiences(i)->pre.write(cout); cout<<endl;
      cout<<"ACTION:     "; experiences(i)->action->write(cout); cout<<endl;
      cout<<"POST-STATE: "; experiences(i)->post.write(cout); cout<<endl;
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
  MT_MSG("RuleLearner: MAX_STEPS = "<<MAX_STEPS);
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
    MT::Array< TL::RuleSetContainer > set_of__rulesC_new;
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
//     if (DEBUG>1) {cout<<"Sanity checks of "<<set_of__rulesC_new.N<<" new rule-sets"<<endl;}
//     FOR1D(set_of__rulesC_new, j) {
//       if (DEBUG>1) {cout<<"Sanity check for new candidate rule-set #"<<j<<endl;}
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
    bool betterRulesFound_thisSearchOperator = false;
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
      betterRulesFound_thisSearchOperator = true;
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
      if (betterRulesFound_thisSearchOperator) {
        cout << "A new best rule-set was found:" << endl;
        rulesC.writeNice();
        cout << "SCORE = " << bestscore << endl;
        rulesC.sanityCheck();  // TODO remove
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
//   rulesC.sanityCheck();
  rulesC.recomputeDefaultRule();
  if (DEBUG>0) {cout<<"Trying to sort rules..."<<endl;}
  rulesC.sort();
 
  if (DEBUG>0) cout<<"Puh, that was it, now I can't find any better rules."<<endl;
	
  // LOGFILE WRITING [start]
  std::ofstream log_info;
  MT::String logfile_info;
  logfile_info << logfile << ".info";
  open(log_info, logfile_info);
  
  log_info<<"--- VOCABULARY ---"<<endl;
  log_info<<"*Primitive predicates*"<<endl;
  TL::writeNice(logicObjectManager::p_prim, log_info);
  log_info<<"*Derived predicates*"<<endl;
  TL::writeNice(logicObjectManager::p_derived, log_info);
  log_info<<"*Primitive functions*"<<endl;
  TL::writeNice(logicObjectManager::f_prim, log_info);
  log_info<<"*Derived functions*"<<endl;
  TL::writeNice(logicObjectManager::f_derived, log_info);
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
          << "  " <<  ((1.0 * num_so_improvements(i))/num_so_applied(i))
          <<"  "<<so_improvements(i);
      if (so_improvements(i)>0.0)log_info<<"  "<<((1.0*so_improvements(i))/num_so_improvements(i));
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

  if (DEBUG > 0) cout << "RuleLearner::learn_rules [END]" << endl;
}


}
