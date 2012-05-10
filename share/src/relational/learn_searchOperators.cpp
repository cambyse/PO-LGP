/*  
    Copyright 2008-2012   Tobias Lang
    
    E-mail:    tobias.lang@fu-berlin.de
    
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


#include <math.h>
#include "reason.h"
#include "learn.h"


namespace relational {
  


/************************************************
 * 
 *     SearchOperator
 * 
 ************************************************/


SearchOperator::SearchOperator() {
  this->approximative = false;
}


void SearchOperator::integrateNewRules(const RuleSetContainer& rulesC_old, const RuleSetContainer& rulesC_2add, 
                                       const StateTransitionL& experiences, RuleSetContainer& rulesC_new) {
  uint DEBUG = 0;
  if (DEBUG > 0) {cout << "integrateNewRules [START]" << endl;}
  if (DEBUG > 1) {
    cout << "***** Really old rules *****" << endl;
    rulesC_old.write(cout, true);
    cout << "***** rulesC_2add *****" << endl;
    rulesC_2add.write(cout, true);
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
    rulesC_2add.rules.elem(i)->cleanup();
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
      rulesC_old.write(cout, true);
    }
    cout << "***** New rules *****" << endl;
    rulesC_new.write(cout, true);
  }
  
  if (DEBUG > 0) {cout << "integrateNewRules [END]" << endl;}
}


const char* SearchOperator::getName() {
  return name;
}


// Algorithm of Figure 4 in Pasula et al. (2007)
void SearchOperator::createRuleSets(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, 
        MT::Array< RuleSetContainer >& set_of_new_rulesC) {
  uint DEBUG = 0;
  set_of_new_rulesC.clear();
  reset();
  uint i;
  while (true) {
    RuleSetContainer rulesC_2add(&experiences);
    // this is where the local knowledge of the individual search operators comes in
    findRules(rulesC_old, experiences, rulesC_2add);
    if (DEBUG>1) {
      FOR1D_(rulesC_2add.rules, i) {
        rulesC_2add.rules.elem(i)->write(cout);
      }
    }
    if (rulesC_2add.rules.num() == 0)
      break;
    RuleSetContainer rulesC_new(&experiences);
    integrateNewRules(rulesC_old, rulesC_2add, experiences, rulesC_new);
    set_of_new_rulesC.append(rulesC_new);
    if (approximative) {
      if (set_of_new_rulesC.d0 >= APPROXIMATOR__RULES_PER_ROUND)
        break;
    }
  }
}















// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    E X P L A I N   E X A M P L E S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


ExplainExperiences::ExplainExperiences(bool slimContext, bool comparingValues) : SearchOperator() {
  if (!slimContext) {
    if (!comparingValues)
      name = "ExplainExperiences";
    else
      name = "ExplainExperiences_comparingValues";
  }
  else {
    if (!comparingValues)
      name = "ExplainExperiences_slim";
    else
      name = "ExplainExperiences_slim_comparingValues";
  }
  nextPotentialExperience = 0;
  this->slimContext = slimContext;
  this->comparingValues = comparingValues;
}


// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void ExplainExperiences::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"ExplainExperiences::findRules [START]"<<endl;
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
      if (DEBUG>0) cout << "findRules: let's explain #" << i << endl;
      if (DEBUG>2) {experiences(i)->write(cout);}
      Rule* newRule = explainExperience(experiences(i));
      // calc experience coverage
      StateTransitionL covered_experiences;
      uintA covered_experiences_ids;
      learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
      // Estimate new outcomes for r'
      CHECK(covered_experiences.N>0, "At least the explained example should be covered.")
      MT::Array< uintA > experiences_per_outcome;
      learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
      if (DEBUG>0) {cout<<"New Rule:"<<endl; newRule->write(cout);}
      if (DEBUG>0) {cout<<"#Covering experiences: "<<covered_experiences.N<<endl;}
      rulesC_2add.append(newRule, covered_experiences_ids, experiences_per_outcome);
//       rulesC_2add.sanityCheck();
      nextPotentialExperience = i+1;
      break;
    }
  }
//  PRINT(rules2_add.num());
//  PRINT(Rule::globalRuleCounter)
	if (DEBUG>0) cout<<"ExplainExperiences::findRules [END]"<<endl;
}


Rule* ExplainExperiences::explainExperience(StateTransition* ex) {
//   return explainExperience_deictic(ex);
  return explainExperience_deictic_ALL_DRs(ex);
}


// TODO Should be replaced by an explicit vocabulary that shall be used for the rules...
void filter_language(LitL& lits) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout << "filter_language before: [N="<<lits.N<<"] "<<lits<<endl;}
  // determine predicates to be filtered
  SymL filtered_symbols;
  uint i;
  FOR1D(lits, i) {
    if (lits(i)->s->name == "above") {
      filtered_symbols.append(lits(i)->s);
    }
  }
  if (DEBUG>0) {PRINT(filtered_symbols);}
  
  lits.memMove = true;
  FOR1D_DOWN(lits, i) {
    if (filtered_symbols.findValue(lits(i)->s) >= 0) {
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
    if (lits(k)->s->name == "table")
      break;
  }
  if (lits.N == k) // --> only for desktop world
    return;
  if (DEBUG>0) {cout << "trim_hack before: [N="<<lits.N<<"] "; write(lits); cout<<endl;}
  
  uint __TABLE_OBJECT_ID = 60; // ATTENTION: this has to be ensured!
  uint i;
  lits.memMove = true;
  
  // no self-referencing binary predicates
  // on(X,X) etc.
  FOR1D_DOWN(lits, i) {
    if (lits(i)->args.N == 2) {
      if (lits(i)->args(0) == lits(i)->args(1)) {
        if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
        lits.remove(i);
      }
    }
  }
  if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  
  // for X>__TABLE_OBJECT_ID: not table(X) [--> cares only about constants]
  Symbol* s_TABLE = Symbol::get(MT::String("table"));
  if (s_TABLE != NULL) {
    FOR1D_DOWN(lits, i) {
      if (lits(i)->s == s_TABLE) { // table
        if (lits(i)->args(0) > __TABLE_OBJECT_ID) {
          if (DEBUG>0) {cout<<"Removing ";lits(i)->write(cout);cout<<endl;}
            lits.remove(i);
        }
      }
    }
    if (DEBUG>1) {cout << "trim_hack now: "<<lits<<endl;}
  }
  
  // if on(X,Y) then -on(Y,X) is redundant
  Symbol* s_ON = Symbol::get(MT::String("on"));
  if (s_ON != NULL) {
    FOR1D_DOWN(lits, i) {
      if (!lits(i)->value > 0.) {
        if (lits(i)->s == s_ON) {
          uint k;
          bool remove = false;
          FOR1D_DOWN(lits, k) {
            if (lits(k)->value > 0.) {
              if (lits(k)->s) { // primitive
                if (lits(i)->args(0) == lits(k)->args(1)
                    &&     lits(i)->args(1) == lits(k)->args(0)) {
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
  
  if (DEBUG>0) {cout << "trim_hack after: [N="<<lits.N<<"] "; write(lits); cout<<endl;}
}


// Algorithm Pasula et al. (2007) p. 330
#if 0
Rule* ExplainExperiences::explainExperience_deictic(StateTransition* ex) {
  NIY;
  // TODO Do we still use this one at all??
  
  uint DEBUG = 3;
  if (DEBUG>0) cout << "explainExperience_deictic [START]" << endl;
  if (DEBUG>1) ex->write(cout);
  uint i, k;
  Rule* newRule = new Rule;
  
  // ensure that all complex are derived
  reason::derive(&ex->pre);
  reason::derive(&ex->post);
  uintA arguments, arguments_mustBeContained;
  
  // Step 1.1: Create an action and context
  // create action
  Substitution invSub;
  FOR1D(ex->action->args, i) {
    if (reason::isConstant(ex->action->args(i)) && !invSub.hasSubs(ex->action->args(i))) {
      invSub.addSubs2Variable(ex->action->args(i));
    }
  }
  newRule->action = invSub.apply(ex->action);
  if (DEBUG>1) {cout<<"New action: ";newRule->action->write(cout);cout<<endl;}
  // create context
  // create normal literals; first only with action arguments
  invSub.getIns(arguments);
  // (also accounts for negations)
  LitL context_candidates, context_candidates_pos, context_candidates_neg;
  Literal::getLiterals_state(context_candidates_pos, arguments, 1.0);
  Literal::getLiterals_state(context_candidates_neg, arguments, 0.0);
  context_candidates.append(context_candidates_pos);
  context_candidates.append(context_candidates_neg);
  
  // hack -- don't use complex reward-concepts [START]
  context_candidates.memMove = true;
  FOR1D_DOWN(context_candidates, k) {
    if ( context_candidates(k)->s->symbol_type == Symbol::function_reward
      || context_candidates(k)->s->name == "homies"
      )
      context_candidates.remove(k);
  }
  // hack -- don't use complex reward-concepts [END]
  
  if (DEBUG>2) {cout<<"Context literal candidates (based on action arguments, w./o. comparisons): ";write(context_candidates);cout<<endl;}
  FOR1D(context_candidates, i) {
    if (reason::holds(ex->pre.lits, context_candidates(i))) {
      if (DEBUG>3) {cout<<"Accepting context_candidates(i="<<i<<")="<<*context_candidates(i)<<endl;}
      newRule->context.append(invSub.apply(context_candidates(i)));
    }
    else {
      if (DEBUG>3) {cout<<"Rejecting context_candidates(i="<<i<<")="<<*context_candidates(i)<<endl;}
    }
  }
  
  // create comparison literals
  LitL equalityLiterals;
  FOR1D(ex->pre.lits, i) {
    if (ex->pre.lits(i)->s->range_type != Symbol::binary)
      equalityLiterals.append(ex->pre.lits(i));
  }
  
  // hack -- don't use complex reward-concepts [START]
  FOR1D_DOWN(equalityLiterals, k) {
    if ( equalityLiterals(k)->s->symbol_type == Symbol::count
      || equalityLiterals(k)->s->symbol_type == Symbol::function_count
      || equalityLiterals(k)->s->symbol_type == Symbol::avg
      || equalityLiterals(k)->s->symbol_type == Symbol::max
      || equalityLiterals(k)->s->symbol_type == Symbol::function_change
      || equalityLiterals(k)->s->symbol_type == Symbol::sum
      || equalityLiterals(k)->s->symbol_type == Symbol::function_reward
    ) {
      equalityLiterals.remove(k);
    }
  }
  // hack -- don't use complex reward-concepts [END]
  
  FOR1D(equalityLiterals, i) {
      newRule->context.append(invSub.apply(equalityLiterals(i)));
  }

  // order by positives first
  Literal::sort(newRule->context);
  if (DEBUG>1) {cout<<"Context (preliminary): ";write(newRule->context);cout<<endl;}
  
  // Step 1.2: Create deictic references and their literals
  if (DEBUG > 2) {
    cout << "ex->del: "; write(ex->del); cout << endl;
    cout << "ex->add: "; write(ex->add); cout << endl;
    PRINT(ex->changedConstants)
  }
  LitL newContext;
  FOR1D(ex->changedConstants, i) {
    if (!invSub.hasSubs(ex->changedConstants(i))) {
      if (DEBUG>1) {cout<<"Deictic candidate: "<<ex->changedConstants(i)<<endl;}
      Substitution newInvSub = invSub;
      Substitution sub;
      invSub.getInverse(sub);
      newInvSub.addSubs2Variable(ex->changedConstants(i));
      newInvSub.getIns(arguments);
      arguments_mustBeContained.clear();
      arguments_mustBeContained.append(ex->changedConstants(i));
      // create normal predicates (also accounts for negations)
      LitL context_candidates__pos, context_candidates__neg;
      Literal::getLiterals_state(context_candidates__pos, arguments, arguments_mustBeContained, 1.0);
      Literal::getLiterals_state(context_candidates__neg, arguments, arguments_mustBeContained, 0.0);
      context_candidates.append(context_candidates__pos);  context_candidates.append(context_candidates__neg);
      
      // hack -- don't use complex reward-concepts [START]
      context_candidates.memMove = true;
      FOR1D_DOWN(context_candidates, k) {
        if ( context_candidates(k)->s->range_type != Symbol::binary
          || context_candidates(k)->s->symbol_type == Symbol::count
          || context_candidates(k)->s->symbol_type == Symbol::function_count
          || context_candidates(k)->s->symbol_type == Symbol::avg
          || context_candidates(k)->s->symbol_type == Symbol::max
          || context_candidates(k)->s->symbol_type == Symbol::function_change
          || context_candidates(k)->s->symbol_type == Symbol::sum
          || context_candidates(k)->s->symbol_type == Symbol::function_reward
        ) {
          context_candidates.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
            
      // create constant-bound comparison literals
      LitL equalityLiterals;
      uintA changedConstantWrapper;
      changedConstantWrapper.append(ex->changedConstants(i));
      NIY;
//       logicObjectManager::getCompLiterals_constantBound(equalityLiterals, changedConstantWrapper, ex->pre, 0);
      
      // hack -- don't use complex reward-concepts [START]
      equalityLiterals.memMove = true;
      FOR1D_DOWN(equalityLiterals, k) {
        NIY;
//         ComparisonLiteral* clit = (ComparisonLiteral*) equalityLiterals(k);
//         if (((ComparisonLiteral*)clit->atom)->fa1->f->category >= category_derived)
//           equalityLiterals.remove(k);
      }
      // hack -- don't use complex reward-concepts [END]
      
      context_candidates.append(equalityLiterals);
      if (comparingValues) {
        // create dynamic-bound comparison literals
        uintA vars_sofar;
        newInvSub.getIns(vars_sofar);
        LitL dynamic_comparisons;
        NIY;
//         logicObjectManager::getCompLiterals_dynamicBound(dynamic_comparisons, vars_sofar, ex->pre, 0);
//         if (DEBUG>3) {cout<<"Dynamic-bound comparison literals:  ";  write(dynamic_comparisons); cout<<endl;}
//         FOR1D(dynamic_comparisons, k) {
//           // hack -- don't use complex reward-concepts [START]
//           if (((ComparisonLiteral*) dynamic_comparisons(k)->atom)->fa1->f->category >= category_derived)
//             continue;
//           if (dynamic_comparisons(k)->args.findValue(ex->changedConstants(i)) >= 0) {
//             context_candidates.append(dynamic_comparisons(k));
//           }
//         }
      }
      if (DEBUG>2) {cout<<"Context literal candidates (based on deictic candidate): ";write(context_candidates);cout<<endl;}
      // create possible newContext
      newContext = newRule->context;
      FOR1D(context_candidates, k) {
        if (DEBUG>3) {cout<<"context_candidates(k)="<<*context_candidates(k)<<" accepted? ";}
        if (reason::holds(ex->pre.lits, context_candidates(k))) {
          newContext.append(newInvSub.apply(context_candidates(k)));
          if (DEBUG>3) {cout<<" yes"<<endl;}
        }
        else {
          if (DEBUG>3) {cout<<" no"<<endl;}
        }
      }
      Literal::sort(newContext);
      if (DEBUG>1) {cout<<"Context with deic ref (potential): ";write(newContext);cout<<endl;}
      // trim literals
      trim_hack(newContext);
      if (slimContext) {NIY;}
      if (DEBUG>0) {
        cout << "newContext after trimming: ";
        write(newContext);
        cout << endl;
      }
      // check whether new variables refers uniquely to s
      SubstitutionSet subs;
      // check whether truly deictic ref (only one sub)
      bool covers = reason::cover(subs, ex->pre.lits, newContext, true, &sub);
      if (covers && subs.num()==1) {
        // check for neg free DRs
        Rule helper_rule;
        helper_rule.action = newRule->action;
        helper_rule.context = newContext;
        uintA drefs_pos, drefs_neg;
        helper_rule.getDeicticRefs(drefs_pos, drefs_neg);
        if (drefs_neg.N == 0) {
          invSub = newInvSub;
          newRule->context = newContext;
          if (DEBUG>1) {cout<<"Accepted ("<<ex->changedConstants(i)<<")"<<endl;}
        }
        else {
          if (DEBUG>1) {cout << "Not accepted ("<<ex->changedConstants(i)<<"):  drefs_neg=" << drefs_neg << endl;}
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
#endif



Rule* ExplainExperiences::explainExperience_deictic_ALL_DRs(StateTransition* ex) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout << "explainExperience_deictic_ALL_DRs [START]" << endl;}
  if (DEBUG>1) {cout<<"Experience:"<<endl; ex->write(cout, true);}
  uint i;
  Rule* newRule = new Rule;
  
  // ensure that all complex are derived
  reason::derive(&ex->pre);
  reason::derive(&ex->post);
  
  // Step 1.1: Create an action and context
  // create action
  Substitution invSub;
  FOR1D(ex->action->args, i) {
    if (reason::isConstant(ex->action->args(i)) && !invSub.hasSubs(ex->action->args(i))) {
      invSub.addSubs2Variable(ex->action->args(i));
    }
  }
  newRule->action = invSub.apply(ex->action);
  if (DEBUG>1) {cout<<"New action: ";newRule->action->write(cout);cout<<endl;}
  // create context
 
  // Step 1.2: Create deictic references and their literals
  Substitution newInvSub = invSub;
  Substitution sub_action;
  invSub.getInverse(sub_action);
  if (DEBUG>0) {
    PRINT(invSub);
    PRINT(newInvSub);
    PRINT(sub_action);
  }
  
  FOR1D(ex->changedConstants, i) {
    if (newInvSub.hasSubs(ex->changedConstants(i))) continue;
    newInvSub.addSubs2Variable(ex->changedConstants(i));
//     cout<<"newInvSub: "; newInvSub.write();  cout<<endl;
  }
  
  uintA context_vars;  context_vars.setAppend(ex->action->args);  context_vars.setAppend(ex->changedConstants);
  TL::sort_asc(context_vars);
  if (DEBUG>1) {PRINT(context_vars);}
  LitL ground_context_candidates, ground_context_candidates_pos, ground_context_candidates_neg;
  Literal::getLiterals_state(ground_context_candidates_pos, context_vars, 1.0);
  Literal::getLiterals_state(ground_context_candidates_neg, context_vars, 0.0);
  ground_context_candidates.append(ground_context_candidates_pos);
  ground_context_candidates.append(ground_context_candidates_neg);
  LitL newContext;
  LitL newContext_ground;
  FOR1D(ground_context_candidates, i) {
    if (DEBUG>3) {cout<<"ground_context_candidates(k)="<<*ground_context_candidates(i)<<" accepted? ";}
    if (reason::holds(ex->pre.lits, ground_context_candidates(i))) {
      newContext.append(newInvSub.apply(ground_context_candidates(i)));
      newContext_ground.append(ground_context_candidates(i));
      if (DEBUG>3) {cout<<" yes"<<endl;}
    }
    else {
      if (DEBUG>3) {cout<<" no"<<endl;}
    }
  }
  Literal::sort(newContext_ground);
  Literal::sort(newContext);
      
  if (DEBUG>0) {PRINT(newContext_ground);  PRINT(newContext);}

  // check whether new variables refers uniquely to s
  SubstitutionSet subs;
    // check whether truly deictic ref (only one sub)
  bool covers = reason::calcSubstitutions(subs, ex->pre.lits, newContext, true, &sub_action);
  if (covers && subs.num()==1) {
    // check for neg free DRs
    Rule helper_rule;
    helper_rule.action = newRule->action;
    helper_rule.context = newContext;
    uintA drefs_neg, drefs_pos;
    helper_rule.getDeicticRefs(drefs_pos, drefs_neg);
    if (drefs_neg.N == 0) {
      invSub = newInvSub;
      newRule->context = newContext;
      if (DEBUG>1) {cout<<"Rule accepted"<<endl;}
    }
    else {
      if (DEBUG>1) {cout << "Rule not accepted:  drefs_neg=" << drefs_neg << endl;}
    }
  }
  else {
    if (DEBUG>1) {
      cout << "Rule not accepted:  covers="<<covers<<"  subs.num()=" << (subs.num()) << "=/=1"<< endl;
      subs.write();
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

DropContextLiterals::DropContextLiterals() : SearchOperator() {
  name = "DropContextLiterals";
  nextRule = 0;
  nextContextLiteral = 0;
}


// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropContextLiterals::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropContextLiterals::findRules [START]"<<endl;
  uint r, p, i;
  Rule* newRule;
  bool stop = false;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextContextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      newRule = new Rule;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
        if (i!=p)
          newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      newRule->action = rulesC_old.rules.elem(r)->action;
      // check for neg free DRs
      uintA drefs_neg, drefs_pos;
      newRule->getDeicticRefs(drefs_pos, drefs_neg);
      if (drefs_neg.N > 0) {
        delete newRule;
        continue;
      }
      // Ensure that new rule covers experiences
      StateTransitionL covered_experiences;
      uintA covered_experiences_ids;
      learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
        learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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

DropContextLiterals_approximativeVersion::DropContextLiterals_approximativeVersion() : SearchOperator() {
  name = "DropContextLiterals_approximativeVersion";
  approximative = true;
  prepareTotalNewSearch = false;
}


// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropContextLiterals_approximativeVersion::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
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
  Rule* newRule;
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
    if (rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)->value > 0. && resamples < DROP_NEGATIVE_BIAS) {
      if (DEBUG>1) {cout<<" (Don't wanna delete positive lit: "; rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)->write(cout); cout<<")"<<endl;}
      usableContextLiterals(k)=1; // set back
      resamples++;
      continue;
    }
    else
      resamples=0;
    newRule = new Rule;
    FOR1D(rulesC_old.rules.elem(id_rule)->context, k) {
      if (k!=id_contextLiteral)
        newRule->context.append(rulesC_old.rules.elem(id_rule)->context(k));
    }
    if (DEBUG>1) {cout<<"Deletion of "<<*rulesC_old.rules.elem(id_rule)->context(id_contextLiteral)<<" executed."<<endl;}
    newRule->action = rulesC_old.rules.elem(id_rule)->action;
    // check for neg free DRs
    uintA drefs_neg, drefs_pos;
    newRule->getDeicticRefs(drefs_pos, drefs_neg);
    if (drefs_neg.N > 0) {
      delete newRule;
      continue;
    }
    // calc experience coverage
    StateTransitionL covered_experiences;
    uintA covered_experiences_ids;
    learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
      learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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


DropReferences::DropReferences() : SearchOperator() {
  name = "DropReferences";
  nextReference = 0;
  nextRule=0;
}

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void DropReferences::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"DropReferences::findRules [START]"<<endl;
  uint r, p, i;
  bool stop = false;
  Rule* newRule = NULL;
  uintA drefs;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    rulesC_old.rules.elem(r)->getDeicticRefs(drefs);
    for (p=nextReference; p<drefs.N; p++) {
      if (DEBUG>0) {
        cout<<"Removing deictic reference "<<drefs(nextReference)<<" in rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
      }
      newRule = new Rule;
      newRule->action = rulesC_old.rules.elem(r)->action;
      FOR1D(rulesC_old.rules.elem(r)->context, i) {
        Literal* lit = rulesC_old.rules.elem(r)->context(i);
        if (DEBUG>4) {PRINT(lit->args);}
        if (lit->s->range_type != Symbol::binary) {
          NIY;
//           ComparisonLiteral* ca = (ComparisonLiteral*) lit->atom;
//           if (ca->fa1->args.findValue(drefs(nextReference)) < 0 ) {
//             if (ca->fa2 == NULL  ||  ca->fa2->args.findValue(drefs(nextReference)) < 0 ) {
//               newRule->context.append(rulesC_old.rules.elem(r)->context(i));
//             }
//           }
        }
        else if (lit->args.findValue(drefs(nextReference))<0)
          newRule->context.append(rulesC_old.rules.elem(r)->context(i));
      }
      if (DEBUG>0) {cout<<"Yielding the new context: "; write(newRule->context); cout<<endl;}
      // check for neg free DRs
      uintA drefs_neg, drefs_pos;
      newRule->getDeicticRefs(drefs_pos, drefs_neg);
      if (drefs_neg.N > 0) {
        delete newRule;
        continue;
      }
      // calc experience coverage
      StateTransitionL covered_experiences;
      uintA covered_experiences_ids;
      learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
        learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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


DropRules::DropRules() : SearchOperator() {
  name = "DropRules";
}
    

void DropRules::createRuleSets(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, 
          MT::Array< RuleSetContainer >& one_rulesC_new) {
  uint DEBUG = 0;
  one_rulesC_new.clear();
  uint i, j;
  if (DEBUG>0) {cout<<"Old rule-set:"<<endl;  rulesC_old.write(cout, true); cout<<endl<<endl;}
  for (i=1; i<rulesC_old.rules.num(); i++) { // default rule must always be in
    if (DEBUG>0) {cout<<"Dropping rule #" << i << endl;}
    RuleSetContainer rulesC_new(&experiences);
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
void DropRules::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {}











// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    S P L I T   O N   L I T E R A L S
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------

SplitOnLiterals::SplitOnLiterals() : SearchOperator() {
  name = "SplitOnLiterals";
  nextRule=1; // ignore default rule
  nextLiteral=0;
}


void SplitOnLiterals::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnLiterals::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint r, k;
  Rule* newRule_pos = NULL;
  Rule* newRule_neg = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      rulesC_old.rules.elem(r)->getAbsentLiterals(absentLiterals, true);
      // use refs [START]
      uintA rule_args;
      rulesC_old.rules.elem(r)->getArguments(rule_args);
      uint newVar = 0;
      FOR1D(rule_args, k)
        if (rule_args(k) == newVar)
          newVar++;
      uintA lit_args;
      lit_args.append(rule_args);
      lit_args.append(newVar);
      uintA wrapper;
      wrapper.append(newVar);
      LitL lits_dr;
      Literal::getLiterals_state(lits_dr, lit_args, wrapper, 1.0, true);
      // hack -- don't use complex reward-concepts [START]
      FOR1D_DOWN(lits_dr, k) {
        if ( lits_dr(k)->s->range_type != Symbol::binary
          || lits_dr(k)->s->symbol_type == Symbol::count
          || lits_dr(k)->s->symbol_type == Symbol::avg
          || lits_dr(k)->s->symbol_type == Symbol::max
          || lits_dr(k)->s->symbol_type == Symbol::function_change
          || lits_dr(k)->s->symbol_type == Symbol::sum
          || lits_dr(k)->s->symbol_type == Symbol::function_reward
        ) {
          lits_dr.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated restriction literals for rule:"<<endl<<*rulesC_old.rules.elem(r);
        cout<<"Restriction literals: "<<lits_dr<<endl;
      }
      
      absentLiterals.append(lits_dr);
      // use refs [END]
      
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
      if (Literal::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && rulesC_old.rules.elem(r)->context.findValue(absentLiterals(nextLiteral)) < 0) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_pos = new Rule;
        newRule_pos->action = rulesC_old.rules.elem(r)->action;
        newRule_pos->context = rulesC_old.rules.elem(r)->context;
        newRule_pos->insertContext(absentLiterals(nextLiteral));
        StateTransitionL covered_experiences;
        uintA covered_experiences_ids;
        learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule_pos, experiences);
        if (covered_experiences.N > 0) {
          if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
          MT::Array< uintA > experiences_per_outcome;
          learn::learn_outcomes(newRule_pos, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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
      Literal* nextLiteral_neg = absentLiterals(nextLiteral)->getNegated();
      wrapper.clear();
      wrapper.append(nextLiteral_neg);
      if (DEBUG>1) {
        cout<<"Trying to insert ";nextLiteral_neg->write(cout);cout<<"   into   ";
        write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      if (Literal::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)
             && rulesC_old.rules.elem(r)->context.findValue(nextLiteral_neg) < 0) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule_neg = new Rule;
        newRule_neg->action = rulesC_old.rules.elem(r)->action;
        newRule_neg->context = rulesC_old.rules.elem(r)->context;
        newRule_neg->insertContext(nextLiteral_neg);
        StateTransitionL covered_experiences;
        uintA covered_experiences_ids;
        learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule_neg, experiences);
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
          learn::learn_outcomes(newRule_neg, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {write(rulesC_2add.rules);}}
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

AddLiterals::AddLiterals() : SearchOperator() {
  name = "AddLiterals";
  nextRule=1; // ignore default rule
  nextLiteral=0;
}


void AddLiterals::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"AddLits::findRules [START]"<<endl;
  uint r;
  Rule* newRule = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (absentLiterals.N == 0) { // first round
      rulesC_old.rules.elem(r)->getAbsentLiterals(absentLiterals);
      // hack -- don't use complex reward-concepts [START]
      absentLiterals.memMove = true;
      uint k;
      // hack -- don't use complex reward-concepts [START]
      FOR1D_DOWN(absentLiterals, k) {
        if ( absentLiterals(k)->s->range_type != Symbol::binary
          || absentLiterals(k)->s->symbol_type == Symbol::count
          || absentLiterals(k)->s->symbol_type == Symbol::avg
          || absentLiterals(k)->s->symbol_type == Symbol::max
          || absentLiterals(k)->s->symbol_type == Symbol::function_change
          || absentLiterals(k)->s->symbol_type == Symbol::sum
          || absentLiterals(k)->s->symbol_type == Symbol::function_reward
        ) {
          absentLiterals.remove(k);
        }
      }
      // hack -- don't use complex reward-concepts [END]
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
      if (Literal::nonContradicting(wrapper, rulesC_old.rules.elem(r)->context)) {
        if (DEBUG>1) cout<<" --> Feasible and will be done."<<endl;
        newRule = new Rule;
        newRule->action = rulesC_old.rules.elem(r)->action;
        newRule->context = rulesC_old.rules.elem(r)->context;
        newRule->insertContext(absentLiterals(nextLiteral));
        StateTransitionL covered_experiences;
        uintA covered_experiences_ids;
        learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
          learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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

AddReferences::AddReferences() : SearchOperator() {
  name = "AddReferences";
  nextRule=1; // ignore default rule
  nextLiteral=0;
}


void AddReferences::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"AddReferences::findRules [START]"<<endl;
  uint r, i;
  Rule* newRule = NULL;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (restrictionLiterals.N == 0) { // first round
      uintA rule_args;
      rulesC_old.rules.elem(r)->getArguments(rule_args);
      uint newVar = 0;
      FOR1D(rule_args, i)
        if (rule_args(i) == newVar)
          newVar++;
      uintA arguments;
      arguments.append(rule_args);
      arguments.append(newVar);
      uintA wrapper;
      wrapper.append(newVar);
      Literal::getLiterals_state(restrictionLiterals, arguments, wrapper, 1.);
      // hack -- don't use complex reward-concepts [START]
      restrictionLiterals.memMove = true;
      FOR1D_DOWN(restrictionLiterals, i) {
        if ( restrictionLiterals(i)->s->range_type != Symbol::binary
          || restrictionLiterals(i)->s->symbol_type == Symbol::count
          || restrictionLiterals(i)->s->symbol_type == Symbol::avg
          || restrictionLiterals(i)->s->symbol_type == Symbol::max
          || restrictionLiterals(i)->s->symbol_type == Symbol::function_change
          || restrictionLiterals(i)->s->symbol_type == Symbol::sum
          || restrictionLiterals(i)->s->symbol_type == Symbol::function_reward
        ) {
          restrictionLiterals.remove(i);
        }
      }
      // hack -- don't use complex reward-concepts [END]
      if (DEBUG>2) {
        cout << "Calculated restriction literals for rule "<<rulesC_old.experiences_per_rule(r)<<":"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"Restriction literals: ";write(restrictionLiterals);cout<<endl;
      }
    }
    while (nextLiteral<restrictionLiterals.N) {
      if (DEBUG>1) {
        cout<<"Inserting ";restrictionLiterals(nextLiteral)->write(cout);cout<<"   into   ";
        write(rulesC_old.rules.elem(r)->context);cout<<endl;
      }
      newRule = new Rule;
      newRule->action = rulesC_old.rules.elem(r)->action;
      newRule->context = rulesC_old.rules.elem(r)->context;
      newRule->insertContext(restrictionLiterals(nextLiteral));
      StateTransitionL covered_experiences;
      uintA covered_experiences_ids;
      learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
        learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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

GeneralizeEquality::GeneralizeEquality() : SearchOperator() {
  name = "GeneralizeEquality";
  nextRule=1; // ignore default rule
  nextLiteral=0;
  doneLess=false;
}

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void GeneralizeEquality::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"GeneralizeEquality::findRules [START]"<<endl;
  uint r, p, i;
  Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (rulesC_old.rules.elem(r)->context(p)->s->range_type != Symbol::binary
                &&  rulesC_old.rules.elem(r)->context(p)->comparison_type == Literal::comparison_equal) {
        if (DEBUG>1) {cout<<"Generalizing literal #"<<p<<": ";rulesC_old.rules.elem(r)->context(p)->write(cout);cout<<endl;}
        newRule = new Rule;
        newRule->action = rulesC_old.rules.elem(r)->action;
        FOR1D(rulesC_old.rules.elem(r)->context, i) {
          if (i!=p) {
            newRule->context.append(rulesC_old.rules.elem(r)->context(i));
          }
          else {
            Literal* lit_comp__old = rulesC_old.rules.elem(r)->context(p);
            Literal* lit_comp__new = NULL;
//             create new context literal
            if (!doneLess) {
              doneLess = true; // try greater-equal thereafter
              nextLiteral = p;
              lit_comp__new = Literal::get(lit_comp__old->s, lit_comp__old->args, lit_comp__old->value, Literal::comparison_lessEqual);
            }
            else {
              doneLess=false;
              nextLiteral = p+1; // try next literal thereafter
              lit_comp__new = Literal::get(lit_comp__old->s, lit_comp__old->args, lit_comp__old->value, Literal::comparison_greaterEqual);
            }
            if (DEBUG>1) {cout<<"Generalized literal: "<<*lit_comp__new<<endl;}
            newRule->context.append(lit_comp__new);
          }
        }
        Literal::sort(newRule->context);
        if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
        StateTransitionL covered_experiences;
        uintA covered_experiences_ids;
        learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
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
          learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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


SplitOnEqualities::SplitOnEqualities() : SearchOperator() {
  name = "SplitOnEqualities";
  nextRule=1; // ignore default rule
  nextVar=0;
  nextFunc=0;
  SymL function_symbols;
  Symbol::get_state_nonBinary(function_symbols) ;
  usedFunctions.append(function_symbols);
  // Achtung DON'T used derived: usedFunctions.append(logicObjectManager::f_derived);!!!!
}

void SplitOnEqualities::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnEqualities::findRules [START]"<<endl;
  rulesC_2add.clear();
  uint r, v, f, i;
  
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    if (vars.N == 0) { // first round
      // determine vars
      rulesC_old.rules.elem(r)->getArguments(vars);
      if (DEBUG>2) {
        cout << "Inspecting rule:"<<endl;
        rulesC_old.rules.elem(r)->write(cout);
        cout<<"findValue the following vars: "<<vars<<endl;
      }
    }
    for (v=nextVar; v<vars.d0; v++) {
      for (f=nextFunc; f<usedFunctions.d0; f++) {
        if (usedFunctions(f)->arity != 1)  // nur fuer unary functions!!
          continue;
        if (DEBUG>1) {cout<<"Checking var "<<v<<" with function "<<usedFunctions(f)->name<<endl;}
        // check whether function has not been used with this variable
        bool alreadyUsed = false;
        FOR1D(rulesC_old.rules.elem(r)->context, i) {
          if (rulesC_old.rules.elem(r)->context(i)->s == usedFunctions(f)) {
            if (rulesC_old.rules.elem(r)->context(i)->args.findValue(vars(v)) >= 0)
              alreadyUsed = true;
          }
        }
        if (DEBUG>1) PRINT(alreadyUsed)
        if (!alreadyUsed) {
          arr usedValues;
          // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          SymbolicState::getValues(usedValues, experiences(0)->pre, *usedFunctions(f), experiences(0)->pre.state_constants);
          if (DEBUG>2) {cout<<"Using function values: "<<usedValues<<endl;}
          FOR1D(usedValues, i) {
            Rule* newRule = new Rule;
            newRule->action = rulesC_old.rules.elem(r)->action;
            newRule->context = rulesC_old.rules.elem(r)->context;
            uintA tosplit_args;
            tosplit_args.append(vars(v));
            Literal* lit_tosplit = Literal::get(usedFunctions(f), tosplit_args, usedValues(i), Literal::comparison_equal);
            newRule->insertContext(lit_tosplit);
            if (DEBUG>0) {cout<<"Potential new rule:"<<endl<<newRule;}
            StateTransitionL covered_experiences;
            uintA covered_experiences_ids;
            learn::calcCoverage(covered_experiences, covered_experiences_ids, newRule, experiences);
            if (covered_experiences.N > 0) {
              if (DEBUG>1) cout<<"Covers "<<covered_experiences.N<<" experiences and will be kept."<<endl;
              MT::Array< uintA > experiences_per_outcome;
              learn::learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids);
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
  if (DEBUG>0) {if (rulesC_2add.rules.num() > 0) {write(rulesC_2add.rules);}}
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

ChangeRange::ChangeRange() : SearchOperator() {
    name = "ChangeRange";
    nextRule=1; // ignore default rule
    nextLiteral=0;
    nextPossibleValue=0;
}

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
// only for CONSTANT-bound comparison predicates
void ChangeRange::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"ChangeRange::findRules [START]"<<endl;
#if 0
  uint r, p, v;
  Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (rulesC_old.rules.elem(r)->context(p)->s->symbol_type == Predicate::predicate_comparison) {
        ComparisonLiteral* clit = dynamic_cast<ComparisonLiteral*>(rulesC_old.rules.elem(r)->context(p));
        if (!((ComparisonLiteral*)clit->atom)->hasConstantBound()) break;
        CHECK(clit!=NULL, "Cast failed");
        if (DEBUG>1) {cout<<"Changing range of literal #"<<p<<": ";clit->write(cout);cout<<endl;}
        if (nextPossibleValue == 0) {
          // collect possibleValues 
          reason::usedValues(*(((ComparisonLiteral*)clit->atom)->fa1->f), experiences(0)->pre, possibleValues); // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          possibleValues.removeValueSafe(((ComparisonLiteral*)clit->atom)->bound);
        }
        if (DEBUG>2) {cout<<"Using values: "<<possibleValues<<endl;}
        
        for (v=nextPossibleValue; v<possibleValues.d0; v++) {
          newRule = new Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          uint n;
          FOR1D(rulesC_old.rules.elem(r)->context, n) {
            if (n!=p) {
              newRule->context.append(rulesC_old.rules.elem(r)->context(n));
            }
            else {
              // create new context literal
              ComparisonLiteral* eq = logicObjectManager::getCompLiteral_constant(((ComparisonLiteral*)clit->atom)->fa1->f, ((ComparisonLiteral*)clit->atom)->comparisonType, possibleValues(v), ((ComparisonLiteral*)clit->atom)->fa1->args);
              reason::insert(*newRule, *eq);
            }
          }
          if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          StateTransitionL covered_experiences;
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
            learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos);
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
#endif
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

MakeInterval::MakeInterval() : SearchOperator() {
  name = "MakeInterval";
  nextRule=1; // ignore default rule
  nextLiteral=0;
  nextPossibleValue=0;
}

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void MakeInterval::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"MakeInterval::findRules [START]"<<endl;
#if 0
  uint r, p, v;
  Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (p=nextLiteral; p<rulesC_old.rules.elem(r)->context.N; p++) {
      if (rulesC_old.rules.elem(r)->context(p)->s->symbol_type == Predicate::predicate_comparison) {
        ComparisonLiteral* clit = dynamic_cast<ComparisonLiteral*>(rulesC_old.rules.elem(r)->context(p));
        if (!((ComparisonLiteral*)clit->atom)->hasConstantBound()) { // only defined for constant bounds
          continue;
        }
        CHECK(clit!=NULL, "Cast failed")
        // TODO CHECK FOR DOUBLE INTERVALS!!!!!!!!!!!!
        // omit equality literals
        if (((ComparisonLiteral*)clit->atom)->comparisonType == comparison_equal)
          continue;
        if (DEBUG>1) {cout<<"Making interval for literal #"<<p<<": ";clit->write(cout);cout<<endl;}
        if (nextPossibleValue == 0) {
          // collect possibleValues 
          reason::usedValues(*(((ComparisonLiteral*)clit->atom)->fa1->f), experiences(0)->pre, possibleValues); // TODO this is not the perfect solution... need to find a clever way to store all used function values...
          if (((ComparisonLiteral*)clit->atom)->comparisonType == comparison_less 
              || ((ComparisonLiteral*)clit->atom)->comparisonType == comparison_lessEqual) {
            uint pv;
            FOR1D_DOWN(possibleValues, pv) {
              if (possibleValues(pv) >= ((ComparisonLiteral*)clit->atom)->bound)
                possibleValues.remove(pv);
            }
          }
          else if (((ComparisonLiteral*)clit->atom)->comparisonType == comparison_greater 
                    || ((ComparisonLiteral*)clit->atom)->comparisonType == comparison_greaterEqual) {
            uint pv;
            FOR1D_DOWN(possibleValues, pv) {
              if (possibleValues(pv) <= ((ComparisonLiteral*)clit->atom)->bound)
                possibleValues.remove(pv);
            }
          }
          else {HALT("Don't know this comparison type, digger.");}
        }
        if (DEBUG>2) {cout<<"Using values: "<<possibleValues<<endl;}
        
        for (v=nextPossibleValue; v<possibleValues.d0; v++) {
          newRule = new Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          
          // create new context literal
          ComparisonType newCompType;
          if (((ComparisonLiteral*)clit->atom)->comparisonType == comparison_less 
              || ((ComparisonLiteral*)clit->atom)->comparisonType == comparison_lessEqual) {
            newCompType = comparison_greaterEqual;
          }
          else if (((ComparisonLiteral*)clit->atom)->comparisonType == comparison_greater 
                      || ((ComparisonLiteral*)clit->atom)->comparisonType == comparison_greaterEqual) {
            newCompType = comparison_lessEqual;
          }
          else
              HALT("Don't know this comparison type, digger.")
          ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_constant(((ComparisonLiteral*)clit->atom)->fa1->f, newCompType, possibleValues(v), ((ComparisonLiteral*)clit->atom)->fa1->args);
          reason::insert(*newRule, *new_clit);

          if (DEBUG>0) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          StateTransitionL covered_experiences;
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
            learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos);
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
#endif
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

CompareFunctionValues::CompareFunctionValues() : SearchOperator() {
  name = "CompareFunctionValues";
  nextRule=1; // ignore default rule
  nextFunction=0;
  nextComparisonType=0;
  nextTermCombination=0;
  SymL function_symbols;
  Symbol::get_state_nonBinary(function_symbols) ;
  usedFunctions.append(function_symbols);
  // collect comparison types SAVE THE ORDER!
  comparisonTypes.append(Literal::comparison_equal);comparisonTypes.append(Literal::comparison_less);comparisonTypes.append(Literal::comparison_lessEqual);
  comparisonTypes.append(Literal::comparison_greater);comparisonTypes.append(Literal::comparison_greaterEqual);
}


// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void CompareFunctionValues::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"CompareFunctionValues::findRules [START]"<<endl;
#if 0
  uint r, f, c, t;
  Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (f=nextFunction; f<usedFunctions.d0; f++) {
      // calc new term combos if needed
      if (nextTermCombination==0) {
        termCombos.clear();
        uintA terms;
        rulesC_old.rules.elem(r)->getArguments(terms);
        MT::Array< uintA > termCombos_unfiltered;
        TL::allPermutations(termCombos_unfiltered, terms, 2 * usedFunctions(f)->arity, false, true);
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
          newRule = new Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          uintA args1, args2;
          uint u;
          for (u=0; u<termCombos(t).N/2; u++) args1.append(termCombos(t)(u));
          for (u=termCombos(t).N/2; u<termCombos(t).N; u++) args2.append(termCombos(t)(u));
          ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_dynamic(usedFunctions(f), comparisonTypes(c), args1, args2);
          reason::insert(*newRule, *new_clit);
          if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          StateTransitionL covered_experiences;
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
            learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos);
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
//             StateTransitionL coveredExperiences;
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
#endif
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

SplitOnCompareFunctionValues::SplitOnCompareFunctionValues() : SearchOperator() {
  name = "SplitOnCompareFunctionValues";
  nextRule=1; // ignore default rule
  nextFunction=0;
  nextTermCombination=0;
  SymL function_symbols;
  Symbol::get_state_nonBinary(function_symbols);
  // collect comparison types SAVE THE ORDER!
  comparisonTypes.append(Literal::comparison_equal);comparisonTypes.append(Literal::comparison_less);comparisonTypes.append(Literal::comparison_greater);
}

// creates possible new rules for the given rule-set
// newRules are potential additional rules which are all intended to be added to the SAME rule-set!
void SplitOnCompareFunctionValues::findRules(const RuleSetContainer& rulesC_old, const StateTransitionL& experiences, RuleSetContainer& rulesC_2add) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"SplitOnCompareFunctionValues::findRules [START]"<<endl;
#if 0
  uint r, f, c, t;
  Rule* newRule;
  for (r=nextRule; r<rulesC_old.rules.num(); r++) {
    for (f=nextFunction; f<usedFunctions.d0; f++) {
      // calc new term combos if needed
      if (nextTermCombination==0) {
        termCombos.clear();
        uintA terms;
        rulesC_old.rules.elem(r)->getArguments(terms);
        MT::Array< uintA > termCombos_unfiltered;
        TL::allPermutations(termCombos_unfiltered, terms, 2 * usedFunctions(f)->arity, false, true);
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
          newRule = new Rule;
          newRule->action = rulesC_old.rules.elem(r)->action;
          newRule->context = rulesC_old.rules.elem(r)->context;
          uintA args1, args2;
          uint u;
          for (u=0; u<termCombos(t).N/2; u++) args1.append(termCombos(t)(u));
          for (u=termCombos(t).N/2; u<termCombos(t).N; u++) args2.append(termCombos(t)(u));
          ComparisonLiteral* new_clit = logicObjectManager::getCompLiteral_dynamic(usedFunctions(f), comparisonTypes(c), args1, args2);
          reason::insert(*newRule, *new_clit);
          if (DEBUG>1) {cout<<"Potential new rule:"<<endl;newRule->write(cout);}
          StateTransitionL covered_experiences;
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
            learn_outcomes(newRule, experiences_per_outcome, covered_experiences, covered_experiences_ids, alpha_PEN, p_min, pen_sum, pen_pos);
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
#endif
  
  if (DEBUG>0) cout<<"SplitOnCompareFunctionValues::findRules [END]"<<endl;
}


void SplitOnCompareFunctionValues::reset() {
  nextRule=1; // ignore default rule
  nextFunction=0;
  nextTermCombination=0;
}




























}
