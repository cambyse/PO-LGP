/*  
    Copyright 2011   Tobias Lang
    
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

#include "logicReasoning.h"
#include "ruleReasoning.h"
#include <stdlib.h>

#define RE_fast 1



  /****************************************
    BASIC HELPERS
  ***************************************/


void TL::ruleReasoning::calcDeicticRefs(const TL::Rule& rule, uintA& drefs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcDeicticRefs [START]"<<endl;
  if (DEBUG>0) {cout<<"Rule:"<<endl<<rule;}
  drefs.clear();
  uint i, j;
  FOR1D(rule.context, i) {
    if (rule.context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      ComparisonAtom* ca = (ComparisonAtom*) rule.context(i)->atom;
      FOR1D(ca->fa1->args, j) {
        if (rule.action->args.findValue(ca->fa1->args(j)) < 0)
          drefs.setAppend(ca->fa1->args(j));
      }
      if (ca->fa2 != NULL) {
        FOR1D(ca->fa2->args, j) {
          if (rule.action->args.findValue(ca->fa2->args(j)) < 0)
            drefs.setAppend(ca->fa2->args(j));
        }
      }
    }
    else {
      FOR1D(rule.context(i)->atom->args, j) {
        if (rule.action->args.findValue(rule.context(i)->atom->args(j)) < 0)
          drefs.setAppend(rule.context(i)->atom->args(j));
      }
    }
  }
  // sort them
  TL::sort_asc(drefs);
  
  if (DEBUG>0) {
    rule.write(cout);
    PRINT(drefs)
  }
  if (DEBUG>0) cout<<"calcDeicticRefs [END]"<<endl;
}


void TL::ruleReasoning::calcDeicticRefs(const TL::Rule& rule, uintA& drefs, boolA& inNegatedLiteralsOnly) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcDeicticRefs2 [START]"<<endl;
  calcDeicticRefs(rule, drefs);
  inNegatedLiteralsOnly.resize(drefs.N);
  inNegatedLiteralsOnly.setUni(true);
  uint r, l;
  FOR1D(drefs, r) {
    FOR1D(rule.context, l) {
      if (rule.context(l)->atom->pred->type == TL::Predicate::predicate_comparison) {
        ComparisonAtom* ca = (ComparisonAtom*) rule.context(l)->atom;
        if (ca->fa1->args.findValue(drefs(r)) >= 0) {
          inNegatedLiteralsOnly(r) = false;
          break;
        }
        if (inNegatedLiteralsOnly(r)  &&  ca->fa2 != NULL) {
          inNegatedLiteralsOnly(r) = false;
          break;
        }
      }
      else {
        if (rule.context(l)->atom->args.findValue(drefs(r)) >= 0)
          if (rule.context(l)->positive) {
            inNegatedLiteralsOnly(r) = false;
            break;
          }
      }
      if (!inNegatedLiteralsOnly(r))
        break;
    }
  }
  if (DEBUG>0) {
    PRINT(drefs);
    PRINT(inNegatedLiteralsOnly);
  }
  if (DEBUG>0) cout<<"calcDeicticRefs2 [END]"<<endl;
}


void TL::ruleReasoning::getNegFreeDeicticRefs(uintA& neg_drefs, const TL::Rule& rule) {
  uint i;
  uintA action_args;
  FOR1D(rule.action->args, i) {
    action_args.setAppend(rule.action->args(i));
  }
  uintA neg_context_args;
  TL::logicReasoning::getUnconstrainedNegatedArguments(neg_context_args, rule.context);
  FOR1D(neg_context_args, i) {
    if (action_args.findValue(neg_context_args(i)) < 0)
      neg_drefs.append(neg_context_args(i));
  }
}


void TL::ruleReasoning::DRcontext(const TL::Rule& rule, boolA& findValueDR) {
  findValueDR.resize(rule.context.N);
  findValueDR.setUni(false);
  uint i, k;
  FOR1D(rule.context, i) {
    if (rule.context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      ComparisonAtom* ca = (ComparisonAtom*) rule.context(i)->atom;
      FOR1D(ca->fa1->args, k) {
        if (rule.action->args.findValue(ca->fa1->args(k)) < 0) {
          findValueDR(i) = true;
          break;
        }
      }
      if (ca->fa2 != NULL) {
        FOR1D(ca->fa2->args, k) {
          if (rule.action->args.findValue(ca->fa2->args(k)) < 0) {
            findValueDR(i) = true;
            break;
          }
        }
      }
    }
    else {
      FOR1D(rule.context(i)->atom->args, k) {
        if (rule.action->args.findValue(rule.context(i)->atom->args(k)) < 0) {
          findValueDR(i) = true;
          break;
        }
      }
    }
  }
}





void TL::ruleReasoning::calcTerms(const TL::Rule& rule, uintA& terms) {
  terms.clear();
  uint i;
  FOR1D(rule.action->args, i) {
    terms.setAppend(rule.action->args(i));
  }
  FOR1D(rule.context, i) {
    if (rule.context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
      ComparisonAtom* ca = (ComparisonAtom*) rule.context(i)->atom;
      terms.setAppend(ca->fa1->args);
      if (ca->fa2 != NULL) terms.setAppend(ca->fa2->args);
    }
    else {
      terms.setAppend(rule.context(i)->atom->args);
    }
  }
  TL::sort_asc(terms);
}


void TL::ruleReasoning::calcAbsentLiterals(const TL::Rule& rule, LitL& literals, bool positiveOnly) {
  literals.clear();
  uintA terms;
  calcTerms(rule, terms);
  LitL literals_cands;
  TL::logicObjectManager::getLiterals(literals_cands, terms, positiveOnly);
  uint i, j;
  FOR1D(literals_cands, i) {
    FOR1D(rule.context, j) {
      if (*literals_cands(i) == *rule.context(j))
        break;
    }
    if (j==rule.context.N)
      literals.append(literals_cands(i));
  }
}


bool TL::ruleReasoning::usesLiteral(const TL::Rule& rule, TL::Literal* lit) {
  CHECK(lit->positive, "should be positive");
  TL::Literal* lit_neg = TL::logicObjectManager::getLiteralNeg(lit);
  uint i, k;
  FOR1D(rule.context, i) {
    if (rule.context(i) == lit  ||  rule.context(i) == lit_neg)
      return true;
  }
  if (rule.action == lit->atom)
    return true;
  FOR1D(rule.outcomes, i) {
    FOR1D(rule.outcomes(i), k) {
      if (rule.outcomes(i)(k) == lit  ||  rule.outcomes(i)(k) == lit_neg)
        return true;
    }
  }
  return false;
}


void TL::ruleReasoning::changingConcepts(uintA& changingIds_p, uintA& changingIds_f, const TL::RuleSet& rules) {
  changingIds_p.clear();
  changingIds_f.clear();
  uint r, o, i;
  uintA changingIds_p_prim, changingIds_f_prim;
  FOR1D_(rules, r) {
    FOR1D(rules.elem(r)->outcomes, o) {
      FOR1D(rules.elem(r)->outcomes(o), i) {
        if (rules.elem(r)->outcomes(o)(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
          TL::ComparisonLiteral* clit = ((TL::ComparisonLiteral*) rules.elem(r)->outcomes(o)(i));
          changingIds_f_prim.setAppend(((ComparisonAtom*)clit->atom)->fa1->f->id);
        }
        else {
          changingIds_p_prim.setAppend(rules.elem(r)->outcomes(o)(i)->atom->pred->id);
        }
      }
    }
  }
  changingIds_p.append(changingIds_p_prim);
  changingIds_f.append(changingIds_f_prim);
  // also include p_derived ones
  FOR1D(changingIds_p_prim, i) {
    PredL p_suc;
    FuncL f_suc;
    TL::logicObjectManager::dependencyGraph.getAllSuccessors(*TL::logicObjectManager::dependencyGraph.getPredicate(changingIds_p_prim(i)), p_suc, f_suc);
    FOR1D(p_suc, o) {changingIds_p.setAppend(p_suc(o)->id);}
    FOR1D(f_suc, o) {changingIds_f.setAppend(f_suc(o)->id);}
  }
  FOR1D(changingIds_f_prim, i) {
    PredL p_suc;
    FuncL f_suc;
    TL::logicObjectManager::dependencyGraph.getAllSuccessors(*TL::logicObjectManager::dependencyGraph.getFunction(changingIds_f_prim(i)), p_suc, f_suc);
    FOR1D(p_suc, o) {changingIds_p.setAppend(p_suc(o)->id);}
    FOR1D(f_suc, o) {changingIds_f.setAppend(f_suc(o)->id);}
  }
}


uint TL::ruleReasoning::numLiterals(const TL::Rule& rule) {
  uint num = 0;
  uint i;
  num += rule.context.N;
  FOR1D(rule.outcomes, i) {
    num += rule.outcomes(i).N;
  }
  return num;
}


bool TL::ruleReasoning::stupidContext(const TL::Rule& rule) {
  // TODO needs more refinement
  // (1) checks that dynamic comparison pts don't use same arguments
  // for both sides of comparison
  uint p,i;
  FOR1D(rule.context, p) {
    if (rule.context(p)->atom->pred->type == TL::Predicate::predicate_comparison) {
      TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) rule.context(p);
      if (!((ComparisonAtom*)clit->atom)->hasConstantBound()) {
        CHECK(((ComparisonAtom*)clit->atom)->args.N % 2 == 0, "strange slot assignments");
        bool different = false;
        for (i=0; i<((ComparisonAtom*)clit->atom)->args.N / 2; i++) {
          if (((ComparisonAtom*)clit->atom)->args(i) != ((ComparisonAtom*)clit->atom)->args(i+((ComparisonAtom*)clit->atom)->args.N / 2)) {
            different = true;
            break;
          }
        }
        if (!different)
          return true;
      }
    }
  }
  return false;
}


bool TL::ruleReasoning::isGrounded(const TL::Rule& r) {
  CHECK(r.action->pred->id != TL::DEFAULT_ACTION_PRED__ID, "don't use this method for default rule");
  CHECK(r.action != TL::logicObjectManager::getAtom_doNothing(), "don't use this method for doNothing rule");
  uint i, j;
  FOR1D(r.context, i) {
    if (!TL::logicReasoning::isGrounded(r.context(i)))
      return false;
  }
  if (!TL::logicReasoning::isGrounded(r.action))
    return false;
  FOR1D(r.outcomes, i) {
    FOR1D(r.outcomes(i), j) {
      if (!TL::logicReasoning::isGrounded(r.outcomes(i)(j)))
        return false;
    }
  }
  return true;
}


bool TL::ruleReasoning::isAbstract(const TL::Rule& r) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"ruleReasoning::isAbstract [START]"<<endl;}
  if (DEBUG>1) {r.write();  cout<<endl;}
  CHECK(r.action->pred->id != TL::DEFAULT_ACTION_PRED__ID, "don't use this method for default rule");
  uint i, j;
  FOR1D(r.context, i) {
    if (!TL::logicReasoning::isAbstract(r.context(i))) {
      if (DEBUG>0) {cout<<"No abstract context #i="<<i<<": "<<*r.context(i)<<endl;   cout<<"ruleReasoning::isAbstract [END]"<<endl;}
      return false;
    }
  }
  if (!TL::logicReasoning::isAbstract(r.action)) {
    if (DEBUG>0) {cout<<"No abstract action: "<<*r.action<<endl;   cout<<"ruleReasoning::isAbstract [END]"<<endl;}
    return false;
  }
  FOR1D(r.outcomes, i) {
    FOR1D(r.outcomes(i), j) {
      if (!TL::logicReasoning::isAbstract(r.outcomes(i)(j))) {
        if (DEBUG>0) {cout<<"No abstract outcome #i="<<i<<", j="<<j<<": "<<*r.outcomes(i)(j)<<endl;   cout<<"ruleReasoning::isAbstract [END]"<<endl;}
        return false;
      }
    }
  }
  if (DEBUG>0) {cout<<"Rule is abstract."<<endl;   cout<<"ruleReasoning::isAbstract [END]"<<endl;}
  return true;
}



bool TL::ruleReasoning::isGrounded_positives(TL::Rule* r) {
  uint i, j;
  FOR1D(r->context, i) {
    if (r->context(i)->positive  &&  !TL::logicReasoning::isGrounded(r->context(i)))
      return false;
  }
  if (!TL::logicReasoning::isGrounded(r->action))
    return false;
  FOR1D(r->outcomes, i) {
    FOR1D(r->outcomes(i), j) {
      if (!TL::logicReasoning::isGrounded(r->outcomes(i)(j)))
        return false;
    }
  }
  return true;
}

void TL::ruleReasoning::removeDoubleLiterals(const TL::RuleSet& ground_rules) {
  int DEBUG = 0;
  uint i, k, l, m;
  TL::Rule* r;
  FOR1D_(ground_rules, i) {
    r = ground_rules.elem(i);
    if (DEBUG>0) {
      cout<<"BEFORE:";
      r->write(cout);
    }
    // (1) context
    for (k=0; ; k++) {
      if (r->context.N <= k+1)
        break;
      l=k+1;
      while (l < r->context.N) {
        if (r->context(k) == r->context(l)) {
          r->context.memMove = true;
          r->context.remove(l);
          if (DEBUG>0) {cout<<"HUI"<<endl;}
        }
        else {
          l++;
        }
      }
    }
    // (2) Outcomes
    FOR1D(r->outcomes, m) {
      for (k=0; ; k++) {
        if (r->outcomes(m).N <= k+1)
          break;
        l=k+1;
        while (l < r->outcomes(m).N) {
          if (r->outcomes(m)(k) == r->outcomes(m)(l)) {
            r->outcomes(m).memMove = true;
            r->outcomes(m).remove(l);
            if (DEBUG>0) {cout<<"HUI"<<endl;}
          }
          else {
            l++;
          }
        }
      }
    }
    if (DEBUG>0) {
      cout<<"AFTER:";
      r->write(cout);
    }
  }
}


void TL::ruleReasoning::checkRules(const TL::RuleSet& rules) {
  uint i, k;
  TL::Rule* r;
  FOR1D_(rules, i) {
    r = rules.elem(i);
    // (1) Check no double in context
    if (r->context.containsDoubles()) {
      r->write(cerr);
      HALT("Context contains doubles");
    }
    // (2) Check no double in outcomes
    FOR1D(r->outcomes, k) {
      if (r->outcomes(k).containsDoubles()) {
        r->write(cerr);
        HALT("Outcome " << k << " contain doubles");
      }
    }
    // (3) Check probs sum to 1
    if (fabs(sum(r->probs) - 1.0) > 0.001) {
      r->write(cerr);
      printf("%10.10f\n", sum(r->probs));
      HALT("bad probs for rule");
    }
    // (4) Check no pure comparison predicate in context or outcomes
    FOR1D(r->context, k) {
      ComparisonLiteral* clit = dynamic_cast< ComparisonLiteral* >(r->context(k));
      if (clit == NULL  &&  r->context(k)->atom->pred->type == TL::Predicate::predicate_comparison) {
        r->write(cerr);
        HALT("Comparison predicate used as normal Predicate Instance (Context literal #" << k << " )");
      }
    }
  }
}



void TL::ruleReasoning::cleanup(TL::Rule& rule) {
  uint i;
  // clean-up drefs
  if (!isGrounded(rule)) {
    uintA drs;
    calcDeicticRefs(rule, drs);
    if (drs.N > 0) {
      uint max_dr = drs.max();
      TL::Substitution sub;
      for (i=rule.action->pred->d; i<max_dr; i++) {
        if (drs.findValue(i) < 0) {
          sub.addSubs(max_dr-sub.num(), i);
        }
      }
      if (!sub.empty()) {
  //       cout<<endl<<endl<<"Vorher:"<<endl; rule.writeNice();
        LitL new_context;
        sub.apply(rule.context, new_context);
        rule.context = new_context;
        for (i=0; i<rule.outcomes.N-1; i++) {
          LitL new_outcome;
          sub.apply(rule.outcomes(i), new_outcome);
          rule.outcomes(i) = new_outcome;
        }
  //       cout<<endl<<endl<<"Nachher:"<<endl; rule.writeNice();
      }
    }
  }
  // order context
  TL::logicReasoning::sort(rule.context);
  // order outcomes
  for (i=0; i<rule.outcomes.N-1; i++) {
    TL::logicReasoning::sort(rule.outcomes(i));
  }
}




/****************************************
  RULE MANIPULATION
***************************************/


void TL::ruleReasoning::insert(TL::Rule& rule, TL::Literal& literal) {
  uint i;
  uintA deicticRefs;
  calcDeicticRefs(rule, deicticRefs);
  uintA literal_deicticRefs;
  uintA lit_args;
  logicReasoning::getArguments(lit_args, literal);
  setSection(literal_deicticRefs, deicticRefs, lit_args);
  rule.context.memMove = true;
	// if positive always place before neg!
  if (literal.positive) {
		// does not contain deictic refs
    if (literal_deicticRefs.N == 0) {
      rule.context.insert(0, &literal);
    }
		// does contain deictic refs
    else {
      uint pos = 0;
			// check deictic refs
      FOR1D(rule.context, i) {
        if (!rule.context(i)->positive)
          break;
        uintA context_lit_args;
        logicReasoning::getArguments(context_lit_args, *rule.context(i));
        if (numberSharedElements(context_lit_args, literal_deicticRefs) > 0) {
          pos = i+1;
        }
      }
      rule.context.insert(pos, &literal);
    }
  }
  else {
    FOR1D(rule.context, i) {
      if (!rule.context(i)->positive)
        break;
    }
    uint firstNeg = i;
		// does not contain deictic refs
    if (literal_deicticRefs.N == 0) {
      rule.context.insert(firstNeg, &literal);
    }
		// does contain deictic refs
    else {
      uint pos = firstNeg;
			// check deictic refs
      for (i=firstNeg; i<rule.context.N; i++) {
        uintA context_lit_args;
        logicReasoning::getArguments(context_lit_args, *rule.context(i));
        if (numberSharedElements(rule.context(i)->atom->args, literal_deicticRefs) > 0) {
          pos = i+1;
        }
      }
      rule.context.insert(pos, &literal);
    }
  }
}



TL::Rule* TL::ruleReasoning::ground(TL::Rule* r, TL::Substitution* sub) {
  TL::Rule* new_r = new TL::Rule;
	
  uint i;
  FOR1D(r->context, i) {
    new_r->context.append(TL::logicReasoning::applyOriginalSub(*sub, r->context(i)));
  }
	
  new_r->action = TL::logicReasoning::applyOriginalSub(*sub, r->action);
	
  uint j;
  new_r->outcomes.resize(r->outcomes.N);
  FOR1D(r->outcomes, i) {
    new_r->probs.append(r->probs(i));
    FOR1D(r->outcomes(i), j) {
      new_r->outcomes(i).append(TL::logicReasoning::applyOriginalSub(*sub, r->outcomes(i)(j)));
    }
  }
  
  new_r->noise_changes = r->noise_changes;
  new_r->outcome_rewards = r->outcome_rewards;
	
  return new_r;
}

#ifndef RE_fast
// Assumption: Rule arguments have to be different vom Deictic References
void TL::ruleReasoning::ground(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA constants) {
  rules_grounded.clear();
  uint r, c1, c2, i;
  FOR1D_(rules_abstract, r) {
    uintA args, drefs;
    calcTerms(*rules_abstract.elem(r), args);
    boolA inNegatedLiteralsOnly;
    calcDeicticRefs(*r_abs, drefs, inNegatedLiteralsOnly);
    if (inNegatedLiteralsOnly.findValue(true) >= 0) {HALT("disallow negated free DRs");}
    setMinus(args, drefs);
    MT::Array< uintA > combos_args;
    TL::allPossibleLists(combos_args, constants, args.N, true, true);
    FOR1D(combos_args, c1) {
      uintA dref_constants = constants;
      setMinus(dref_constants, combos_args(c1));
      MT::Array< uintA > combos_drefs;
      TL::allPossibleLists(combos_drefs, dref_constants, drefs.N, true, true);
      FOR1D(combos_drefs, c2) {
        TL::Substitution sub;
        FOR1D(args, i) {
          sub.addSubs(args(i), combos_args(c1)(i));
        }
        FOR1D(drefs, i) {
          sub.addSubs(drefs(i), combos_drefs(c2)(i));
        }
        TL::Rule* grounded_rule = ground(rules_abstract.elem(r), &sub);
        if (!stupidContext(*grounded_rule))
          rules_grounded.append(grounded_rule);
        else {
//           cout<<"Bad rule:"<<endl;
//           grounded_rule->write();
        }
      }
    }
  }
  rules_grounded.sort_using_args();
}
#endif
#ifdef RE_fast
// Assumption: Rule arguments have to be different vom Deictic References
void TL::ruleReasoning::ground(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants) {
  uint DEBUG = 0;
  rules_grounded.clear();
  uint r, c1, c2, i, k;
  uint changingDR;
  boolA copyTable_context(15); // assume max 15 context
  boolA copyTable_outcomes(10, 15); // assume max 10 outcomes with max 15 changes each
  uint NUM_REJECTED = 0;
  
  FOR1D_(rules_abstract, r) {
    TL::Rule* r_abs = rules_abstract.elem(r);
    uintA args, drefs;
    calcTerms(*r_abs, args);
    boolA inNegatedLiteralsOnly;
    calcDeicticRefs(*r_abs, drefs, inNegatedLiteralsOnly);
    if (inNegatedLiteralsOnly.findValue(true) >= 0) {HALT("disallow negated free DRs");}
    setMinus(args, drefs);
    if (DEBUG>0) {
      cout<<endl<<"******* New abstract rule:"<<endl; r_abs->write(cout);
      PRINT(args);
      PRINT(drefs);
      PRINT(constants);
    }
    MT::Array< uintA > combos_args;
    TL::allPossibleLists(combos_args, constants, args.N, false, true);
    if (drefs.N > 0) {
      changingDR = drefs(0);
      FOR1D(r_abs->context, i) {
        uintA context_lit_args;
        logicReasoning::getArguments(context_lit_args, *r_abs->context(i));
        if (context_lit_args.findValue(changingDR) < 0)
          copyTable_context(i) = true;
        else
          copyTable_context(i) = false;
      }
      FOR1D(r_abs->outcomes, i) {
        FOR1D(r_abs->outcomes(i), k) {
          if (r_abs->outcomes(i)(k)->atom->args.findValue(changingDR) < 0)
            copyTable_outcomes(i,k) = true;
          else
            copyTable_outcomes(i,k) = false;
        }
      }
    }
    if (DEBUG>0) {PRINT(combos_args);}
    FOR1D(combos_args, c1) {
      if (DEBUG>1) {PRINT(combos_args(c1));}
      // Check types [START]
      FOR1D(r_abs->action->pred->arg_types, i) {
        TL::TermType* arg_type = r_abs->action->pred->arg_types(i);
        TL::TermType* object_type = TL::logicObjectManager::getTermTypeOfObject(combos_args(c1)(i));
        if (DEBUG>1) {
          r_abs->action->write(); cout<<endl;
          cout<<"i="<<i<<"  arg "; arg_type->writeNice();
          cout<<"  with   const ";  object_type->writeNice();
          cout<<"  [combos_args(c1)(i)="<<combos_args(c1)(i)<<"]"<<endl;
        }
        if (!arg_type->subsumes(*object_type)) {
//           cout<<"not subsumed!"<<endl;
          break;
        }
      }
      // Abort if types are inadequate
      if (r_abs->action->pred->arg_types.N > 0   &&  i < combos_args(c1).N) {
//         cout<<"HUHU KILLING me softly"<<endl;
        if (DEBUG>1) {cout<<"Inadequate types"<<endl;}
        continue;
      }
      // Check types [END]
      
      uintA dref_constants = constants;
      setMinus(dref_constants, combos_args(c1));
      MT::Array< uintA > combos_drefs;
      TL::allPossibleLists(combos_drefs, dref_constants, drefs.N, true, true);
      if (DEBUG>0) PRINT(combos_drefs);
      TL::Rule* r_last = NULL;
      FOR1D(combos_drefs, c2) {
        TL::Substitution sub;
        FOR1D(args, i) {
          sub.addSubs(args(i), combos_args(c1)(i));
        }
        FOR1D(drefs, i) {
          sub.addSubs(drefs(i), combos_drefs(c2)(i));
        }
        if (DEBUG>0) {cout<<endl<<"Grounding with: ";sub.write(cout);}
        TL::Rule* r_ground;
        // ACHTUNG! combos_drefs change from left to right 
        //          -->  most recent change = combos_drefs(c2)(0)
        // If there are deictic references, we'll copy from last rule all PTs
        // that do not include the latest DR (the one to change in the new rule).
        if (c2>0  &&  r_last != NULL  &&
            ( drefs.N==1 // safe if only 1 DR
            ||
            combos_drefs(c2)(1) == combos_drefs(c2-1)(1))
            // ensure that second-order changing DR is the same
           ) {
          CHECK(combos_drefs(c2)(0) != combos_drefs(c2-1)(0), "");
          if (DEBUG>0) cout<<"PT-copying"<<endl;
          r_ground = new TL::Rule;
          r_ground->copyBody(*r_abs);
          
          FOR1D(r_abs->context, i) {
            if (DEBUG>0) r_ground->context(i)->write(cout);
            if (copyTable_context(i)) {
              r_ground->context(i) = r_last->context(i);
              if (DEBUG>0) cout<<" -> copied   ";
            }
            else {
              r_ground->context(i) = TL::logicReasoning::applyOriginalSub(sub, r_abs->context(i));
              if (DEBUG>0) cout<<" -> subsed   ";
            }
          }
          
          r_ground->action = r_last->action;
  
          r_ground->outcomes = r_abs->outcomes.resize(r_abs->outcomes.N);
          FOR1D(r_abs->outcomes, i) {
            FOR1D(r_abs->outcomes(i), k) {
              if (DEBUG>0) r_abs->outcomes(i)(k)->write(cout);
              if (copyTable_outcomes(i,k)) {
                r_ground->outcomes(i)(k) = r_last->outcomes(i)(k);
                if (DEBUG>0) cout<<" -> copied   ";
              }
              else {
                r_ground->outcomes(i)(k) = TL::logicReasoning::applyOriginalSub(sub, r_abs->outcomes(i)(k));
                if (DEBUG>0) cout<<" -> subsed   ";
              }
            }
          }
          r_ground->probs = r_abs->probs;
          r_ground->noise_changes = r_abs->noise_changes;
        }
        else {
          r_ground = ground(rules_abstract.elem(r), &sub);
        }
       
        if (DEBUG>0) {cout<<endl<<"Ground rule:"<<endl;r_ground->write(cout);}
        
        if (!stupidContext(*r_ground)) {
          rules_grounded.append(r_ground);
          r_last = r_ground;
          if (DEBUG>0) cout<<"  --> Accepted"<<endl;
        }
        else {
//           cout<<"Bad rule:"<<endl;
//           grounded_rule->write();
          NUM_REJECTED++;
          if (DEBUG>0)  cout<<"  --> Rejected"<<endl;
        }
      }
    }
  }
//   PRINT(NUM_REJECTED);
  rules_grounded.sort_using_args();
}
#endif




void TL::ruleReasoning::ground_with_filtering(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants, const TL::State& s, bool delete_nonchanging_concepts) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"TL::ruleReasoning::ground_with_filtering [START]"<<endl;}
  if (DEBUG>0) {
    cout<<endl<<endl<<endl;
    cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    cout << "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"<<endl;
    cout<< "**************  ATTENTION  ************"<<endl;
    cout << "Assumes that DRs lists change from left to right!!!"<<endl <<endl <<endl <<endl;
  }
  
  // (1) Determine never changing predicate instances and function values
  
  uintA changingIds_preds;
  uintA changingIds_funcs;
  changingConcepts(changingIds_preds, changingIds_funcs, rules_abstract);
  uint i, k;
  
  PredL nonchanging_preds;
  FOR1D(TL::logicObjectManager::p_prim, i) {
    if (changingIds_preds.findValue(TL::logicObjectManager::p_prim(i)->id) < 0)
      nonchanging_preds.append(TL::logicObjectManager::p_prim(i));
  }
  FOR1D(TL::logicObjectManager::p_derived, i) {
    if (changingIds_preds.findValue(TL::logicObjectManager::p_derived(i)->id) < 0)
      nonchanging_preds.append(TL::logicObjectManager::p_derived(i));
  }
  
  FuncL nonchanging_funcs;
  FOR1D(TL::logicObjectManager::f_prim, i) {
    if (changingIds_funcs.findValue(TL::logicObjectManager::f_prim(i)->id) < 0)
      nonchanging_funcs.append(TL::logicObjectManager::f_prim(i));
  }
  FOR1D(TL::logicObjectManager::f_derived, i) {
    if (changingIds_funcs.findValue(TL::logicObjectManager::f_derived(i)->id) < 0)
      nonchanging_funcs.append(TL::logicObjectManager::f_derived(i));
  }
  
  
  LitL nonchanging_pis;
  FOR1D(nonchanging_preds, i) {
    TL::Predicate* p = nonchanging_preds(i);
    MT::Array< uintA > combos;
    allPossibleLists(combos, constants, p->d, true, true);
    FOR1D(combos, k) {
      TL::Literal* lit = TL::logicObjectManager::getLiteral(p, true, combos(k));
      if (TL::logicReasoning::holds(s, lit)) {
        nonchanging_pis.append(lit);
      }
    }
  }
  
  FuncVL nonchanging_fvs;
  FOR1D(nonchanging_funcs, i) {
    TL::Function* f = nonchanging_funcs(i);
    MT::Array< uintA > combos;
    allPossibleLists(combos, constants, f->d, true, true);
    FOR1D(combos, k) {
      TL::FunctionValue* fv = TL::logicObjectManager::getFV(f, combos(k), TL::logicReasoning::getValue(combos(k), f, s));
      nonchanging_fvs.append(fv);
    }
  }
  
   if (DEBUG>0) {
    s.write(cout); cout<<endl;
    cout<<"Never changing predicates:"<<endl;
    writeNice(nonchanging_preds);
    cout<<"Never changing functions:"<<endl;
    writeNice(nonchanging_funcs);
    cout<<"Never changing predicate instances:"<<endl;
    write(nonchanging_pis);
    cout<<endl;
    cout<<"Never changing function values:"<<endl;
    write(nonchanging_fvs);
    cout<<endl;
  }
  
  
  // (2) Ground rules with pruning
  rules_grounded.clear();
  uint r, c1, c2;
  uint latestDR;
  uint MAX_NUM_CONTEXTLITERALS = 20;
  uint MAX_NUM_OUTCOME_PROPERTIES = 20;
  boolA context_containsLatestDR(MAX_NUM_CONTEXTLITERALS); // assume max 15 context
  boolA context_containsDR(MAX_NUM_CONTEXTLITERALS); // assume max 15 context
  boolA outcomes_containsLatestDR(10, MAX_NUM_OUTCOME_PROPERTIES); // assume max 10 outcomes with max 15 changes each
  TL::Rule* r_last = NULL;
  bool valid_action_args;
  bool valid_nonlatest_drefs = true;
  bool valid_all;
  
  uint NUM_REJECTED = 0;
  
  // ABSTRACT RULES
  FOR1D_(rules_abstract, r) {
    TL::Rule* r_abs = rules_abstract.elem(r);
    uintA args, drefs;
    calcTerms(*r_abs, args);
    boolA inNegatedLiteralsOnly;
    calcDeicticRefs(*r_abs, drefs, inNegatedLiteralsOnly);
    if (inNegatedLiteralsOnly.findValue(true) >= 0) {
      cerr<<endl<<endl<<endl<<"Bad abstract rule:"<<endl;  r_abs->write(cerr);  HALT("disallow negated free DRs!");
    }
    setMinus(args, drefs);
    
    // Prepare copying and pruning:  we copy everything except PIs that contain the first DR
    context_containsDR.setUni(false);
    context_containsLatestDR.setUni(false);
    outcomes_containsLatestDR.setUni(false);
    if (drefs.N > 0) {
      latestDR = drefs(0);
      FOR1D(r_abs->context, i) {
        uintA context_lit_args;
        logicReasoning::getArguments(context_lit_args, *r_abs->context(i));
        if (context_lit_args.findValue(latestDR) < 0)
          context_containsLatestDR(i) = false;
        else
          context_containsLatestDR(i) = true;
        if (numberSharedElements(context_lit_args, drefs) == 0)
          context_containsDR(i) = false;
        else
          context_containsDR(i) = true;
      }
      FOR1D(r_abs->outcomes, i) {
        FOR1D(r_abs->outcomes(i), k) {
          if (r_abs->outcomes(i)(k)->atom->args.findValue(latestDR) < 0)
            outcomes_containsLatestDR(i,k) = false;
          else
            outcomes_containsLatestDR(i,k) = true;
        }
      }
    }
    
    if (DEBUG>1) {
      cout<<endl<<"***************************************************"<<endl;
      cout<<endl<<"New abstract rule:"<<endl; r_abs->write(cout);
      PRINT(args);
      PRINT(drefs);
      PRINT(constants);
      cout<<"context_containsDR:"<<endl;
      for (i=0; i<r_abs->context.N; i++) {
        cout<<context_containsDR(i)<<" ";
      }
      cout<<endl;
      PRINT(context_containsDR);
      
      cout<<"context_containsLatestDR:"<<endl;
      for (i=0; i<r_abs->context.N; i++) {
        cout<<context_containsLatestDR(i)<<" ";
      }
      cout<<endl;
      
      cout<<"outcomes_containsLatestDR:"<<endl;
      FOR1D(r_abs->outcomes, i) {
        cout<<i<<":";
        FOR1D(r_abs->outcomes(i), k) {
          cout<<" "<<outcomes_containsLatestDR(i, k);
        }
        cout<<endl;
      }
      cout<<endl;
    }
    
    MT::Array< uintA > combos_args;
//     TL::allPossibleLists(combos_args, constants, args.N, true, true);
    TL::allPossibleLists(combos_args, constants, args.N, false, true);

    // Level 1:  ARGUMENT combos
//     PRINT(combos_args.N);
//     cout<<"Current #ground rules = "<<rules_grounded.num()<<endl;
    FOR1D(combos_args, c1) {
      if (c1%1000 == 0) {
        cout<<"."<<std::flush;
      }
      
      // Check types
      FOR1D(r_abs->action->pred->arg_types, i) {
        TL::TermType* arg_type = r_abs->action->pred->arg_types(i);
        TL::TermType* object_type = TL::logicObjectManager::getTermTypeOfObject(combos_args(c1)(i));
//         cout<<"SDHF: ";
//         r_abs->action->writeNice();  cout<<"  "; arg_type->writeNice();  cout<<endl;
//         PRINT(combos_args(c1)(i));
//         object_type->writeNice();  cout<<endl;
        if (!arg_type->subsumes(*object_type))
          break;
      }
      // Abort if types are inadequate;  but only if types are specified at all
      if (r_abs->action->pred->arg_types.N > 0   &&  i < combos_args(c1).N) {
//         cout<<"HUHU KILLING me softly"<<endl;
        continue;
      }
      
      uintA dref_constants = constants;
      setMinus(dref_constants, combos_args(c1)); // Assumption: DRef different from arguments -- aber nicht beim Regellernen!
      MT::Array< uintA > combos_drefs;
      TL::allPossibleLists(combos_drefs, dref_constants, drefs.N, true, true);
      if (DEBUG>2) {cout<<"------- News action args -------"<<endl; PRINT(combos_args);}
      
      valid_action_args = true;
      r_last = NULL;
      
      // Level 2:  DR combos
      FOR1D(combos_drefs, c2) {
        if (c2>0  &&  drefs.N > 1  &&  combos_drefs(c2)(1) == combos_drefs(c2-1)(1)   &&  !valid_nonlatest_drefs) {
          if (DEBUG>2) cout<<"Omitting combos_drefs="<<combos_drefs(c2)<<endl;
          NUM_REJECTED++;
          continue;
        }
        else
          valid_nonlatest_drefs = true;
        // If we have new non_latest_DR --> cannot copy anymore.
        if (c2>0  &&  drefs.N > 1  &&  combos_drefs(c2)(1) != combos_drefs(c2-1)(1))
          r_last = NULL;

        TL::Substitution sub;
        FOR1D(args, i) {
          sub.addSubs(args(i), combos_args(c1)(i));
        }
        FOR1D(drefs, i) {
          sub.addSubs(drefs(i), combos_drefs(c2)(i));
        }
        if (DEBUG>2) {cout<<endl<<"++++++++++ Next try ++++++++++"<<endl<<"Grounding with: ";sub.write(cout);cout<<endl;}
        TL::Rule* r_ground;
        // (I)   RULE CREATION BY COPYING  
        //       If there are deictic references, we'll copy from last rule all PTs
        //       that do not include the latest DR (the one to change in the new rule),
        //       which is combos_drefs(c2)(0).
        //       Attention:  combos_drefs change from left to right 
        //          -->  most recent change = combos_drefs(c2)(0)
        if (r_last != NULL) {
          CHECK(combos_drefs(c2)(0) != combos_drefs(c2-1)(0), "");
          if (DEBUG>2) cout<<"--- Creation by copying"<<endl;
          r_ground = new TL::Rule;
          r_ground->copyBody(*r_abs);
          
          valid_all = true;
          FOR1D(r_abs->context, i) {
            if (DEBUG>3) r_ground->context(i)->write(cout);
            if (context_containsLatestDR(i)) {
              r_ground->context(i) = TL::logicReasoning::applyOriginalSub(sub, r_abs->context(i));
              if (DEBUG>3) cout<<" -> subsed   ";
              // check whether holds if nonchanging
              if (r_ground->context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
                if (nonchanging_funcs.findValue(((ComparisonAtom*)((TL::ComparisonLiteral*) r_ground->context(i))->atom)->fa1->f) >= 0) {
                  if (!logicReasoning::holds(nonchanging_fvs, ((TL::ComparisonLiteral*) r_ground->context(i)))) {
                    valid_all = false;
                  }
                }
              }
              else {
                if (nonchanging_preds.findValue(r_ground->context(i)->atom->pred) >= 0) {
                  if (!logicReasoning::holds(nonchanging_pis, r_ground->context(i))) {
                    valid_all = false;
                  }
                }
              }
              if (!valid_all) {
                if (DEBUG>3) {cout<<" invalid! ";}
                break;
              }
            }
            else {
              r_ground->context(i) = r_last->context(i);
              if (DEBUG>3) cout<<" -> copied   ";
            }
          }
          
          if (!valid_all  ||  stupidContext(*r_ground)) {
            delete r_ground;
            NUM_REJECTED++;
            if (DEBUG>2) cout<<" --> REJECTED"<<endl;
          }
          else {
            r_ground->action = r_last->action;
    
            r_ground->outcomes = r_abs->outcomes.resize(r_abs->outcomes.N);
            FOR1D(r_abs->outcomes, i) {
              FOR1D(r_abs->outcomes(i), k) {
                if (DEBUG>3) r_abs->outcomes(i)(k)->write(cout);
                if (outcomes_containsLatestDR(i,k)) {
                  r_ground->outcomes(i)(k) = TL::logicReasoning::applyOriginalSub(sub, r_abs->outcomes(i)(k));
                  if (DEBUG>3) cout<<" -> subsed   ";
                }
                else {
                  r_ground->outcomes(i)(k) = r_last->outcomes(i)(k);
                  if (DEBUG>3) cout<<" -> copied   ";
                }
              }
            }
            
            r_ground->noise_changes = r_abs->noise_changes;
            rules_grounded.append(r_ground);
            r_last = r_ground;
            if (DEBUG>2) {cout<<endl<<"Candidate ground rule:"<<endl;r_ground->write(cout);}
            if (DEBUG>2) cout<<" --> ACCEPTED"<<endl;
          }
        }
        // (II)   RULE CREATION BY FULL GROUNDING
        else {
          if (DEBUG>2) cout<<"--- Creation by full grounding"<<endl;
          r_ground = ground(rules_abstract.elem(r), &sub);
          // FILTERING
          // Check whether context 
          //   (A)  that contain only action-arguments
          //   (B)  that contain not the latest DR
          //  are invalid.
          valid_all = true;
          FOR1D(r_ground->context, i) {
            // COMPARISON --> check function
            if (r_ground->context(i)->atom->pred->type == TL::Predicate::predicate_comparison) {
              TL::ComparisonLiteral* clit = (TL::ComparisonLiteral*) r_ground->context(i);
              if (DEBUG>2) {cout<<"CHECKING CPI "; r_ground->context(i)->write(cout); cout<<endl;}
              // (i) Is non-changing function?
              if (nonchanging_funcs.findValue(((ComparisonAtom*)clit->atom)->fa1->f) >= 0) {
                // (ii) Does context hold?
                if (!logicReasoning::holds(nonchanging_fvs, clit)) {
                  valid_all= false;
                  // (iii A) Only action arguments?
                  if (!context_containsDR(i)) {
                    valid_action_args = false;
                    if (DEBUG>2) {cout<<"valid_action_args=false because of CPI "<<i<<" "; r_ground->context(i)->write(cout); cout<<endl;}
                  }
                  // (iii B) Only non_latest_DR arguments?
                  else if (!context_containsLatestDR(i)) {
                    valid_nonlatest_drefs = false;
                    if (DEBUG>2) {cout<<"valid_nonlatest_drefs=false because of CPI "; r_ground->context(i)->write(cout); cout<<endl;}
                  }
                }
              }
            }
            // NOT COMPARISON --> check predicate
            else {
              // (i) Is non-changing predicate?
              if (nonchanging_preds.findValue(r_ground->context(i)->atom->pred) >= 0) {
                // (ii) Does context hold?
                if (!logicReasoning::holds(nonchanging_pis, r_ground->context(i))) {
                  valid_all= false;
                  // (iii A) Only action arguments?
                  if (!context_containsDR(i)) {
                    valid_action_args = false;
                    if (DEBUG>2) {cout<<"valid_action_args=false:  PI "<<i<<" "; r_ground->context(i)->write(cout); cout<<endl;}
                  }
                  // (iii B) Only non_latest_DR arguments?
                  else if (!context_containsLatestDR(i)) {
                    valid_nonlatest_drefs = false;
                    if (DEBUG>2) {cout<<"valid_nonlatest_drefs=false because of CPI "; r_ground->context(i)->write(cout); cout<<endl;}
                  }
                }
              }
            } // End of checking r_ground->context(i)
          } // End of filtering of r_ground->context
          if (DEBUG>2) {cout<<endl<<"Candidate ground rule:"<<endl;r_ground->write(cout);}
          if (DEBUG>2) {PRINT(valid_all);  PRINT(valid_action_args);  PRINT(valid_nonlatest_drefs);}
          if (valid_all  &&  valid_action_args  &&  valid_nonlatest_drefs  &&  !stupidContext(*r_ground)) {
            rules_grounded.append(r_ground);
            r_last = r_ground;
            if (DEBUG>2) cout<<" --> ACCEPTED"<<endl;
          }
          else if (!valid_action_args) {
            delete r_ground;
            NUM_REJECTED += combos_drefs.N - c2;
            if (DEBUG>2) cout<<" --> REJECTED"<<endl;
            break;
          }
          else if (!valid_nonlatest_drefs) { // valid_action_args = true,  valid_nonlatest_drefs = false
            delete r_ground;
            NUM_REJECTED++;
            if (DEBUG>2) cout<<" --> REJECTED"<<endl;
          }
          else {   // valid_all = false  or stupidContext(.)
            delete r_ground;
            NUM_REJECTED++;
            if (DEBUG>2) cout<<" --> REJECTED"<<endl;
          }
        } // End of else for creation of new rule
      } // End of combos_drefs
    } // End of combos_args
  } // End of abstract rules
  
  
  rules_grounded.sort_using_args();
  
  if (delete_nonchanging_concepts) {
    removeNonChangingConcepts(rules_grounded, rules_abstract);
  }
  
  if (DEBUG>0) {
    cout<<"# Accepted rules: "<<rules_grounded.num()<<endl;
    cout<<"# Rejected rules: "<<NUM_REJECTED<<endl;
  }
  if (DEBUG>0) {cout<<"TL::ruleReasoning::ground_with_filtering [END]"<<endl;}
}




void TL::ruleReasoning::removeNonChangingConcepts(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract) {
  uintA changingIds_preds;
  uintA changingIds_funcs;
  changingConcepts(changingIds_preds, changingIds_funcs, rules_abstract);
  uint i, k, l;
  FOR1D_(rules_grounded, i) {
    TL::Rule* r_ground = rules_grounded.elem(i);
    r_ground->context.memMove = true;
    FOR1D_DOWN(r_ground->context, k) {
      if (changingIds_preds.findValue(r_ground->context(k)->atom->pred->id) < 0)
        r_ground->context.remove(k);
    }
    FOR1D(r_ground->outcomes, l) {
      r_ground->outcomes.memMove = true;
      FOR1D_DOWN(r_ground->outcomes(l), k) {
        if (changingIds_preds.findValue(r_ground->outcomes(l)(k)->atom->pred->id) < 0)
          r_ground->outcomes(l).remove(k);
      }
    }
  }
}








  /****************************************
    TRANSITION KNOWLEDGE
  ***************************************/
  
void TL::ruleReasoning::calcSuccessorState(const TL::State& predecessor, const LitL& outcome, TL::State& successor, bool deriveDerived) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcSuccessorState [START]"<<endl;
  uint i, j;
	// keep non-negated literals from predecessor
  FOR1D(predecessor.lits_prim, i) {
    if (DEBUG>2) {
      cout << "(i="<<i<<"):  "<<*predecessor.lits_prim(i)
            <<"  atom="<<*predecessor.lits_prim(i)->atom<<"  p_atom="<<predecessor.lits_prim(i)->atom<<endl;}
    FOR1D(outcome, j) {
      if (DEBUG>2) {
        cout<<"  outcome(j="<<j<<"):  "<<*outcome(j)<<"  atom="<<*outcome(j)->atom<<"   p_atom="<<outcome(j)->atom<<endl;
      }
      if (!outcome(j)->positive && outcome(j)->atom == predecessor.lits_prim(i)->atom)
        break;
    }
    if (j==outcome.N) // add if not found
      successor.lits_prim.append(predecessor.lits_prim(i));
  }
	// add positive literals from outcome
  FOR1D(outcome, i) {
    if (outcome(i)->positive) {
      if (successor.lits_prim.findValue(outcome(i)) < 0)
        successor.lits_prim.append(outcome(i));
    }
  }
  TL::logicReasoning::sort(successor.lits_prim);
  // function values
  // TODO ACHTUNG fv_prim might actually change!!!!!!!!!!!
  // not implemented yet to deal with changing function values!!!
  FOR1D(predecessor.fv_prim, i) {
    successor.fv_prim.append(predecessor.fv_prim(i));
  }
  
  if (deriveDerived)
    TL::logicReasoning::derive(&successor);
  if (DEBUG>0) {
    cout<<"PRE:     "<<predecessor.lits_prim<<endl;
    cout<<"OUTCOME: "<<outcome<<endl;
    cout<<"SUCC:    "<<successor.lits_prim<<endl;
  }
  if (DEBUG>0) cout<<"calcSuccessorState [END]"<<endl;
}


double TL::ruleReasoning::calcSuccessorState(const TL::State& predecessor, TL::Rule* rule, uint& flag, TL::State& successor, bool deriveDerived) {
  //     rule->write();
  uint outcome_id = TL::basic_sample(rule->probs);
  if (outcome_id == rule->outcomes.N - 1) { // noise outcome
    flag = STATE_TRANSITION__NOISE_OUTCOME;
    successor = predecessor;
  }
  else {
    flag = 0;
    ruleReasoning::calcSuccessorState(predecessor, rule->outcomes(outcome_id), successor, deriveDerived);
  }
  if (rule->outcome_rewards.N > 0)
    return rule->outcome_rewards(outcome_id);
  else
    return 0.0;
}


double TL::ruleReasoning::calcSuccessorState(const TL::State& predecessor, const TL::RuleSet& ground_rules, TL::Atom* action, uint& flag, TL::State& successor, bool deriveDerived) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcSuccessorState [START]"<<endl;}
  if (DEBUG>0) {cout<<"Precessor state:"  ; predecessor.write();  cout<<endl;  cout<<"Action:  "<<*action<<endl;}
  TL::RuleSet ground_rules_cov;
  coveringGroundedRules_groundedAction(ground_rules, predecessor, action, ground_rules_cov);
  if (DEBUG>0) {cout<<"Covering rules (#="<<ground_rules_cov.num()<<"):"<<endl;  ground_rules_cov.write();}
  CHECK(ground_rules_cov.num()>0, "No covering rules!");
  if (DEBUG>0) {cout<<"calcSuccessorState [END]"<<endl;}
  if (ground_rules_cov.num() == 2) { // -> unique non-default covering rule
    // ground_rules_cov.elem(0)  =  noisy default rule
    // ground_rules_cov.elem(1)  =  unique non-default covering rule
    return calcSuccessorState(predecessor, ground_rules_cov.elem(1), flag, successor, deriveDerived);
  }
  else {
    return TL_DOUBLE_NIL;
  }
}


double TL::ruleReasoning::probability_groundRule(TL::Rule* r_ground, const TL::State& pre, const TL::State& post, double noiseStateProbability) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"probability [START]"<<endl;
  if (DEBUG>1) {
    r_ground->write(cout);
    cout<<"PRE: ";pre.write(cout);cout<<endl;
    cout<<"POST: ";post.write(cout);cout<<endl;
  }
  CHECK(isGrounded_positives(r_ground), "Rule is not grounded.");
	
  uintA covering_outcomes;
  coveringOutcomes(r_ground, pre, post, covering_outcomes);
  uint o;
  double succ_prob;
  double totalProb = 0.0;
  FOR1D(covering_outcomes, o) {
		// noisy outcome
    if (covering_outcomes(o)==r_ground->outcomes.N-1)
      succ_prob = noiseStateProbability;
    else
      succ_prob = 1.0;
    totalProb += succ_prob * r_ground->probs(covering_outcomes(o));
    if (DEBUG>1) {
      cout<<"Outcome: ";TL::write(r_ground->outcomes(covering_outcomes(o)));
      cout<<" --> P(s'|o,s,a,r)="<<succ_prob<<" * P(o|s,a,r)="<<r_ground->probs(covering_outcomes(o))<<endl;
    }
  }
	
  if (DEBUG>0) PRINT(totalProb);
  if (DEBUG>0) cout<<"probability [END]"<<endl;
	
  return totalProb;
}


double TL::ruleReasoning::probability_abstractRule(TL::Rule* abstractRule, const TL::State& pre, TL::Atom* groundedAction, const TL::State& post, double noiseStateProbability, TL::Substitution* sub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"probability [START]"<<endl;
  CHECK(TL::logicReasoning::isGrounded(groundedAction), "Action is not grounded");
  CHECK(TL::logicReasoning::isAbstract(abstractRule->action), "Rule action has to be abstract!");
	
  TL::SubstitutionSet subs;
  if (sub==NULL) {
    if (cover_rule_groundedAction(pre, groundedAction, abstractRule, subs))
      sub = subs.elem(0);
    else
      return 0.0;
  }
	
  if (DEBUG>1) {cout<<"Sub: ";sub->write(cout);cout<<endl;}
	
  TL::Rule* r_ground = ground(abstractRule, sub);
  double prob = probability_groundRule(r_ground, pre, post, noiseStateProbability);
  delete r_ground;
		
  if (DEBUG>0) PRINT(prob);
  if (DEBUG>0) cout<<"probability [END]"<<endl;
  return prob;
}


double TL::ruleReasoning::probability_defaultRule(TL::Rule* defaultRule, const TL::State& pre, const TL::State& post, double noiseStateProbability) {
  if (pre == post)
    return defaultRule->probs(0) * (100.0 * noiseStateProbability);
  else
    return defaultRule->probs(1) * noiseStateProbability;
}





  /****************************************
    SCORING
   ***************************************/
  
double TL::ruleReasoning::log_likelihood(const RuleSet& rules, const SymbolicExperienceL& experiences, double p_min) {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"log_likelihood [START]"<<endl;}
  double loglik = 0.;
  uint i;
  uint noise_predictions = 0;
  FOR1D(experiences, i) {
    if (DEBUG>1) {cout<<"+++++ Ex "<<i<<" +++++"<<endl;}
    if (DEBUG>1) {cout<<"Ex-ADD: "<<experiences(i)->add<<endl;  cout<<"Ex-DEL: "<<experiences(i)->del<<endl;}
    TL::RuleSet coveringGroundedRules;
    coveringRules_groundedAction(rules, experiences(i)->pre, experiences(i)->action, coveringGroundedRules);
    CHECK(coveringGroundedRules.num() > 0, "at least default rule should cover");
    if (DEBUG>1) {PRINT(coveringGroundedRules.num());}
    TL::Rule* explaining_rule = NULL;
    double lik = 0.;
    // Explain as non-noise
    if (coveringGroundedRules.num() == 2) {
      // erste sollte default regel sein, oder?
      CHECK(coveringGroundedRules.elem(0)->action->pred->id == TL::DEFAULT_ACTION_PRED__ID, "");
      explaining_rule = coveringGroundedRules.elem(1);
      lik = probability_groundRule(explaining_rule, experiences(i)->pre, experiences(i)->post, p_min);
    }
    // Explain as noise
    else {
      explaining_rule = coveringGroundedRules.elem(0);
      CHECK(explaining_rule->action->pred->id == TL::DEFAULT_ACTION_PRED__ID, "");
      lik = probability_defaultRule(explaining_rule, experiences(i)->pre, experiences(i)->post, p_min);
      noise_predictions++;
    }
    if (DEBUG>1) {cout<<"Explaining rule:"<<endl<<*explaining_rule;  PRINT(lik);  PRINT(log(lik));}
    loglik += log(lik);
  }
  if (DEBUG>0) {
    cout<<"Non-noise predictions:  "<<((experiences.N - noise_predictions) * 1.0)/experiences.N<<"  ("<<experiences.N - noise_predictions<<")"<<endl;
    cout<<"Noise predictions:      "<<(noise_predictions * 1.0)/experiences.N<<"  ("<<noise_predictions<<")"<<endl;
    PRINT(loglik);
  }
  if (DEBUG>0) {cout<<"log_likelihood [END]"<<endl;}
  return loglik;
}



double TL::ruleReasoning::score(const RuleSet& rules, const SymbolicExperienceL& experiences, double p_min, double alpha_pen) {
  uint DEBUG  =1;
  if (DEBUG>0) {cout<<"score [START]"<<endl;}
  // Loglik
  double loglik = log_likelihood(rules, experiences, p_min);
  if (DEBUG>0) {PRINT(loglik);}
  // Penalty
  double penalty = 0.0;
  uint i;
  FOR1D_(rules, i) {
    penalty += TL::ruleReasoning::numLiterals(*rules.elem(i));
  }
  penalty *= alpha_pen * (-1.0);
  if (DEBUG>0) {PRINT(penalty);}
  // Score
  double score = loglik + penalty;
  if (DEBUG>0) {PRINT(score);}
  if (DEBUG>0) {cout<<"score [END]"<<endl;}
  return score;
}

  
  
  
  /****************************************
    COVERAGE
  ***************************************/

bool TL::ruleReasoning::cover_context(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs, TL::Substitution* actionSub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover_context [START]"<<endl;
  CHECK(actionSub!=NULL, "Action substitution must be provided!");
  CHECK(actionSub->num() <= rule->action->args.N, "");
  if (DEBUG>0) {
    rule->write(cout);
    s.write(cout);cout<<endl;
    cout<<"Action sub: ";actionSub->write(cout);cout<<endl;
  }
  CHECK(subs.num()==0, "Already subs given!");
  CHECK(s.derivedDerived, "p_derived haven't been p_derived");
  CHECK(TL::logicReasoning::calcNumTerms(*(rule->action))==actionSub->num(), "Incomplete actionSub.");
  uintA actionSub_outs;
  actionSub->getOuts(actionSub_outs);
  //  The deep question of QUANTIFICATION with NEGATED FREE DEICTIC VARS
  //  Example rule:
  //    ACTION:  act(X)   CONTEXT:  p2(X), -p1(Y)
  //  Two possibilities:
  //   (i)  Representing all quantification:  \forall Y: -p1(Y)
  //        --> BAD BAAAAAAAAAAD  Don't do that!
  //            This is like introducing a formula "for all constants, the respective predicate does not hold".
  //   (ii) Representing exists quantification:  \exists Y: -p1(Y)
  //        --> does not make intuitive sense
  //            Example:   ACTION:  puton(X)    CONTEXT:  -on(Y,X)
  //                       Rule applicable if exactly one arbitrary object not on X
  //   Baseline: Should be forbidden in NID rules, in general.
  //             However, if there comes such a case nonetheless, we use exists quantification.
  bool mightbecovering = TL::logicReasoning::cover(s, rule->context, subs, false, actionSub);
  uint i, j;
  if (DEBUG>0) {
    PRINT(mightbecovering);
    cout<<"Substitutions (#="<<subs.num()<<"):"<<endl;
    FOR1D_(subs, i) {
      subs.elem(i)->write(cout);cout<<endl;
    }
    PRINT(actionSub_outs);
  }
  bool covering;
// 	// NON-DEICTIC [start]
// 	covering = mightbecovering;
// 	// NON-DEICTIC [end]
	// DEICTIC VARIANT [start]
  if (mightbecovering) {
    uintA deicticRefs;
    boolA inNegatedLiteralsOnly;
    calcDeicticRefs(*rule, deicticRefs, inNegatedLiteralsOnly);
    if (DEBUG>0) {PRINT(deicticRefs);}
    uint dref__subs_id;
    covering = true;
    // Filter substitutions
    FOR1D_DOWN_(subs, j) {
      FOR1D(deicticRefs, i) {
        dref__subs_id = subs.elem(j)->getSubs(deicticRefs(i));
        CHECK_(TL::logicReasoning::isConstant(dref__subs_id),
               {rule->write(cout);  s.write(cout); cout<<endl;  cout<<"Action sub: "; actionSub->write(cout);cout<<endl;
                PRINT(dref__subs_id);  PRINT(deicticRefs);
                cout<<"subs(j="<<j<<"):  ";  subs.elem(j)->write();  cout<<endl;},
                "No substitution for deictic reference.");
        // Deictic references must be different from action arguments
        if (actionSub_outs.findValue(dref__subs_id) >= 0) {
          if (DEBUG>0) {cout<<"Removing subs(j="<<j<<")  as  DR=" << dref__subs_id << " is action arg"<<endl;}
          subs.remove(j);
          break;
        }
      }
    }
    if (DEBUG>0) {PRINT(subs.num());}
    covering = subs.num() == 1;
  }
  else
    covering = false;
	// DEICTIC VARIANT [end]
  if (DEBUG>0) PRINT(covering);
  if (DEBUG>0) cout<<"cover_context [END]"<<endl;
  return covering;
}

bool TL::ruleReasoning::cover_rule_groundedAction(const TL::State& s, const TL::Atom* groundedAction, const TL::Rule* rule, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
//   if (groundedAction == TL::logicObjectManager::getLiteral("puton(60") &&  isAbstract(*rule)   
//     &&  rule->context.N > 0  &&   rule->context(0) == TL::logicObjectManager::getLiteral("-on(1 0)"))
//     DEBUG = 5;
  if (DEBUG>0) cout<<"cover_rule_groundedAction [START]"<<endl;
  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!");
  CHECK(groundedAction->pred->type == TL::Predicate::predicate_action, "No action, man!");
  if (DEBUG>0) {cout<<"STATE:  "; s.write(cout); cout<<endl;  cout<<"GROUNDED ACTION:  "<<*groundedAction<<endl; cout<<"RULE:"<<endl; rule->write(cout);
    cout<<"SUBS (N="<<subs.num()<<"):"<<endl;  uint i; FOR1D_(subs, i) {cout<<"("<<i<<")  ";  subs.elem(i)->write();}}
	// applied world knowledge: default rule always covers
  if (rule->action->pred->id == TL::DEFAULT_ACTION_PRED__ID) {
    if (subs.num() == 0)
      subs.append(new TL::Substitution);
    return true;
  }
  // doNothing rule
  if (TL::logicObjectManager::getAtom_doNothing() == rule->action) {
    if (rule->action == groundedAction)
      return true;
    else
      return false;
  }
  CHECK(isAbstract(*rule), "Rule has to be abstract:  "<<endl<<*rule);
  TL::Substitution* actionSub = new TL::Substitution;
  bool covers;
  if (TL::logicReasoning::unify(groundedAction, rule->action, *actionSub))
    // ###### ESSENTIAL CALL
    covers = cover_context(s, rule, subs, actionSub);
  else
    covers = false;
  delete actionSub;
	
  if (DEBUG>0) {
    cout<<"covers? "<<covers;
    if(covers){cout<<"  Sub: ";subs.elem(0)->write(cout);}
    cout<<endl;
  }
  if (DEBUG>0) cout<<"cover_rule_groundedAction [END]"<<endl;
  return covers;
}


bool TL::ruleReasoning::cover_rule(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover_rule [START]"<<endl;
  if (DEBUG>0) {
    /*cout<<"Rule:"<<endl;*/rule->write(cout);
    cout<<"State: ";s.write(cout);cout<<endl;
  }
  MT::Array< uintA > actionObjectLists;
  allPossibleLists(actionObjectLists, TL::logicObjectManager::constants, rule->action->pred->d, true, true);
  uint i;
  FOR1D(actionObjectLists, i) {
    TL::Atom* groundedAction = TL::logicObjectManager::getAtom(rule->action->pred, actionObjectLists(i));
    TL::SubstitutionSet subs_inner;
    if (cover_rule_groundedAction(s, groundedAction, rule, subs_inner)) {
      subs.append(subs_inner);
    }
  }
  if (DEBUG>0) {cout<<"Covers: "<<(subs.num()>0)<<endl;}
  if (DEBUG>1) {
    FOR1D_(subs,i) {
      subs.elem(i)->write(cout);cout<<endl;
    }
  }
  if (DEBUG>0) cout<<"cover_rule [END]"<<endl;
  return subs.num()>0;
}


bool TL::ruleReasoning::cover_groundRule_groundedAction(const TL::State& s, const TL::Atom* groundedAction, const TL::Rule* ground_rule) {
  if (ground_rule->action->pred->id == TL::DEFAULT_ACTION_PRED__ID)  // noisy default rule always covers
    return true;
  if (groundedAction != ground_rule->action)
    return false;
  return logicReasoning::holds(s, ground_rule->context);
}


void TL::ruleReasoning::coveringRules(const TL::RuleSet& allRules, const TL::State& s, TL::RuleSet& r_grounds) {
  r_grounds.clear();
  uint i, k;
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule(s, allRules.elem(i), subs)) {
      FOR1D_(subs, k) {
        r_grounds.append(ground(allRules.elem(i), subs.elem(k)));
      }
    }
  }
}


void TL::ruleReasoning::coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::Atom* groundedAction, TL::RuleSet& r_grounds) {
  r_grounds.clear();
  uint i, k;
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule_groundedAction(s, groundedAction, allRules.elem(i), subs)) {
      FOR1D_(subs, k) {
        r_grounds.append(ground(allRules.elem(i), subs.elem(k)));
      }
    }
  }
}

void TL::ruleReasoning::coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::Atom* groundedAction, uintA& coveringRuleIDs) {
  coveringRuleIDs.clear();
  uint i;	
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule_groundedAction(s, groundedAction, allRules.elem(i), subs))
      coveringRuleIDs.append(i);
  }
}


void TL::ruleReasoning::coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::Atom* groundedAction, uintA& coveringRuleIDs) {
  coveringRuleIDs.clear();
  uint i; 
  for (i=0; i<allGroundedRules.num(); i++) {
    if (cover_groundRule_groundedAction(s, groundedAction, allGroundedRules.elem(i)))
      coveringRuleIDs.append(i);
  }
}



void TL::ruleReasoning::coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::Atom* groundedAction, TL::RuleSet& coveringGroundedRules) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"coveringGroundedRules [START]"<<endl;}
  coveringGroundedRules.clear();
  uint i;
  for (i=0; i<allGroundedRules.num(); i++) {
    if (DEBUG>0) {cout<<"Candidate rule:"<<endl;  allGroundedRules.elem(i)->write();}
    if (cover_groundRule_groundedAction(s, groundedAction, allGroundedRules.elem(i))) {
      coveringGroundedRules.append(allGroundedRules.elem(i));
      if (DEBUG>0) {cout<<"--> Covers"<<endl;}
    }
  }
  if (DEBUG>0) {cout<<"coveringGroundedRules [END]"<<endl;}
}


void TL::ruleReasoning::coveringRules(uintA& coveringRulesIDs, const TL::RuleSet& abstract_rules, const AtomL& ground_actions, const TL::State& s) {
  coveringRulesIDs.resize(ground_actions.N);
  uint i;
  FOR1D(ground_actions, i) {
    uintA ids_covering_rules__action;
    TL::ruleReasoning::coveringRules_groundedAction(abstract_rules, s, ground_actions(i), ids_covering_rules__action);
    if (ids_covering_rules__action.N == 2)
      coveringRulesIDs(i) = ids_covering_rules__action(1);
    else
      coveringRulesIDs(i) = 0;   // default rule
  }
}


void TL::ruleReasoning::coveringGroundedRules(uintA& coveringRulesIDs, const TL::RuleSet& ground_rules, const AtomL& ground_actions, const TL::State& s) {
  coveringRulesIDs.resize(ground_actions.N);
  uint i;
  FOR1D(ground_actions, i) {
    uintA ids_covering_rules__action;
    TL::ruleReasoning::coveringGroundedRules_groundedAction(ground_rules, s, ground_actions(i), ids_covering_rules__action);
    if (ids_covering_rules__action.N == 2)
      coveringRulesIDs(i) = ids_covering_rules__action(1);
    else
      coveringRulesIDs(i) = 0;   // default rule
  }
}


uint TL::ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::Atom* groundedAction) {
  uintA coveringGroundedRules_ids;
  coveringGroundedRules_groundedAction(allGroundedRules, s, groundedAction, coveringGroundedRules_ids);
  if (coveringGroundedRules_ids.N == 2) {
    CHECK(coveringGroundedRules_ids(0) == 0, "first rule should be noisy default rule");
    return coveringGroundedRules_ids(1);
  }
  else
    return 0;
}


uint TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::Atom* groundedAction) {
  uintA coveringRuleIDs;
  coveringRules_groundedAction(allRules, s, groundedAction, coveringRuleIDs);
  if (coveringRuleIDs.N == 2) {
    CHECK(coveringRuleIDs(0) == 0, "first rule should be noisy default rule");
    return coveringRuleIDs(1);
  }
  else
    return 0;
}



void TL::ruleReasoning::coveringOutcomes(TL::Rule* r_ground, const TL::State& pre, const TL::State& post, uintA& covering_outcomes) {
  covering_outcomes.clear();
  uint o;
  FOR1D(r_ground->outcomes, o) {
		// noisy outcome
    if (o==r_ground->outcomes.N-1) {
      covering_outcomes.append(o);
    }	
    else {
      TL::State succ;
      calcSuccessorState(pre, r_ground->outcomes(o), succ, false);
      if (succ == post)
        covering_outcomes.append(o);
    }
  }
}



void TL::ruleReasoning::coveringOutcomes(TL::Rule* abstractRule, const TL::State& pre, TL::Atom* groundedAction, const TL::State& post, uintA& covering_outcomes) {
  CHECK(TL::logicReasoning::isAbstract(abstractRule->action), "Rule action has to be abstract!");
  TL::SubstitutionSet subs;
  cover_rule_groundedAction(pre, groundedAction, abstractRule, subs);
  CHECK(subs.num()==1, "rule coverage only in case of exactly one sub");
  TL::Rule* r_ground = ground(abstractRule, subs.elem(0));
  coveringOutcomes(r_ground, pre, post, covering_outcomes);
}




void TL::ruleReasoning::calcGroundDeicticReferences(uintA& ground_drefs, const TL::State& state, const TL::Atom* groundedAction, const TL::Rule* rule) {
  ground_drefs.clear();
  
  uintA drefs;
  boolA drefs_inNegatedLiteralsOnly;
  TL::ruleReasoning::calcDeicticRefs(*rule, drefs, drefs_inNegatedLiteralsOnly);
  
  TL::SubstitutionSet subs;
  bool covers = TL::ruleReasoning::cover_rule_groundedAction(state, groundedAction, rule, subs);
  if ((!covers) || ((rule->action->args.N > 0 || drefs.N > 0)  &&  subs.num() != 1)) {
    cerr<<endl<<endl<<endl;
    cerr<<"Bad rule:"<<endl;  rule->write(cerr);
    cerr<<"groundedAction="<<*groundedAction<<endl;
    if (!covers) HALT("Rule does not cover!");
    HALT("Bad grounded action: subs.num()="<<subs.num());
  }

  uint i;
  FOR1D(drefs, i) {
    ground_drefs.append(subs.elem(0)->getSubs(drefs(i)));
  }
}




 /****************************************
    SPECIAL RULES
 ***************************************/


TL::Rule* TL::ruleReasoning::generateDefaultRule(double noiseProb, double minProb, double change) {
  TL::Rule* newDefaultRule = new TL::Rule;
  uintA args_empty;
  newDefaultRule->action = TL::logicObjectManager::getAtom(TL::logicObjectManager::getPredicate(TL::DEFAULT_ACTION_PRED__ID), args_empty);
  LitL sameOutcome;
  newDefaultRule->outcomes.append(sameOutcome);
  LitL noiseOutcome;
  newDefaultRule->outcomes.append(noiseOutcome);
  if (noiseProb < minProb)
    noiseProb = minProb;
  else if (1.-noiseProb < minProb)
    noiseProb = 1.-minProb;
  newDefaultRule->probs.append(1-noiseProb);
  newDefaultRule->probs.append(noiseProb);
  newDefaultRule->noise_changes = change;
  return newDefaultRule;
}



bool TL::ruleReasoning::isDefaultRule(TL::Rule* rule) {
  return rule->context.N == 0  &&  rule->action->pred->id == TL::DEFAULT_ACTION_PRED__ID;
}



TL::Rule* TL::ruleReasoning::getDoNothingRule() {
  TL::Rule* r = new TL::Rule;
  r->action = TL::logicObjectManager::getAtom_doNothing();
  r->outcomes.resize(2);
  r->probs.resize(2);
  r->probs(0) = 1.0;
  r->probs(1) = 0.0;
  r->noise_changes = 0.0;
  return r;
}






