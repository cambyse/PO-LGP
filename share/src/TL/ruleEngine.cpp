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

#include "ruleEngine.h"
#include <stdlib.h>

#define RE_fast 1

TL::LogicEngine* re_le;


void TL::RuleEngine::setLogicEngine(TL::LogicEngine* _le) {
  re_le = _le;
}


  /****************************************
    BASIC HELPERS
  ***************************************/


void TL::RuleEngine::calcDeicticRefs(const TL::Rule& rule, uintA& drefs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcDeicticRefs [START]"<<endl;
  drefs.clear();
  uint i, j;
  FOR1D(rule.context, i) {
    FOR1D(rule.context(i)->args, j) {
      if (rule.action->args.findValue(rule.context(i)->args(j)) < 0)
        drefs.setAppend(rule.context(i)->args(j));
    }
  }
  // sort them
  TL::sort_asc(drefs);
  
  if (DEBUG>0) {
    rule.writeNice(cout);
    PRINT(drefs)
  }
  if (DEBUG>0) cout<<"calcDeicticRefs [END]"<<endl;
}


void TL::RuleEngine::calcDeicticRefs(const TL::Rule& rule, uintA& drefs, boolA& inNegatedLiteralsOnly) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcDeicticRefs2 [START]"<<endl;
  calcDeicticRefs(rule, drefs);
  inNegatedLiteralsOnly.resize(drefs.N);
  inNegatedLiteralsOnly.setUni(true);
  uint r, p, i;
  FOR1D(drefs, r) {
    FOR1D(rule.context, p) {
      FOR1D(rule.context(p)->args, i) {
        if (rule.context(p)->args.findValue(drefs(r)) >= 0)
          if (rule.context(p)->positive) {
          inNegatedLiteralsOnly(r) = false;
          break;
          }
      }
      if (!inNegatedLiteralsOnly(r))
        break;
    }
  }
  if (DEBUG>0) {
    PRINT(drefs)
        PRINT(inNegatedLiteralsOnly)
  }
  if (DEBUG>0) cout<<"calcDeicticRefs2 [END]"<<endl;
}


void TL::RuleEngine::DRcontext(const TL::Rule& rule, boolA& findValueDR) {
  findValueDR.resize(rule.context.N);
  findValueDR.setUni(false);
  uint i,k;
  FOR1D(rule.context, i) {
    FOR1D(rule.context(i)->args, k) {
      if (rule.action->args.findValue(rule.context(i)->args(k)) < 0) {
        findValueDR(i) = true;
        break;
      }
    }
  }
}





void TL::RuleEngine::calcTerms(const TL::Rule& rule, uintA& terms) {
  terms.clear();
  uint i, j;
  FOR1D(rule.action->args, i) {
    terms.setAppend(rule.action->args(i));
  }
  FOR1D(rule.context, j) {
    FOR1D(rule.context(j)->args, i) {
      terms.setAppend(rule.context(j)->args(i));
    }
  }
  TL::sort_asc(terms);
}


void TL::RuleEngine::calcAbsentLiterals(const TL::Rule& rule, PredIA& literals, bool positiveOnly) {
  literals.clear();
  uintA terms;
  calcTerms(rule, terms);
  PredIA literals_cands;
  re_le->generateAllPossiblePredicateInstances(literals_cands, terms, positiveOnly);
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


bool TL::RuleEngine::usesPI(const TL::Rule& rule, TL::PredicateInstance* pi) {
  uint i, k;
  FOR1D(rule.context, i) {
    if (rule.context(i) == pi)
      return true;
  }
  if (rule.action == pi)
    return true;
  FOR1D(rule.outcomes, i) {
    FOR1D(rule.outcomes(i), k) {
      if (rule.outcomes(i)(k) == pi)
        return true;
    }
  }
  return false;
}


void TL::RuleEngine::changingConcepts(uintA& changingIds_p, uintA& changingIds_f, const TL::RuleSet& rules) {
  changingIds_p.clear();
  changingIds_f.clear();
  uint r, o, i;
  uintA changingIds_p_prim, changingIds_f_prim;
  FOR1D_(rules, r) {
    FOR1D(rules.elem(r)->outcomes, o) {
      FOR1D(rules.elem(r)->outcomes(o), i) {
        if (rules.elem(r)->outcomes(o)(i)->pred->type == TL_PRED_COMPARISON) {
          changingIds_f_prim.setAppend(((TL::ComparisonPredicateInstance*) rules.elem(r)->outcomes(o)(i))->f->id);
        }
        else {
          changingIds_p_prim.setAppend(rules.elem(r)->outcomes(o)(i)->pred->id);
        }
      }
    }
  }
  changingIds_p.append(changingIds_p_prim);
  changingIds_f.append(changingIds_f_prim);
  // also include p_derived ones
  FOR1D(changingIds_p_prim, i) {
    PredA p_suc;
    FuncA f_suc;
    re_le->dependencyGraph.getAllSuccessors(*re_le->dependencyGraph.getPredicate(changingIds_p_prim(i)), p_suc, f_suc);
    FOR1D(p_suc, o) {changingIds_p.setAppend(p_suc(o)->id);}
    FOR1D(f_suc, o) {changingIds_f.setAppend(f_suc(o)->id);}
  }
  FOR1D(changingIds_f_prim, i) {
    PredA p_suc;
    FuncA f_suc;
    re_le->dependencyGraph.getAllSuccessors(*re_le->dependencyGraph.getFunction(changingIds_f_prim(i)), p_suc, f_suc);
    FOR1D(p_suc, o) {changingIds_p.setAppend(p_suc(o)->id);}
    FOR1D(f_suc, o) {changingIds_f.setAppend(f_suc(o)->id);}
  }
}


uint TL::RuleEngine::numPredicateInstances(const TL::Rule& rule) {
  uint num = 0;
  uint i;
  num += rule.context.N;
  FOR1D(rule.outcomes, i) {
    num += rule.outcomes(i).N;
  }
  return num;
}


bool TL::RuleEngine::stupidContext(const TL::Rule& rule) {
  // TODO needs more refinement
  // (1) checks that dynamic comparison pts don't use same arguments
  // for both sides of comparison
  uint p,i;
  FOR1D(rule.context, p) {
    if (rule.context(p)->pred->type == TL_PRED_COMPARISON) {
      TL::ComparisonPredicateInstance* cpt = (TL::ComparisonPredicateInstance*) rule.context(p);
      if (!cpt->hasConstantBound()) {
        CHECK(cpt->args.N % 2 == 0, "strange slot assignments");
        bool different = false;
        for (i=0; i<cpt->args.N / 2; i++) {
          if (cpt->args(i) != cpt->args(i+cpt->args.N / 2)) {
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



bool TL::RuleEngine::isGrounded(TL::Rule* r) {
  uint i, j;
  FOR1D(r->context, i) {
    if (!re_le->isGrounded(r->context(i)))
      return false;
  }
  if (!re_le->isGrounded(r->action))
    return false;
  FOR1D(r->outcomes, i) {
    FOR1D(r->outcomes(i), j) {
      if (!re_le->isGrounded(r->outcomes(i)(j)))
        return false;
    }
  }
  return true;
}


bool TL::RuleEngine::isGrounded_positives(TL::Rule* r) {
  uint i, j;
  FOR1D(r->context, i) {
    if (r->context(i)->positive  &&  !re_le->isGrounded(r->context(i)))
      return false;
  }
  if (!re_le->isGrounded(r->action))
    return false;
  FOR1D(r->outcomes, i) {
    FOR1D(r->outcomes(i), j) {
      if (!re_le->isGrounded(r->outcomes(i)(j)))
        return false;
    }
  }
  return true;
}

void TL::RuleEngine::removeDoublePredicateInstances(const TL::RuleSet& ground_rules) {
  int DEBUG = 0;
  uint i, k, l, m;
  TL::Rule* r;
  FOR1D_(ground_rules, i) {
    r = ground_rules.elem(i);
    if (DEBUG>0) {
      cout<<"BEFORE:";
      r->writeNice(cout);
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
      r->writeNice(cout);
    }
  }
}


void TL::RuleEngine::checkRules(const TL::RuleSet& rules) {
  uint i, k;
  TL::Rule* r;
  FOR1D_(rules, i) {
    r = rules.elem(i);
    // (1) Check no double in context
    if (r->context.containsDoubles()) {
      r->writeNice(cerr);
      HALT("Context contains doubles");
    }
    // (2) Check no double in outcomes
    FOR1D(r->outcomes, k) {
      if (r->outcomes(k).containsDoubles()) {
        r->writeNice(cerr);
        HALT("Outcome " << k << " contain doubles");
      }
    }
    // (3) Check probs sum to 1
    if (fabs(sum(r->probs) - 1.0) > 0.001) {
      r->writeNice(cerr);
      printf("%10.10f\n", sum(r->probs));
      HALT("bad probs for rule");
    }
    // (4) Check no pure comparison predicate in context or outcomes
    FOR1D(r->context, k) {
      PredicateInstance* pi = dynamic_cast< PredicateInstance* >(r->context(k));
      ComparisonPredicateInstance* cpi = dynamic_cast< ComparisonPredicateInstance* >(r->context(k));
      if (cpi == NULL  &&  pi != NULL  &&  pi->pred->type == TL_PRED_COMPARISON) {
        r->writeNice(cerr);
        HALT("Comparison predicate used as normal Predicate Instance (Context literal #" << k << " )");
      }
    }
  }
}



void TL::RuleEngine::cleanup(TL::Rule& rule) {
  uint i;
  // clean-up drefs
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
      PredIA new_context;
      sub.apply(rule.context, new_context);
      rule.context = new_context;
      for (i=0; i<rule.outcomes.N-1; i++) {
        PredIA new_outcome;
        sub.apply(rule.outcomes(i), new_outcome);
        rule.outcomes(i) = new_outcome;
      }
//       cout<<endl<<endl<<"Nachher:"<<endl; rule.writeNice();
    }
  }
  // order context
  TL::LogicEngine::order(rule.context);
  // order outcomes
  for (i=0; i<rule.outcomes.N-1; i++) {
    TL::LogicEngine::order(rule.outcomes(i));
  }
}



/****************************************
   READ & WRITE
  ***************************************/


void TL::RuleEngine::writeNice(const TL::RuleSet& rs, ostream& os, bool breaks) {
  uint k;
  os << "Rule-set:" << endl;
  for (k=0; k<rs.num(); k++) {
    os << "[" << k << "]" << endl;
    rs.elem(k)->writeNice(os);
    if (breaks) os<<endl;
  }
}


void TL::RuleEngine::readRules(const char* filename, RuleSet& rules) {
  TL::readRules(filename, re_le->p_prim, re_le->p_derived, re_le->p_comp, re_le->actions, re_le->f_prim, re_le->f_derived, rules);
  // Technicality to ensure that read rules use the same PredicateInstance etc. objects.
  // (These objects are managed by the LogicEngine object accessed by the RuleEngine.)
  uint i;
  FOR1D_(rules, i) {
    TL::RuleEngine::makeOriginal(*rules.elem(i));
  }
  rules.sort();
}




// PLAIN RULE FILES


void readPlainPIs(PredIA& pis, MT::String& line) {
  uint DEBUG = 0;
  if (DEBUG>0) {PRINT(line);}
  pis.clear();
  while (MT::peerNextChar(line) != -1) {
    MT::String text;
    text.read(line,NULL,")\n");
    if (text.N() == 0  ||  text(0) == '\n')
      break;
    text << ")";
    pis.append(re_le->getPI(text.p));
    if (DEBUG>0) {pis.last()->writeNice(cout);cout<<endl;}
    MT::skip(line, ",");
    MT::skip(line, " ");
  }
  if (DEBUG>0) {writeNice(pis);cout<<endl;}
}

TL::Rule* readRulePlain(ifstream& in) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readRulePlain [START]"<<endl;}
  
  CHECK(in.is_open(), "Input stream ain't open!");
  TL::Rule* r = new TL::Rule;
  
  MT::String line;
  
  // Context
  line.read(in, NULL, "\n"); // PRE:
  CHECK((line(0)=='P' && line(1)=='R')  ||  (line(0)=='C' && line(1)=='O'),"bad context");   CHECK(line.N() < 8, "bad context");
  if (DEBUG>2) PRINT(line);
  line.read(in, NULL, "\n");
  if (DEBUG>2) PRINT(line);
  readPlainPIs(r->context, line);
  
  // Action
  line.read(in, NULL, "\n"); // ACTION:
  CHECK(line(0)=='A',"bad action");   CHECK(line.N() < 10, "bad action: too short");
  if (DEBUG>2) PRINT(line);
  line.read(in, NULL, "\n");
  if (DEBUG>2) PRINT(line);
  PredIA actions_wrapper;
  readPlainPIs(actions_wrapper, line);
  r->action = actions_wrapper(0);
  
  // Outcomes
  if (DEBUG>0) {cout<<"Reading the outcomes:"<<endl;}
  line.read(in, NULL, "\n"); // POST:
  CHECK((line(0)=='P' && line(1)=='O') ||  (line(0)=='O' && line(1)=='U'),"bad outcomes:  "<<line);   CHECK(line.N() < 11, "bad outcomes due to length");
  MT::Array< PredIA > outcomes;
  while ( MT::peerNextChar(in) != ' '  &&  MT::peerNextChar(in) != 'P'  &&   MT::peerNextChar(in) != 'C'  &&  MT::peerNextChar(in) != 'R') {
    line.read(in, NULL, "=\n");
    if (DEBUG>2) PRINT(line);
    if (line.N()<=2) {
      cout<<"bad line: "<<endl;
      PRINT(line);
      HALT("");
    }
    double prob;
    line >> prob;
    r->probs.append(prob);
    PredIA outcome;
    readPlainPIs(outcome, line);
    r->outcomes.append(outcome);
    if (MT::skip(in) == -1)
      break;
    //  mglw. noch reward fuers outcome einlesen
    if (MT::peerNextChar(in) == 'R') {
      while (r->outcome_rewards.N < r->outcomes.N-1) {  // possibly fill for previous unrewarded outcomes
        r->outcome_rewards.append(0.);
      }
      MT::skipUntil(in, " ");
      double outcome_reward;
      in >> outcome_reward;
      r->outcome_rewards.append(outcome_reward);
      if (MT::skip(in) == -1)
        break;
    }
  }
  PredIA dummy_noise_outcome;
  r->outcomes.append(dummy_noise_outcome);
  r->noise_changes = 0.;
  r->probs.append(1.0 - sum(r->probs));
  if (r->outcome_rewards.N > 0)
    r->outcome_rewards.append(0.);
  
  if (DEBUG>0) {
    cout<<"Neue Regel:"<<endl;
    r->writeNice(cout);
  }
  
  if (DEBUG>0) {cout<<"readRulePlain [END]"<<endl;}
  return r;
}

void TL::RuleEngine::readRulesPlain(const char* filename, RuleSet& rules) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readRulesPlain [START]"<<endl;}
  rules.clear();
  ifstream in(filename);
//   PRINT(filename);
  if (!in.is_open()) {
    cerr<<"Rule-file " << filename << " can't be opened!"<<endl;
    HALT("");
  }
  rules.append(generateDefaultRule(0.));
  while (MT::skip(in) != -1) {
    rules.append(readRulePlain(in));
  }
  // Technicality to ensure that read rules use the same PredicateInstance etc. objects.
  // (These objects are managed by the LogicEngine object accessed by the RuleEngine.)
  uint i;
  FOR1D_(rules, i) {
    TL::RuleEngine::makeOriginal(*rules.elem(i));
  }
  rules.sort();
  if (DEBUG>0) {cout<<"readRulesPlain [END]"<<endl;}
}





void writeRulePlain(ofstream& out, const TL::Rule& rule, bool skip_noise_outcome) {
  uint i, k;
  out<<"PRE:"<<endl;
  FOR1D(rule.context, i) {
    if (i>0)
      out<<", ";
    rule.context(i)->writeNice(out);
  }
  out << endl;
  out<<"ACTION:"<<endl;
  rule.action->writeNice(out);
  out << endl;
  out<<"POST:"<<endl;
  FOR1D(rule.outcomes, i) {
    // skip noise outcome
    if (skip_noise_outcome && i == rule.outcomes.N-1)
        break;
    out<<rule.probs(i)<<" ";
    FOR1D(rule.outcomes(i), k) {
      if (k>0)
        out<<", ";
      rule.outcomes(i)(k)->writeNice(out);
    }
    if (i<rule.outcome_rewards.N  && !TL::isZero(rule.outcome_rewards(i)))
      out<< "   = REWARD " << rule.outcome_rewards(i);
    out<<endl;
  }
}


void TL::RuleEngine::writeRulesPlain(const char* filename, const RuleSet& rules, bool skip_noise_outcome) {
  ofstream out(filename);
//   PRINT(filename);
  CHECK(out.is_open(), "Rule-file " << filename << " can't be opened!");
  uint i;
  FOR1D_(rules, i) {
    if (i==0) // skip default rule
      continue;
    writeRulePlain(out, *rules.elem(i), skip_noise_outcome);
    out << endl;
  }
  out.close();
}




  /****************************************
    RULE MANIPULATION
  ***************************************/


void TL::RuleEngine::insert(TL::Rule& rule, TL::PredicateInstance& literal) {
  uint i;
  uintA deicticRefs;
  calcDeicticRefs(rule, deicticRefs);
  uintA literal_deicticRefs;
  setSection(literal_deicticRefs, deicticRefs, literal.args);
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
        if (numberSharedElements(rule.context(i)->args, literal_deicticRefs) > 0) {
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
        if (numberSharedElements(rule.context(i)->args, literal_deicticRefs) > 0) {
          pos = i+1;
        }
      }
      rule.context.insert(pos, &literal);
    }
  }
}



TL::Rule* TL::RuleEngine::ground(TL::Rule* r, TL::Substitution* sub) {
  TL::Rule* new_r = new TL::Rule;
	
  uint i;
  FOR1D(r->context, i) {
    new_r->context.append(re_le->applyOriginalSub(*sub, r->context(i)));
  }
	
  new_r->action = re_le->applyOriginalSub(*sub, r->action);
	
  uint j;
  new_r->outcomes.resize(r->outcomes.N);
  FOR1D(r->outcomes, i) {
    new_r->probs.append(r->probs(i));
    FOR1D(r->outcomes(i), j) {
      new_r->outcomes(i).append(re_le->applyOriginalSub(*sub, r->outcomes(i)(j)));
    }
  }
  
  new_r->noise_changes = r->noise_changes;
  new_r->outcome_rewards = r->outcome_rewards;
	
  return new_r;
}

#ifndef RE_fast
// Assumption: Rule arguments have to be different vom Deictic References
void TL::RuleEngine::ground(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA constants) {
  rules_grounded.clear();
  uint r, c1, c2, i;
  FOR1D_(rules_abstract, r) {
    uintA args, drefs;
    calcTerms(*rules_abstract.elem(r), args);
    calcDeicticRefs(*rules_abstract.elem(r), drefs);
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
//           grounded_rule->writeNice();
        }
      }
    }
  }
  rules_grounded.sort_using_args();
}
#endif
#ifdef RE_fast
// Assumption: Rule arguments have to be different vom Deictic References
void TL::RuleEngine::ground(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants) {
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
    calcDeicticRefs(*r_abs, drefs);
    setMinus(args, drefs);
    if (DEBUG>0) {
      cout<<endl<<"******* New abstract rule:"<<endl; r_abs->writeNice(cout);
      PRINT(args);
      PRINT(drefs);
      PRINT(constants);
    }
    MT::Array< uintA > combos_args;
    TL::allPossibleLists(combos_args, constants, args.N, false, true);
    if (drefs.N > 0) {
      changingDR = drefs(0);
      FOR1D(r_abs->context, i) {
        if (r_abs->context(i)->args.findValue(changingDR) < 0)
          copyTable_context(i) = true;
        else
          copyTable_context(i) = false;
      }
      FOR1D(r_abs->outcomes, i) {
        FOR1D(r_abs->outcomes(i), k) {
          if (r_abs->outcomes(i)(k)->args.findValue(changingDR) < 0)
            copyTable_outcomes(i,k) = true;
          else
            copyTable_outcomes(i,k) = false;
        }
      }
    }
    FOR1D(combos_args, c1) {
      // Check types [START]
      FOR1D(r_abs->action->pred->arg_types, i) {
        TL::TermType* arg_type = r_abs->action->pred->arg_types(i);
        TL::TermType* object_type = re_le->getTermTypeOfObject(combos_args(c1)(i));
//         cout<<"SDHF: ";
//         r_abs->action->writeNice(); cout<<endl;
//         cout<<"i="<<i<<"  arg "; arg_type->writeNice();
//         cout<<"  with   const ";  object_type->writeNice();
//         cout<<"  [combos_args(c1)(i)="<<combos_args(c1)(i)<<"]"<<endl;
        if (!arg_type->subsumes(*object_type)) {
//           cout<<"not subsumed!"<<endl;
          break;
        }
      }
      // Abort if types are inadequate
      if (i < combos_args(c1).N) {
//         cout<<"HUHU KILLING me softly"<<endl;
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
        if (DEBUG>0) {cout<<endl<<"Grounding with: ";sub.writeNice(cout);}
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
            if (DEBUG>0) r_ground->context(i)->writeNice(cout);
            if (copyTable_context(i)) {
              r_ground->context(i) = r_last->context(i);
              if (DEBUG>0) cout<<" -> copied   ";
            }
            else {
              r_ground->context(i) = re_le->applyOriginalSub(sub, r_abs->context(i));
              if (DEBUG>0) cout<<" -> subsed   ";
            }
          }
          
          r_ground->action = r_last->action;
  
          r_ground->outcomes = r_abs->outcomes.resize(r_abs->outcomes.N);
          FOR1D(r_abs->outcomes, i) {
            FOR1D(r_abs->outcomes(i), k) {
              if (DEBUG>0) r_abs->outcomes(i)(k)->writeNice(cout);
              if (copyTable_outcomes(i,k)) {
                r_ground->outcomes(i)(k) = r_last->outcomes(i)(k);
                if (DEBUG>0) cout<<" -> copied   ";
              }
              else {
                r_ground->outcomes(i)(k) = re_le->applyOriginalSub(sub, r_abs->outcomes(i)(k));
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
       
        if (DEBUG>0) {cout<<endl<<"Ground rule:"<<endl;r_ground->writeNice(cout);}
        
        if (!stupidContext(*r_ground)) {
          rules_grounded.append(r_ground);
          r_last = r_ground;
          if (DEBUG>0) cout<<"  --> Accepted"<<endl;
        }
        else {
//           cout<<"Bad rule:"<<endl;
//           grounded_rule->writeNice();
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




void TL::RuleEngine::ground_with_filtering(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract, const uintA& constants, const TL::State& s, bool delete_nonchanging_concepts) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"TL::RuleEngine::ground_with_filtering [START]"<<endl;}
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
  
  PredA nonchanging_preds;
  FOR1D(re_le->p_prim, i) {
    if (changingIds_preds.findValue(re_le->p_prim(i)->id) < 0)
      nonchanging_preds.append(re_le->p_prim(i));
  }
  FOR1D(re_le->p_derived, i) {
    if (changingIds_preds.findValue(re_le->p_derived(i)->id) < 0)
      nonchanging_preds.append(re_le->p_derived(i));
  }
  
  FuncA nonchanging_funcs;
  FOR1D(re_le->f_prim, i) {
    if (changingIds_funcs.findValue(re_le->f_prim(i)->id) < 0)
      nonchanging_funcs.append(re_le->f_prim(i));
  }
  FOR1D(re_le->f_derived, i) {
    if (changingIds_funcs.findValue(re_le->f_derived(i)->id) < 0)
      nonchanging_funcs.append(re_le->f_derived(i));
  }
  
  
  PredIA nonchanging_pis;
  FOR1D(nonchanging_preds, i) {
    TL::Predicate* p = nonchanging_preds(i);
    MT::Array< uintA > combos;
    allPossibleLists(combos, constants, p->d, true, true);
    FOR1D(combos, k) {
      TL::PredicateInstance* pi = re_le->getPI(p, true, combos(k));
      if (re_le->holds(s, pi)) {
        nonchanging_pis.append(pi);
      }
    }
  }
  
  FuncVA nonchanging_fvs;
  FOR1D(nonchanging_funcs, i) {
    TL::Function* f = nonchanging_funcs(i);
    MT::Array< uintA > combos;
    allPossibleLists(combos, constants, f->d, true, true);
    FOR1D(combos, k) {
      TL::FunctionValue* fv = re_le->getFV(f, combos(k), TL::LogicEngine::getValue(combos(k), f, s));
      nonchanging_fvs.append(fv);
    }
  }
  
   if (DEBUG>0) {
    s.writeNice(cout); cout<<endl;
    cout<<"Never changing predicates:"<<endl;
    writeNice(nonchanging_preds);
    cout<<"Never changing functions:"<<endl;
    writeNice(nonchanging_funcs);
    cout<<"Never changing predicate instances:"<<endl;
    writeNice(nonchanging_pis);
    cout<<endl;
    cout<<"Never changing function values:"<<endl;
    writeNice(nonchanging_fvs);
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
    calcDeicticRefs(*r_abs, drefs);
    setMinus(args, drefs);
    
    // Prepare copying and pruning:  we copy everything except PIs that contain the first DR
    if (drefs.N > 0) {
      latestDR = drefs(0);
      FOR1D(r_abs->context, i) {
        if (r_abs->context(i)->args.findValue(latestDR) < 0)
          context_containsLatestDR(i) = false;
        else
          context_containsLatestDR(i) = true;
        if (numberSharedElements(r_abs->context(i)->args, drefs) == 0)
          context_containsDR(i) = false;
        else
          context_containsDR(i) = true;
      }
      FOR1D(r_abs->outcomes, i) {
        FOR1D(r_abs->outcomes(i), k) {
          if (r_abs->outcomes(i)(k)->args.findValue(latestDR) < 0)
            outcomes_containsLatestDR(i,k) = false;
          else
            outcomes_containsLatestDR(i,k) = true;
        }
      }
    }
    
    if (DEBUG>1) {
      cout<<endl<<"***************************************************"<<endl;
      cout<<endl<<"New abstract rule:"<<endl; r_abs->writeNice(cout);
      PRINT(args);
      PRINT(drefs);
      PRINT(constants);
      cout<<"context_containsDR:"<<endl;
      for (i=0; i<r_abs->context.N; i++) {
        cout<<context_containsDR(i)<<" ";
      }
      cout<<endl;
      
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
    if (DEBUG>2) {PRINT(combos_args);}
//     PRINT(combos_args.N);
//     cout<<"Current #ground rules = "<<rules_grounded.num()<<endl;
    FOR1D(combos_args, c1) {
      if (c1%1000 == 0) {
        cout<<"."<<std::flush;
      }
      
      // Check types
      if (r_abs->action->pred->arg_types.N > 0) {
        FOR1D(r_abs->action->pred->arg_types, i) {
          TL::TermType* arg_type = r_abs->action->pred->arg_types(i);
          TL::TermType* object_type = re_le->getTermTypeOfObject(combos_args(c1)(i));
  //         cout<<"SDHF: ";
  //         r_abs->action->writeNice();  cout<<"  "; arg_type->writeNice();  cout<<endl;
  //         PRINT(combos_args(c1)(i));
  //         object_type->writeNice();  cout<<endl;
          if (!arg_type->subsumes(*object_type))
            break;
        }
        // Abort if types are inadequate
        if (i < combos_args(c1).N) {
  //         cout<<"HUHU KILLING me softly"<<endl;
          continue;
        }
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
        if (DEBUG>2) {cout<<endl<<"++++++++++ Next try ++++++++++"<<endl<<"Grounding with: ";sub.writeNice(cout);cout<<endl;}
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
            if (DEBUG>3) r_ground->context(i)->writeNice(cout);
            if (context_containsLatestDR(i)) {
              r_ground->context(i) = re_le->applyOriginalSub(sub, r_abs->context(i));
              if (DEBUG>3) cout<<" -> subsed   ";
              // check whether holds if nonchanging
              if (r_ground->context(i)->pred->type == TL_PRED_COMPARISON) {
                if (nonchanging_funcs.findValue(((TL::ComparisonPredicateInstance*) r_ground->context(i))->f) >= 0) {
                  if (!LogicEngine::holds(nonchanging_fvs, ((TL::ComparisonPredicateInstance*) r_ground->context(i)))) {
                    valid_all = false;
                  }
                }
              }
              else {
                if (nonchanging_preds.findValue(r_ground->context(i)->pred) >= 0) {
                  if (!LogicEngine::holds(nonchanging_pis, r_ground->context(i))) {
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
                if (DEBUG>3) r_abs->outcomes(i)(k)->writeNice(cout);
                if (outcomes_containsLatestDR(i,k)) {
                  r_ground->outcomes(i)(k) = re_le->applyOriginalSub(sub, r_abs->outcomes(i)(k));
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
            if (DEBUG>2) {cout<<endl<<"Candidate ground rule:"<<endl;r_ground->writeNice(cout);}
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
            if (r_ground->context(i)->pred->type == TL_PRED_COMPARISON) {
              TL::ComparisonPredicateInstance* cpi = (TL::ComparisonPredicateInstance*) r_ground->context(i);
              if (DEBUG>2) {cout<<"CHECKING CPI "; r_ground->context(i)->writeNice(cout); cout<<endl;}
              // (i) Is non-changing function?
              if (nonchanging_funcs.findValue(cpi->f) >= 0) {
                // (ii) Does context hold?
                if (!LogicEngine::holds(nonchanging_fvs, cpi)) {
                  valid_all= false;
                  // (iii A) Only action arguments?
                  if (!context_containsDR(i)) {
                    valid_action_args = false;
                    if (DEBUG>2) {cout<<"valid_action_args=false because of CPI "<<i<<" "; r_ground->context(i)->writeNice(cout); cout<<endl;}
                  }
                  // (iii B) Only non_latest_DR arguments?
                  else if (!context_containsLatestDR(i)) {
                    valid_nonlatest_drefs = false;
                    if (DEBUG>2) {cout<<"valid_nonlatest_drefs=false because of CPI "; r_ground->context(i)->writeNice(cout); cout<<endl;}
                  }
                }
              }
            }
            // NOT COMPARISON --> check predicate
            else {
              // (i) Is non-changing predicate?
              if (nonchanging_preds.findValue(r_ground->context(i)->pred) >= 0) {
                // (ii) Does context hold?
                if (!LogicEngine::holds(nonchanging_pis, r_ground->context(i))) {
                  valid_all= false;
                  // (iii A) Only action arguments?
                  if (!context_containsDR(i)) {
                    valid_action_args = false;
                    if (DEBUG>2) {cout<<"valid_action_args=false:  PI "<<i<<" "; r_ground->context(i)->writeNice(cout); cout<<endl;}
                  }
                  // (iii B) Only non_latest_DR arguments?
                  else if (!context_containsLatestDR(i)) {
                    valid_nonlatest_drefs = false;
                    if (DEBUG>2) {cout<<"valid_nonlatest_drefs=false because of CPI "; r_ground->context(i)->writeNice(cout); cout<<endl;}
                  }
                }
              }
            } // End of checking r_ground->context(i)
          } // End of filtering of r_ground->context
          if (DEBUG>2) {cout<<endl<<"Candidate ground rule:"<<endl;r_ground->writeNice(cout);}
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
  if (DEBUG>0) {cout<<"TL::RuleEngine::ground_with_filtering [END]"<<endl;}
}




void TL::RuleEngine::removeNonChangingConcepts(TL::RuleSet& rules_grounded, const TL::RuleSet& rules_abstract) {
  uintA changingIds_preds;
  uintA changingIds_funcs;
  changingConcepts(changingIds_preds, changingIds_funcs, rules_abstract);
  uint i, k, l;
  FOR1D_(rules_grounded, i) {
    TL::Rule* r_ground = rules_grounded.elem(i);
    r_ground->context.memMove = true;
    FOR1D_DOWN(r_ground->context, k) {
      if (changingIds_preds.findValue(r_ground->context(k)->pred->id) < 0)
        r_ground->context.remove(k);
    }
    FOR1D(r_ground->outcomes, l) {
      r_ground->outcomes.memMove = true;
      FOR1D_DOWN(r_ground->outcomes(l), k) {
        if (changingIds_preds.findValue(r_ground->outcomes(l)(k)->pred->id) < 0)
          r_ground->outcomes(l).remove(k);
      }
    }
  }
}






void TL::RuleEngine::makeOriginal(TL::Rule& r) {
  uint i, k;
  FOR1D(r.context, i) {
    r.context(i) = re_le->getPIorig(r.context(i));
  }
  r.action = re_le->getPIorig(r.action);
  FOR1D(r.outcomes, i) {
    FOR1D(r.outcomes(i), k) {
      r.outcomes(i)(k) = re_le->getPIorig(r.outcomes(i)(k));
    }
  }
}








  /****************************************
    TRANSITION KNOWLEDGE
  ***************************************/
  
void TL::RuleEngine::calcSuccessorState(const TL::State& precessor, const PredIA& outcome, TL::State& successor, bool deriveDerived) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"calcSuccessorState [START]"<<endl;
  uint i, j;
	// keep non-negated literals from precessor
  FOR1D(precessor.pi_prim, i) {
    TL::PredicateInstance currentNeg = *precessor.pi_prim(i);
    currentNeg.positive = !precessor.pi_prim(i)->positive;
    FOR1D(outcome, j) {
      if (currentNeg == *outcome(j))
        break;
    }
    if (j==outcome.N)
      successor.pi_prim.append(precessor.pi_prim(i));
  }
	// add positive literals from outcome
  FOR1D(outcome, i) {
    if (outcome(i)->positive) {
      if (successor.pi_prim.findValue(outcome(i)) < 0)
        successor.pi_prim.append(outcome(i));
    }
  }
    // function values
    // TODO ACHTUNG fValues might actually change!!!!!!!!!!!
  // not implemented yet to deal with changing function values!!!
  FOR1D(precessor.fv_prim, i) {
    successor.fv_prim.append(precessor.fv_prim(i));
  }
  if (deriveDerived)
    re_le->derive(&successor);
  if (DEBUG>0) {
    cout<<"PRE: "; TL::writeNice(precessor.pi_prim); cout<<endl;
    cout<<"OUTCOME: "; TL::writeNice(outcome); cout<<endl;
    cout<<"SUCC: "; TL::writeNice(successor.pi_prim); cout<<endl;
  }
  if (DEBUG>0) cout<<"calcSuccessorState [END]"<<endl;
}


double TL::RuleEngine::calcSuccessorState(const TL::State& precessor, TL::Rule* rule, uint& flag, TL::State& successor, bool deriveDerived) {
  //     rule->writeNice();
  uint outcome_id = TL::basic_sample(rule->probs);
  if (outcome_id == rule->outcomes.N - 1) { // noise outcome
    flag = STATE_TRANSITION__NOISE_OUTCOME;
    successor = precessor;
  }
  else {
    flag = 0;
    RuleEngine::calcSuccessorState(precessor, rule->outcomes(outcome_id), successor, deriveDerived);
  }
  if (rule->outcome_rewards.N > 0)
    return rule->outcome_rewards(outcome_id);
  else
    return 0.0;
}


double TL::RuleEngine::calcSuccessorState(const TL::State& precessor, const TL::RuleSet& ground_rules, TL::PredicateInstance* action, uint& flag, TL::State& successor, bool deriveDerived) {
  TL::RuleSet ground_rules_cov;
  coveringGroundedRules_groundedAction(ground_rules, precessor, action, ground_rules_cov);
  CHECK(ground_rules_cov.num()>0, "No covering rules!");
  if (ground_rules_cov.num() == 2) { // -> unique non-default covering rule
    // ground_rules_cov.elem(0)  =  noisy default rule
    // ground_rules_cov.elem(1)  =  unique non-default covering rule
    return calcSuccessorState(precessor, ground_rules_cov.elem(1), flag, successor, deriveDerived);
  }
  else
    return TL_DOUBLE_NIL;
}


double TL::RuleEngine::probability_groundRule(TL::Rule* groundedRule, const TL::State& pre, const TL::State& post, double noiseStateProbability) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"probability [START]"<<endl;
  if (DEBUG>1) {
    groundedRule->writeNice(cout);
    cout<<"PRE: ";pre.writeNice(cout);cout<<endl;
    cout<<"POST: ";post.writeNice(cout);cout<<endl;
  }
  CHECK(isGrounded_positives(groundedRule), "Rule is not grounded.");
	
  uintA covering_outcomes;
  coveringOutcomes(groundedRule, pre, post, covering_outcomes);
  uint o;
  double succ_prob;
  double totalProb = 0.0;
  FOR1D(covering_outcomes, o) {
		// noisy outcome
    if (covering_outcomes(o)==groundedRule->outcomes.N-1)
      succ_prob = noiseStateProbability;
    else
      succ_prob = 1.0;
    totalProb += succ_prob * groundedRule->probs(covering_outcomes(o));
    if (DEBUG>1) {
      cout<<"Outcome: ";TL::writeNice(groundedRule->outcomes(covering_outcomes(o)));
      cout<<" --> P(s'|o,s,a,r)="<<succ_prob<<" * P(o|s,a,r)="<<groundedRule->probs(covering_outcomes(o))<<endl;
    }
  }
	
  if (DEBUG>0) PRINT(totalProb);
  if (DEBUG>0) cout<<"probability [END]"<<endl;
	
  return totalProb;
}


double TL::RuleEngine::probability_abstractRule(TL::Rule* abstractRule, const TL::State& pre, TL::PredicateInstance* groundedAction, const TL::State& post, double noiseStateProbability, TL::Substitution* sub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"probability [START]"<<endl;
  CHECK(re_le->isGrounded(groundedAction), "Action is not grounded");
  CHECK(re_le->isAbstract(abstractRule->action), "Rule action has to be abstract!");
	
  TL::SubstitutionSet subs;
  if (sub==NULL) {
    if (cover_rule_groundedAction(pre, groundedAction, abstractRule, subs))
      sub = subs.elem(0);
    else
      return 0.0;
  }
	
  if (DEBUG>1) {cout<<"Sub: ";sub->writeNice(cout);cout<<endl;}
	
  TL::Rule* groundedRule = ground(abstractRule, sub);
  double prob = probability_groundRule(groundedRule, pre, post, noiseStateProbability);
  delete groundedRule;
		
  if (DEBUG>0) PRINT(prob);
  if (DEBUG>0) cout<<"probability [END]"<<endl;
  return prob;
}


// TODO das ist hier nicht sauber: die default rule sollte in der zukunft nur das Noise-Outcome haben
double TL::RuleEngine::probability_defaultRule(TL::Rule* defaultRule, const TL::State& pre, const TL::State& post, double noiseStateProbability) {
  if (pre == post)
    return defaultRule->probs(0) * (100.0 * noiseStateProbability);
  else
    return defaultRule->probs(1) * noiseStateProbability;
}




  /****************************************
    COVERAGE
  ***************************************/

bool TL::RuleEngine::cover_context(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs, TL::Substitution* actionSub) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover_context [START]"<<endl;
  CHECK(s.derivedDerived, "p_derived haven't been p_derived");
  CHECK(subs.num()==0, "Already subs given!");
  CHECK(actionSub!=NULL, "Action substitution must be provided!");
  CHECK(re_le->calcNumTerms(*(rule->action))==actionSub->num(), "Incomplete actionSub.");
  if (DEBUG>0) {
    rule->writeNice(cout);
    s.writeNice(cout);cout<<endl;
    cout<<"Action sub: ";actionSub->writeNice(cout);cout<<endl;
  }
  uintA actionSub_outs;
  actionSub->getOuts(actionSub_outs);
// rule coverage always requires free variables to be all existentially quantified (--> deictic references)
  bool mightbecovering = re_le->cover(s, rule->context, subs, false, actionSub);
  uint i, j;
  if (DEBUG>0) {
    cout<<"Substitutions (#="<<subs.num()<<"):"<<endl;
    FOR1D_(subs, i) {
      subs.elem(i)->writeNice(cout);cout<<endl;
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
    uint dref__subs_id;
    covering = true;
    FOR1D(deicticRefs, i) {
      uintA dref__substituting_ids;
      FOR1D_DOWN_(subs, j) {
        dref__subs_id = subs.elem(j)->getSubs(deicticRefs(i));
        CHECK(re_le->isConstant(dref__subs_id), "No substitution for deictic reference.");
        CHECK(actionSub->num() <= rule->action->args.N, "");
        // Deictic references must be different from action arguments
        if (actionSub_outs.findValue(dref__subs_id) < 0) {
          dref__substituting_ids.setAppend(dref__subs_id);
        }
        else {
          if (DEBUG>0) {cout<<"Removing subs(j="<<j<<")  as  DR=" << dref__subs_id << " is action arg"<<endl;}
          subs.remove(j);
        }
      }
      if (DEBUG>0) {
        cout<<"Deictic Var. " << deicticRefs(i)<<": "<<dref__substituting_ids<<endl;
      }
      if (dref__substituting_ids.N != 1) {
        covering = false;
        // delete substitutions
        subs.clear();
        break;
      }
    }
  }
  else
    covering = false;
	// DEICTIC VARIANT [end]
  if (DEBUG>0) PRINT(covering);
  if (DEBUG>0) cout<<"cover_context [END]"<<endl;
  return covering;
}


bool TL::RuleEngine::cover_rule_groundedAction(const TL::State& s, TL::PredicateInstance* groundedAction, const TL::Rule* rule, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover_rule_groundedAction [START]"<<endl;
  CHECK(subs.num()==0, "Already subs given, Aldaaaaaa!");
  CHECK(groundedAction->pred->type == TL_PRED_ACTION, "No action, man!");
  if (DEBUG>0) {s.writeNice(cout); cout<<endl; groundedAction->writeNice(cout); cout<<endl; rule->writeNice(cout);}
	// applied world knowledge: default rule always covers
  if (rule->action->pred->id == TL_DEFAULT_ACTION_PRED__ID  && rule->context.N == 0) {
    if (subs.num() == 0)
      subs.append(new TL::Substitution);
    return true;
  }
  TL::Substitution* actionSub = new TL::Substitution;
  bool covers;
  if (re_le->unify(groundedAction, rule->action, *actionSub))
    // ###### ESSENTIAL CALL
    covers = cover_context(s, rule, subs, actionSub);
  else
    covers = false;
  delete actionSub;
	
  if (DEBUG>0) {
    cout<<"covers? "<<covers;
    if(covers){cout<<"  Sub: ";subs.elem(0)->writeNice(cout);}
    cout<<endl;
  }
  if (DEBUG>0) cout<<"cover_rule_groundedAction [END]"<<endl;
  return covers;
}


bool TL::RuleEngine::cover_rule(const TL::State& s, const TL::Rule* rule, TL::SubstitutionSet& subs) {
  uint DEBUG = 0;
  if (DEBUG>0) cout<<"cover_rule [START]"<<endl;
  if (DEBUG>0) {
    /*cout<<"Rule:"<<endl;*/rule->writeNice(cout);
    cout<<"State: ";s.writeNice(cout);cout<<endl;
  }
  MT::Array< uintA > actionObjectLists;
  allPossibleLists(actionObjectLists, re_le->constants, rule->action->pred->d, true, true);
  uint i;
  FOR1D(actionObjectLists, i) {
    TL::PredicateInstance* groundedAction = re_le->getPI(rule->action->pred, true, actionObjectLists(i));
    TL::SubstitutionSet subs_inner;
    if (cover_rule_groundedAction(s, groundedAction, rule, subs_inner)) {
      subs.append(subs_inner);
    }
  }
  if (DEBUG>0) {cout<<"Covers: "<<(subs.num()>0)<<endl;}
  if (DEBUG>1) {
    FOR1D_(subs,i) {
      subs.elem(i)->writeNice(cout);cout<<endl;
    }
  }
  if (DEBUG>0) cout<<"cover_rule [END]"<<endl;
  return subs.num()>0;
}


bool TL::RuleEngine::cover_groundRule_groundedAction(const TL::State& s, TL::PredicateInstance* groundedAction, const TL::Rule* ground_rule) {
  if (ground_rule->action->pred->id == TL_DEFAULT_ACTION_PRED__ID)  // noisy default rule always covers
    return true;
  if (groundedAction != ground_rule->action)
    return false;
  return LogicEngine::holds(s, ground_rule->context);
}


void TL::RuleEngine::coveringRules(const TL::RuleSet& allRules, const TL::State& s, TL::RuleSet& groundedRules) {
  groundedRules.clear();
  uint i, k;
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule(s, allRules.elem(i), subs)) {
      FOR1D_(subs, k) {
        groundedRules.append(ground(allRules.elem(i), subs.elem(k)));
      }
    }
  }
}


void TL::RuleEngine::coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction, TL::RuleSet& groundedRules) {
  groundedRules.clear();
  uint i, k;
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule_groundedAction(s, groundedAction, allRules.elem(i), subs)) {
      FOR1D_(subs, k) {
        groundedRules.append(ground(allRules.elem(i), subs.elem(k)));
      }
    }
  }
}

void TL::RuleEngine::coveringRules_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction, uintA& coveringRuleIDs) {
  coveringRuleIDs.clear();
  uint i;	
  for (i=0; i<allRules.num(); i++) {
    TL::SubstitutionSet subs;
    if (cover_rule_groundedAction(s, groundedAction, allRules.elem(i), subs))
      coveringRuleIDs.append(i);
  }
}


void TL::RuleEngine::coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction, uintA& coveringRuleIDs) {
  coveringRuleIDs.clear();
  uint i; 
  for (i=0; i<allGroundedRules.num(); i++) {
    if (cover_groundRule_groundedAction(s, groundedAction, allGroundedRules.elem(i)))
      coveringRuleIDs.append(i);
  }
}



void TL::RuleEngine::coveringGroundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction, TL::RuleSet& coveringGroundedRules) {
  coveringGroundedRules.clear();
  uint i;
  for (i=0; i<allGroundedRules.num(); i++) {
    if (cover_groundRule_groundedAction(s, groundedAction, allGroundedRules.elem(i))) {
      coveringGroundedRules.append(allGroundedRules.elem(i));
    }
  }
}


void TL::RuleEngine::coveringRules(uintA& coveringRulesIDs, const TL::RuleSet& abstract_rules, const PredIA& ground_actions, const TL::State& s) {
  coveringRulesIDs.resize(ground_actions.N);
  uint i;
  FOR1D(ground_actions, i) {
    uintA ids_covering_rules__action;
    TL::RuleEngine::coveringRules_groundedAction(abstract_rules, s, ground_actions(i), ids_covering_rules__action);
    if (ids_covering_rules__action.N == 2)
      coveringRulesIDs(i) = ids_covering_rules__action(1);
    else
      coveringRulesIDs(i) = 0;   // default rule
  }
}


void TL::RuleEngine::coveringGroundedRules(uintA& coveringRulesIDs, const TL::RuleSet& ground_rules, const PredIA& ground_actions, const TL::State& s) {
  coveringRulesIDs.resize(ground_actions.N);
  uint i;
  FOR1D(ground_actions, i) {
    uintA ids_covering_rules__action;
    TL::RuleEngine::coveringGroundedRules_groundedAction(ground_rules, s, ground_actions(i), ids_covering_rules__action);
    if (ids_covering_rules__action.N == 2)
      coveringRulesIDs(i) = ids_covering_rules__action(1);
    else
      coveringRulesIDs(i) = 0;   // default rule
  }
}


TL::Rule* TL::RuleEngine::uniqueCoveringRule_groundedRules_groundedAction(const TL::RuleSet& allGroundedRules, const TL::State& s, TL::PredicateInstance* groundedAction) {
  TL::RuleSet coveringGroundedRules;
  coveringGroundedRules_groundedAction(allGroundedRules, s, groundedAction, coveringGroundedRules);
  if (coveringGroundedRules.num() == 2) {
    CHECK(coveringGroundedRules.elem(0)->action->pred->id == TL_DEFAULT_ACTION_PRED__ID, "first rule should be noisy default rule");
    return coveringGroundedRules.elem(1);
  }
  else
    return NULL;
}


uint TL::RuleEngine::uniqueAbstractCoveringRule_groundedAction(const TL::RuleSet& allRules, const TL::State& s, TL::PredicateInstance* groundedAction) {
  uintA coveringRuleIDs;
  coveringRules_groundedAction(allRules, s, groundedAction, coveringRuleIDs);
  if (coveringRuleIDs.N == 2) {
    CHECK(coveringRuleIDs(0) == 0, "first rule should be noisy default rule");
    return coveringRuleIDs(1);
  }
  else
    return 0;
}



void TL::RuleEngine::coveringOutcomes(TL::Rule* groundedRule, const TL::State& pre, const TL::State& post, uintA& covering_outcomes) {
  covering_outcomes.clear();
  uint o;
  FOR1D(groundedRule->outcomes, o) {
		// noisy outcome
    if (o==groundedRule->outcomes.N-1) {
      covering_outcomes.append(o);
    }	
    else {
      TL::State succ;
      calcSuccessorState(pre, groundedRule->outcomes(o), succ, false);
      if (succ == post)
        covering_outcomes.append(o);
    }
  }
}



void TL::RuleEngine::coveringOutcomes(TL::Rule* abstractRule, const TL::State& pre, TL::PredicateInstance* groundedAction, const TL::State& post, uintA& covering_outcomes) {
  CHECK(re_le->isAbstract(abstractRule->action), "Rule action has to be abstract!");
  TL::SubstitutionSet subs;
  cover_rule_groundedAction(pre, groundedAction, abstractRule, subs);
  CHECK(subs.num()==1, "rule coverage only in case of exactly one sub");
  TL::Rule* groundedRule = ground(abstractRule, subs.elem(0));
  coveringOutcomes(groundedRule, pre, post, covering_outcomes);
}




void TL::RuleEngine::calcGroundDeicticReferences(uintA& ground_drefs, const TL::State& state, TL::PredicateInstance* groundedAction, TL::Rule* rule) {
  ground_drefs.clear();
  
  uintA drefs;
  boolA drefs_inNegatedLiteralsOnly;
  TL::RuleEngine::calcDeicticRefs(*rule, drefs, drefs_inNegatedLiteralsOnly);
  
  TL::SubstitutionSet subs;
  TL::RuleEngine::cover_rule_groundedAction(state, groundedAction, rule, subs);
  if (subs.num() != 1) {
    HALT("Bad grounded action: subs.num()="<<subs.num());
  }

  uint i;
  FOR1D(drefs, i) {
    if (drefs_inNegatedLiteralsOnly(i))
      continue;
    ground_drefs.append(subs.elem(0)->getSubs(drefs(i)));
  }
}




 /****************************************
    SPECIAL RULES
 ***************************************/


TL::Rule* TL::RuleEngine::generateDefaultRule(double noiseProb, double minProb, double change) {
  TL::Rule* newDefaultRule = new TL::Rule;
  uintA args_empty;
  newDefaultRule->action = re_le->getPI(re_le->getPredicate(TL_DEFAULT_ACTION_PRED__ID), true, args_empty);
  PredIA sameOutcome;
  newDefaultRule->outcomes.append(sameOutcome);
  PredIA noiseOutcome;
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



bool TL::RuleEngine::isDefaultRule(TL::Rule* rule) {
  return rule->context.N == 0  &&  rule->action->pred->id == TL_DEFAULT_ACTION_PRED__ID;
}



TL::Rule* TL::RuleEngine::getDoNothingRule() {
  TL::Rule* r = new TL::Rule;
  r->action = re_le->getPI_doNothing();
  r->outcomes.resize(2);
  r->probs.resize(2);
  r->probs(0) = 1.0;
  r->probs(1) = 0.0;
  r->noise_changes = 0.0;
  return r;
}






