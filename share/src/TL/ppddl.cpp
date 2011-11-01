#include "TL/ppddl.h"
#include "TL/logicDefinitions.h"
#include "TL/logicReasoning.h"

const uint CONSTANTS_START = 11;

TL::Literal* buildLiteral(MT::String& text, const MT::Array<MT::String> string_objects) {
  uint DEBUG = 0;
  MT::skip(text, "(");
  bool is_true = true;
  if (MT::peerNextChar(text) == '-') {
    text >> "-";
    is_true = false;
  }
  MT::String predicate_name;
  predicate_name.read(text, NULL, " ");
  if (DEBUG>0) {PRINT(predicate_name);}
  
  MT::Array< MT::String > string_arguments;
  while (MT::skip(text) != -1) {
    MT::String obj;
    obj.read(text, NULL, " ");
    if (DEBUG>0) {PRINT(obj);}
    string_arguments.append(obj);
  }
  uintA args;
  args.resize(0);
  uint i;
  FOR1D(string_arguments, i) {
    args.append(string_objects.findValue(string_arguments(i)) + CONSTANTS_START);
  }
  
  return TL::logicObjectManager::getLiteral(TL::logicObjectManager::getPredicate(predicate_name), is_true, args);
}


void TL::readPPDDLdomain(TL::State& start_state, TL::Reward& reward, const char * filename) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"readPPDDLdomain [START]"<<endl;}
  
  uint i;
  
  if (DEBUG>0) {PRINT(filename);}
  ifstream in(filename);
  CHECK(in.is_open(), "Input stream ain't open!");
  
  // (1) OBJECTS
  // Assumption thus far: only one type
  MT::String line_objects;
  do {
    MT::skip(in);
    in >> line_objects;
    if (DEBUG>1) {PRINT(line_objects);}
  } while (line_objects(2) != 'o');
  
  MT::Array< MT::String > string_objects;
  MT::skipUntil(line_objects, " "); // skipping "(:objects"
  while (MT::peerNextChar(line_objects) != '-') {
    MT::String obj;
    obj.read(line_objects, NULL, " ");
    string_objects.append(obj);
    if (DEBUG>1) {PRINT(obj);}
  }
  
  uintA constants;
  FOR1D(string_objects, i) {
    constants.append(i+CONSTANTS_START);
  }
  if (DEBUG>0) {
    cout<<"CONSTANTS:"<<endl;
    FOR1D(string_objects, i) {
      cout<<string_objects(i)<<"  -->  "<< constants(i)<<endl;
    }
  }
  TL::logicObjectManager::setConstants(constants);
  if (DEBUG>0) {PRINT(TL::logicObjectManager::constants);}
  
  
  // (2) INITIAL STATE
  MT::skip(in);
  MT::String line_state;
  in >> line_state;
  if (DEBUG>1) {PRINT(line_state);}
  CHECK(line_state(2) == 'i', "");
  MT::skipUntil(line_state, " ");
  while (MT::peerNextChar(line_state) != ')') {
//     MT::skip(in);
    MT::String string_predicate;
    string_predicate.read(line_state, NULL, ")");
    if (DEBUG>1) {PRINT(string_predicate);}
    start_state.lits_prim.append(buildLiteral(string_predicate, string_objects));
    if (DEBUG>2) {cout<<" ->  "<<*start_state.lits_prim.last()<<endl;}
    MT::skip(line_state);
    if (MT::peerNextChar(line_state) != '('  &&  MT::peerNextChar(line_state) != ')') {
      in >> line_state;
      if (DEBUG>1) {PRINT(line_state);}
      MT::skip(line_state);
    }
//     HALT("");
  }
  TL::logicReasoning::sort(start_state.lits_prim);
  if (DEBUG>0) {
    cout<<"START STATE:  "<<start_state<<endl;
  }
  
  
  MT::skip(in);
  
  // (3) GOAL
  if (reward.reward_type == TL::Reward::reward_literalList) {
    TL::LiteralListReward* cast_reward = (TL::LiteralListReward*) &reward;
    MT::String line_reward;
    in >> line_reward;
    if (DEBUG>1) {PRINT(line_reward);}
    CHECK(line_reward(2) == 'g', "");
    MT::skipUntil(line_reward, " "); // skipping "(:reward"
    cast_reward->lits.clear();
    while (MT::peerNextChar(line_reward) != ')') {
      MT::skip(line_reward);
      MT::String string_predicate;
      string_predicate.read(line_reward, NULL, ")");
      if (DEBUG>1) {PRINT(string_predicate);}
      // skipping (and if necessary
      if (string_predicate(1) == 'a'  &&  string_predicate(2) == 'n'  &&  string_predicate(3) == 'd') {
        MT::skipUntil(string_predicate, " ");
      }
      cast_reward->lits.append(buildLiteral(string_predicate, string_objects));
  //     HALT("");
    }
  }
  else if (reward.reward_type == TL::Reward::reward_maximize_function) {
    MT::String line_reward;
    in >> line_reward;
    if (DEBUG>1) {PRINT(line_reward);}
    CHECK(line_reward(2) == 'g', "");
    MT::skipUntil(line_reward, " "); // skipping "(:reward"
    // CountFunction
    if (MT::peerNextChar(line_reward) == 'C') {
      line_reward >> "C";
      MT::skip(line_reward);
      MT::String string_function;
      string_function.read(line_reward, NULL, "(");
//       PRINT(string_function);
      uint q;
      TL::CountFunction* f = NULL;
      FOR1D(TL::logicObjectManager::f_derived, q) {
//         PRINT(TL::logicObjectManager::f_derived(q)->name);
        if (TL::logicObjectManager::f_derived(q)->name == string_function) {
          f = (TL::CountFunction*) TL::logicObjectManager::f_derived(q);
          break;
        }
      }
      MT::skip(line_reward, ") ");
      line_reward >> f->max_value;
      CHECK(f!=NULL, "");
      uintA empty;
      empty.resize(0);
      TL::MaximizeFunctionReward* cast_reward = (TL::MaximizeFunctionReward*) &reward;
      cast_reward->fa = TL::logicObjectManager::getFA(f, empty);
      
      // reward-reward
      MT::skipUntil(in, "(");
      MT::String line_rewardReward;
      in >> line_rewardReward;
      if (DEBUG>1) {PRINT(line_rewardReward);}
      CHECK(line_rewardReward(1) == 'g' && line_rewardReward(6) == 'r', "Strange line: "<<line_rewardReward);
      MT::skipUntil(line_rewardReward, " "); // skipping "(:reward-reward"
      double rewardReward;
      line_rewardReward >> rewardReward;
      f->reward_for_max_value = rewardReward;
    }
    else {
      LitL reward_lits;
      while (MT::peerNextChar(line_reward) != ')') {
        MT::skip(line_reward);
        MT::String string_predicate;
        string_predicate.read(line_reward, NULL, ")");
        if (DEBUG>1) {PRINT(string_predicate);}
        // skipping (and if necessary
        if (string_predicate(1) == 'a'  &&  string_predicate(2) == 'n'  &&  string_predicate(3) == 'd') {
          MT::skipUntil(string_predicate, " ");
        }
        reward_lits.append(buildLiteral(string_predicate, string_objects));
    //     HALT("");
      }
      // reward-reward
      MT::skipUntil(in, "(");
      MT::String line_rewardReward;
      in >> line_rewardReward;
      if (DEBUG>1) {PRINT(line_rewardReward);}
      CHECK(line_rewardReward(1) == 'g' && line_rewardReward(6) == 'r', "Strange line: "<<line_rewardReward);
      MT::skipUntil(line_rewardReward, " "); // skipping "(:reward-reward"
      double rewardReward;
      line_rewardReward >> rewardReward;
      if (DEBUG>1) {PRINT(rewardReward);}
      TL::RewardFunction* f = new TL::RewardFunction;
      f->reward_value = rewardReward;
      f->grounded_pis = reward_lits;
      f->id = 1;
      f->name = "reward_reward";
      f->d = 0;
      if (DEBUG>1) {f->writeNice(); cout<<endl;}
      FuncL wrapper;
      wrapper.append(f);
      TL::logicObjectManager::addStateFunctions(wrapper);
      uintA empty;
      empty.resize(0);
      TL::MaximizeFunctionReward* cast_reward = (TL::MaximizeFunctionReward*) &reward;
      cast_reward->fa = TL::logicObjectManager::getFA(f, empty);
    }
  }
  else
    HALT("");
  
  if (DEBUG>0) {
    cout<<"GOAL:  "; reward.writeNice();  cout<<endl;
  }
  
    // (4) DERIVE INITIAL STATE
  TL::logicReasoning::derive(&start_state);
  if (DEBUG>0) {
    cout<<"START STATE:  "<<start_state<<endl;
  }
  
  
  if (DEBUG>0) {cout<<"readPPDDLdomain [END]"<<endl;}
}











// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

MT::Array< char > vars;


void lit2ppddl(TL::Literal& lit, ostream& out) {
  uint i;
  out << "(";
  if (lit.atom->pred->type == TL::Predicate::predicate_comparison) {
    TL::ComparisonAtom* ca = (TL::ComparisonAtom*) lit.atom;
    CHECK(ca->hasConstantBound(), "non-constant-bound");
    switch(ca->comparisonType) {
      case TL::comparison_less: out << "<"; break;
      case TL::comparison_lessEqual: out << "<="; break;
      case TL::comparison_greater: out << ">"; break;
      case TL::comparison_greaterEqual: out << ">="; break;
      case TL::comparison_equal: out << "="; break;
    }
    out << " ";
    if (ca->args.N == 1) {
      if (ca->fa1->args(0) < vars.N)
        out << "(" << ca->fa1->f->name << " ?" << vars(ca->args(0)) << ")";
      else
        out << "(" << ca->fa1->f->name << " o" << ca->args(0) << ")";
    }
    else
      HALT("");
    out << " " << ca->bound;
  }
  else {
    if (!lit.positive)
      out << "not (";
    out << lit.atom->pred->name;
    FOR1D(lit.atom->args, i) {
      if (lit.atom->args(i) < vars.N)
        out << " ?" << vars(lit.atom->args(i));
      else
        out << " o" << lit.atom->args(i);
    }
    if (!lit.positive)
      out << ")";
  }
  out << ")";
}


void fv2ppddl(TL::FunctionValue& fv, ostream& out) {
  out << "(";
  out << "= ";
  if (fv.atom->args.N == 1) {
    if (fv.atom->args(0) < vars.N)
      out << "(" << fv.atom->f->name << " ?" << vars(fv.atom->args(0)) << ")";
    else
      out << "(" << fv.atom->f->name << " o" << fv.atom->args(0) << ")";
  }
  else
    HALT("");
  out << " " << fv.value;
  out << ")";
}




void TL::writeRulesAsPPDDL(const TL::RuleSet& rules, bool all_outcome, ostream& out) {
  vars.clear();
  vars.append('x');
  vars.append('y');
  vars.append('z');
  vars.append('v');
  vars.append('w');
  
  out.precision(3);
  
  uint i, k, l;
  FOR1D_(rules, i) {
    TL::Rule* rule = rules.elem(i);
    // letzteres HACK fuer noAction
    if (TL::ruleReasoning::isDefaultRule(rule)  ||  rule->action->pred->id == 3)
      continue;
    uintA terms, drefs;
    ruleReasoning::calcTerms(*rule, terms);
    ruleReasoning::calcDeicticRefs(*rule, drefs);

    out << "(:action op"<<i<<rule->action->pred->name<<endl;
    
    out << "\t:parameters (";
    FOR1D(terms, k) {
      out << " ?" << vars(terms(k));
    }
    out << ")"<<endl;
    
    out << "\t:precondition (and ";
    FOR1D(rule->context, k) {
      lit2ppddl(*rule->context(k), out);
      out<<"  ";
    }
    // explicitely set all args to be different from drefs
    CHECK(rule->action->pred->d == 1, "");
    FOR1D(drefs, k) {
      out << "(not (= ?" << vars(0) << " ?" << vars(k+1) << "))  ";
    }
    
    // explicitely set all objects to be not out
    TL::Predicate* p_OUT = TL::logicObjectManager::getPredicate(MT::String("out"));
    FOR1D(terms, k) {
      uintA args(1);  args(0) = terms(k);
      Literal* lit = TL::logicObjectManager::getLiteral(p_OUT, false, args);
      lit2ppddl(*lit, out);
      out<<"  ";
    }
    out << " )" << endl;
    
    out << "\t:effect (probabilistic"<<endl;
    if (all_outcome) {
      FOR1D(rule->outcomes, k) {
        if (k == rule->outcomes.N-1)
          break;
        if (rule->outcomes(k).N == 0)
          continue;
        out << "\t\t\t" << (rule->probs(k)-0.001);
        out << "\t(and ";
        FOR1D(rule->outcomes(k), l) {
          lit2ppddl(*rule->outcomes(k)(l), out);
          out<<" ";
        }
        out<<")"<<endl;
      }
    }
    else {
      MT_MSG("Single outcome determinization");
      arr probs = rule->probs;
      probs.remove(probs.N-1);
      uint max_id = probs.maxIndex();
      if (rule->outcomes(max_id).N == 0)
        continue;
      out << "\t\t\t" << (rule->probs(max_id)-0.001);
      out << "\t(and ";
      FOR1D(rule->outcomes(max_id), l) {
        lit2ppddl(*rule->outcomes(max_id)(l), out);
        out<<" ";
      }
      out<<")"<<endl;
    }
    out << "\t\t)" << endl;
    
    
    out << ")" << endl << endl;
  }
}


void TL::writePPDDL_description(const TL::RuleSet& rules, 
                                bool all_outcome, 
                                const TL::State& state, 
                                const TL::LiteralListReward& reward,
                                const char* filename) {
  bool use_functions = false;
  bool use_complex_rule_conversion = false;
  
  ofstream out(filename);
  // (1) Domain
  out<<"(define (domain desktop-world)"<<endl;
  if (use_complex_rule_conversion) {
    out<<"\t(:requirements :probabilistic-effects :equality :fluents :negative-preconditions  :rewards  :conditional-effects :existential-preconditions)"<<endl;
  }
  else {
    out<<"\t(:requirements :probabilistic-effects :equality :fluents :negative-preconditions  :rewards)"<<endl;
  }
  out<<"\t(:predicates  (block ?x)  (ball ?x)  (table ?x)  (on ?x ?y)  (inhand ?x)  (upright ?x)  (out ?x)";
  if (TL::logicObjectManager::getPredicate(MT::String("clear")) != NULL)
    out << "  (clear ?x)";
  if (TL::logicObjectManager::getPredicate(MT::String("homies")) != NULL)
    out << "  (homies ?x ?y)";
  if (TL::logicObjectManager::getPredicate(MT::String("inhandNil")) != NULL)
    out << "  (inhandNil)";
  out << "  )" << endl;
  if (use_functions) {out<<"\t(:functions  (size ?x) )"<<endl;}
  out<<endl;
  // Rules
  if (use_complex_rule_conversion) {
    NIY;
//     writeRulesAsPPDDL2(rules, le, out);
  }
  else
    writeRulesAsPPDDL(rules, all_outcome, out);
  out<<endl;
  out<<")"<<endl;
  out<<endl;
  out<<endl;
  
  // (2) Problem
  out<<"(define (problem dw)"<<endl;
  out<<"(:domain desktop-world)"<<endl;
  out<<"(:objects";
  uint i;
  FOR1D(TL::logicObjectManager::constants, i) {
    out<<" o"<<TL::logicObjectManager::constants(i);
  }
  out<<")"<<endl;
  // Start state
  out<<"(:init ";
  FOR1D(state.lits_prim, i) {
    lit2ppddl(*state.lits_prim(i), out);  out<<" ";
  }
  out<<endl;
  FOR1D(state.lits_derived, i) {
    if (state.lits_derived(i)->atom->pred->id == 41  ||  state.lits_derived(i)->atom->pred->id == 42)
      lit2ppddl(*state.lits_derived(i), out);  out<<" ";
  }
  out<<endl;
  if (use_functions) {
    FOR1D(state.fv_prim, i) {
      fv2ppddl(*state.fv_prim(i), out);  out<<" ";
    }
    out<<endl;
  }
  out<<")"<<endl;
  // Goal
  out<<"(:reward (and ";
  FOR1D(reward.lits, i) {
    lit2ppddl(*reward.lits(i), out);  out<<" ";
  }
  out<<") )"<<endl;
  
  out<<"(:reward-reward 1)"<<endl;
//   out<<"(:metric maximize (reward))"<<endl;

  out<<")"<<endl;
}

