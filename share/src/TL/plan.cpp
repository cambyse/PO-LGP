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


#include "plan.h"
#include "logicReasoning.h"




// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    Rewards
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

TL::Reward::Reward() {
}

TL::Reward::Reward(RewardType _reward_type) {
  reward_type = _reward_type;
}


// LiteralReward

TL::LiteralReward::LiteralReward(TL::Literal* _pt) : Reward(reward_literal) {
  CHECK(_pt->positive, "");
  lit = _pt;
}

double TL::LiteralReward::evaluate(const State& s) const {
  if (TL::logicReasoning::holds(s, lit))
    return 1.0;
  else
    return 0.0;
}

bool TL::LiteralReward::satisfied(const State& s) const {
  double result = evaluate(s);
  return TL::areEqual(1., result);
}

bool TL::LiteralReward::possible(const State& s) const {
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  uint i;
  FOR1D(lit->atom->args, i) {
    uintA args(1);  args(0)=lit->atom->args(i);
    TL::Literal* pi_out = logicObjectManager::getLiteral(p_OUT, true, args);
    if (logicReasoning::holds(s, pi_out))
      return false;
  }
  return true;
}

void TL::LiteralReward::writeNice(ostream& out) const {
  lit->write(out);
}

void TL::LiteralReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# LiteralReward"<<endl;
  out<<"# "; lit->write(out); out<<endl;
  lit->write(out); out<<endl;
  out.close();
}


void TL::LiteralReward::getRewardObjects(uintA& objects, const TL::State* s) const {
  objects.clear();
  objects.setAppend(lit->atom->args);
}



// LiteralListReward

TL::LiteralListReward::LiteralListReward(LitL& _lits) : Reward(reward_literalList) {
  lits = _lits;
}

double TL::LiteralListReward::evaluate(const State& s) const {
  if (TL::logicReasoning::holds(s, lits))
    return 1.0;
  else
    return 0.0;
}

bool TL::LiteralListReward::satisfied(const State& s) const {
  double result = evaluate(s);
  if (TL::areEqual(1., result))
    return true;
  else
    return false;
}

bool TL::LiteralListReward::possible(const State& s) const {
  // BRING IN DOMAIN KNOWLEDGE
  
  // Desktop world domain
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  uint i, k;
  if (p_OUT != NULL) {
    FOR1D(lits, k) {
      FOR1D(lits(k)->atom->args, i) {
        uintA args(1);  args(0)=lits(k)->atom->args(i);
        TL::Literal* pt_out = logicObjectManager::getLiteral(p_OUT, true, args);
        if (logicReasoning::holds(s, pt_out))
          return false;
      }
    }
  }
  
  // Ex-Blocksworld domain
  TL::Predicate* p_NO_DESTROYED_TABLE = logicObjectManager::getPredicate(MT::String("no-destroyed-table"));
  TL::Predicate* p_ON_TABLE = logicObjectManager::getPredicate(MT::String("on-table"));
  TL::Predicate* p_NO_DESTROYED = logicObjectManager::getPredicate(MT::String("no-destroyed"));
  if (p_NO_DESTROYED_TABLE != NULL  &&  p_ON_TABLE != NULL) {
    uintA empty;
    TL::Literal* pi_no_destroyed_table = logicObjectManager::getLiteral(p_NO_DESTROYED_TABLE, true, empty);
    FOR1D(lits, k) {
      // (1)  Impossible if one object X still has to be put "on-table(X)" (but is not yet!)
      // and it already does NOT hold "no-destroyed-table()".
      if (lits(k)->atom->pred == p_ON_TABLE  &&  lits(k)->positive) {
//         PRINT(*lits(k));
//         PRINT(!logicReasoning::holds(s, lits(k)));
//         PRINT(!logicReasoning::holds(s, pi_no_destroyed_table));
        if (!logicReasoning::holds(s, lits(k)) &&  !logicReasoning::holds(s, pi_no_destroyed_table)) {
          cout<<"Impossible as "<<*pi_no_destroyed_table<<" does not hold and we still require "<<*lits(k)<<endl;
          cerr<<"Impossible as "<<*pi_no_destroyed_table<<" does not hold and we still require "<<*lits(k)<<endl;
          return false;
        }
      }
      // (2)  Impossible if object to be moved is destroyed.
      if (lits(k)->atom->args.N == 2) {
        if (!logicReasoning::holds(s, lits(k))) {
          uintA helper;  helper.append(lits(k)->atom->args(0));
          TL::Literal* pi_helper = logicObjectManager::getLiteral(p_NO_DESTROYED, true, helper);
          if (!logicReasoning::holds(s, pi_helper)) {
            cout<<"Impossible as "<<lits(k)->atom->args(0)<<" is already destroyed; i.e., it does not hold that "<<*pi_helper<<endl;
            cerr<<"Impossible as "<<lits(k)->atom->args(0)<<" is already destroyed; i.e., it does not hold that "<<*pi_helper<<endl;
            return false;
          }
        }
      }
    }
  }
  
  // Triangle-tireworld domain
  TL::Predicate* p_VEHICLE_AT = logicObjectManager::getPredicate(MT::String("vehicle-at"));
  TL::Predicate* p_SPARE_IN = logicObjectManager::getPredicate(MT::String("spare-in"));
  TL::Predicate* p_NOT_FLATTIRE = logicObjectManager::getPredicate(MT::String("not-flattire"));
  TL::Predicate* p_HASSPARE = logicObjectManager::getPredicate(MT::String("hasspare"));
  if (p_VEHICLE_AT != NULL  &&  p_NOT_FLATTIRE != NULL) {
    uintA empty;
    TL::Literal* pi_not_flattire = logicObjectManager::getLiteral(p_NOT_FLATTIRE, true, empty);
    TL::Literal* pi_hasspare = logicObjectManager::getLiteral(p_HASSPARE, true, empty);
    FOR1D(lits, k) {
      // (1)  Impossible if   (i) not in goal-location,  (ii) not not-flattire,  (iii) not has-spare   and (iv) not spare-in current location
      if (lits(k)->atom->pred == p_VEHICLE_AT  &&  lits(k)->positive) {
        // (i)
        if (!logicReasoning::holds(s, lits(k))) {
          // (ii), (iii)
          if (!logicReasoning::holds(s, pi_not_flattire)  &&  !logicReasoning::holds(s, pi_hasspare) ) {
            // (iv)
            uint current_location = logicReasoning::getArgument(s, *p_VEHICLE_AT);
            uintA wrapper;  wrapper.append(current_location);
            Literal* pi_spare_in_current_location = logicObjectManager::getLiteral(p_SPARE_IN, true, wrapper);
            if (!logicReasoning::holds(s, pi_spare_in_current_location)) {
              cout<<"Impossible as -" << *lits(k) << " (REWARD), but -"<<*pi_not_flattire<<",  current_location="
                        <<current_location<<", -"<<*pi_hasspare<<" and -"
                        <<*pi_spare_in_current_location<< " (CURRENT STATE)."<<endl;
              return false;
            }
          }
        }
      }
    }
  }
  
  return true;
}

void TL::LiteralListReward::writeNice(ostream& out) const {
  uint i;
  out<<"[" << lits.N  << "]  ";
  FOR1D(lits, i) {
    lits(i)->write(out);out<<" ";
  }
}

void TL::LiteralListReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# LiteralListReward"<<endl;
  uint i;
  FOR1D(lits, i) {
    out<<"# "; lits(i)->write(out); out<<endl;
  }
  out<<lits.N<<endl;
  FOR1D(lits, i) {
    lits(i)->write(out); out<<endl;
  }
  out.close();
}

void TL::LiteralListReward::getRewardObjects(uintA& objects, const TL::State* s) const {
  objects.clear();
  uint i;
  FOR1D(lits, i) {
    objects.setAppend(lits(i)->atom->args);
  }
}


// DisjunctionReward


TL::DisjunctionReward::DisjunctionReward(LitL& _lits) : Reward(reward_one_of_literal_list) {
  this->lits = _lits;
  this->weights.resize(this->lits.N);
  this->weights.setUni(1.0);
}

TL::DisjunctionReward::DisjunctionReward(LitL& _lits, arr& _weights) : Reward(reward_one_of_literal_list) {
  this->lits = _lits;
  this->weights = _weights;
}

double TL::DisjunctionReward::evaluate(const State& s) const {
  uint i;
  double max = 0.0;
  FOR1D(lits, i) {
    if (TL::logicReasoning::holds(s, lits(i))) {
      max = TL_MAX(max, weights(i));
    }
  }
  return max;
}

bool TL::DisjunctionReward::satisfied(const State& s) const {
  uint i;
  FOR1D(lits, i) {
    if (TL::logicReasoning::holds(s, lits(i)))
      return true;
  }
  return false;
}

bool TL::DisjunctionReward::possible(const State& s) const {
  TL::Predicate* p_OUT = logicObjectManager::getPredicate(MT::String("out"));
  uint i, k;
  if (p_OUT != NULL) {
    FOR1D(lits, k) {
      FOR1D(lits(k)->atom->args, i) {
        uintA args(1);  args(0)=lits(k)->atom->args(i);
        TL::Literal* pt_out = logicObjectManager::getLiteral(p_OUT, true, args);
        if (logicReasoning::holds(s, pt_out))
          return false;
      }
    }
  }
  return true;
}

void TL::DisjunctionReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# DisjunctionReward"<<endl;
  uint i;
  FOR1D(lits, i) {
    out<<"# "; lits(i)->write(out); out<<endl;
  }
  out<<lits.N<<endl;
  FOR1D(lits, i) {
    lits(i)->write(out); out<<endl;
  }
  out.close();
}

void TL::DisjunctionReward::writeNice(ostream& out) const {
  uint i;
  out<<"[" << lits.N  << "]  OR  ";
  FOR1D(lits, i) {
    out<<weights(i)<<":";lits(i)->write(out); out << "  ";
  }
}

void TL::DisjunctionReward::getRewardObjects(uintA& objects, const TL::State* s) const {
  objects.clear();
  uint i;
  FOR1D(lits, i) {
    objects.setAppend(lits(i)->atom->args);
  }
}


// MaximizeFunctionReward

TL::MaximizeFunctionReward::MaximizeFunctionReward() : Reward(reward_maximize_function) {
  fa = NULL;
}

TL::MaximizeFunctionReward::MaximizeFunctionReward(TL::FunctionAtom* _fa) : Reward(reward_maximize_function) {
  fa = _fa;
}

double TL::MaximizeFunctionReward::evaluate(const State& s) const {
  uint i;
  if (fa->f->category == category_primitive) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->atom->f == fa->f && s.fv_prim(i)->atom->args == fa->args)
        return s.fv_prim(i)->value;
    }
    CHECK(i==s.fv_prim.N, "");
  }
  else {
    CHECK(s.derivedDerived, "");
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->atom->f == fa->f && s.fv_derived(i)->atom->args == fa->args)
        return s.fv_derived(i)->value;
    }
    CHECK(i==s.fv_derived.N, "");
  }
  HALT("failed");
  return -10000.;
}


bool TL::MaximizeFunctionReward::satisfied(const State& s) const {
  // Satisfied = maximum value
  // For count function with 1-arity has maximium value if all objects (except table) fullfill it
  if (fa->f->type == TL::Function::function_count) {
    CountFunction* cf = (CountFunction*) fa->f;
    if (cf->max_value == -1) {
      if (cf->countedPred->d == 1) {
        double value = logicReasoning::getValue(fa->f, s);
        uintA constants;
        logicReasoning::getConstants(s, constants);
        if (TL::areEqual(value, constants.N-1))
          return true;
        else
          return false;
      }
    }
    else {
      int value = (int) logicReasoning::getValue(fa->f, s);
//       PRINT(value);
//       PRINT(cf->max_value);
      if (value == cf->max_value)
        return true;
      else
        return false;
    }
  }
  else if (fa->f->type == TL::Function::function_reward) {
    RewardFunction* grf = dynamic_cast<RewardFunction*>(fa->f);
    return logicReasoning::holds(s, grf->grounded_pis);
  }
  return false; // is never satisfied...
}


bool TL::MaximizeFunctionReward::possible(const State& s) const {
  return true; // is always true
}


void TL::MaximizeFunctionReward::writeNice(ostream& out) const {
  if (fa == NULL) {
    out<<"TL::MaximizeFunctionReward::writeNice:  fa=NULL"<<endl;
  }
  else if (fa->f->type == TL::Function::function_count) {
    out<<"maximize #"<<((CountFunction*) (fa->f))->countedPred->name;
  }
  else {
    fa->write(out);
    out<<"   ";
    fa->f->writeNice(out);
    out<<"   ";
    out<<"   ";
    if (important_literals.N > 0) {cout<<"(important literals: "<<important_literals<<")  ";}
    out<<"MAXIMIZE_FUNCTION";
  }
}

void TL::MaximizeFunctionReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# MaximizeFunctionReward"<<endl;
  fa->write(out); out<<endl;
  out<<reward_maximize_function<<endl;
  fa->write(out); out<<endl;
  out.close();
}

void TL::MaximizeFunctionReward::getRewardObjects(uintA& objects, const TL::State* s) const {
  objects.clear();
  if (fa->f->type == TL::Function::function_count) {
    // Reward objects are those for which predicate instances don't hold yet
    LitL lits;
    logicObjectManager::getLiterals(lits, ((CountFunction*) (fa->f))->countedPred, logicObjectManager::constants);
    lits.memMove = true;
    uint i;
    FOR1D_DOWN(lits, i) {
      if (logicReasoning::holds(*s, lits(i)))
        lits.remove(i);
    }
    FOR1D(lits, i) {
      objects.setAppend(lits(i)->atom->args);
    }
  }
  else if (fa->f->type == TL::Function::function_reward) {
    RewardFunction* rf = (RewardFunction*) fa->f;
    logicReasoning::getConstants(rf->grounded_pis, objects);
  }
  else if (fa->f->type == TL::Function::function_sum) {
    // Reward objects are all objects in state.
    logicReasoning::getConstants(*s, objects);
  }
  
  else 
    HALT("Warning: get reward objects has not been implemented in a korrekt way yet");
}




// NotTheseStatesReward


TL::NotTheseStatesReward::NotTheseStatesReward(const StateL& _undesired_states) : Reward(reward_not_these_states) {
  undesired_states = _undesired_states;
}

double TL::NotTheseStatesReward::evaluate(const State& s) const {
  uint i;
  FOR1D(undesired_states, i) {
    if (s == *undesired_states(i))
      return 0.0;
  }
  return 1.0;
}


bool TL::NotTheseStatesReward::satisfied(const State& s) const {
  uint i;
  FOR1D(undesired_states, i) {
    if (s == *undesired_states(i))
      return false;
  }
  return true;
}


bool TL::NotTheseStatesReward::possible(const State& s) const {
  return true; // is always true
}


void TL::NotTheseStatesReward::writeNice(ostream& out) const {
  out<<"reward_not_these_states  "<<endl;
  uint i;
  out<<undesired_states.N<<" undesired states:"<<endl;
  FOR1D(undesired_states, i) {
    out<<i<<": ";
    undesired_states(i)->write(out, true);
    out<<endl;
  }
}

void TL::NotTheseStatesReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# NotTheseStatesReward"<<endl;
  out<<reward_not_these_states<<endl;
  out<<"# Number of undesired states = "<<undesired_states.N << endl;
  uint i;
  FOR1D(undesired_states, i) {
    undesired_states(i)->write(out);
    out<<endl;
  }
  out.close();
}

void TL::NotTheseStatesReward::getRewardObjects(uintA& objects, const TL::State* s) const {
  objects.clear();
  NIY;
}







// Reward helpers

TL::Reward* TL::readReward(const char* filename) {
  ifstream in(filename);
  if (!in.is_open()) HALT("File cannot be opened.");
  MT::skip(in);
  uint type;
  in >> type;
  MT::skip(in);
  if (type == TL::Reward::reward_literal) {
    MT::String line;
    line.read(in, NULL, "\n");
    Literal* lit = logicObjectManager::getLiteral(line);
    uint i;
    FOR1D(lit->atom->args, i) {
      if (logicObjectManager::constants.findValue(lit->atom->args(i)) < 0)
        HALT("Reward uses unknown argument "<<lit->atom->args(i));
    }
    lit = logicObjectManager::getLiteralOrig(lit);
    return new LiteralReward(lit);
  }
  else if (type == TL::Reward::reward_literalList) {
    LitL lits;
    while (MT::skip(in) != -1) {
      MT::String line;
      line.read(in, NULL, "\n");
      Literal* lit = logicObjectManager::getLiteral(line);
      uint i;
      FOR1D(lit->atom->args, i) {
        if (logicObjectManager::constants.findValue(lit->atom->args(i)) < 0)
          HALT("Reward uses unknown argument "<<lit->atom->args(i));
      }
      lit = logicObjectManager::getLiteralOrig(lit);
      lits.append(lit);
    }
    return new LiteralListReward(lits);
  }
  else if (type == TL::Reward::reward_maximize_function) {
    MT::String line;
    line.read(in, NULL, "\n");
    FunctionAtom* fa = logicObjectManager::getFA(line);
    uint i;
    FOR1D(fa->args, i) {
      if (logicObjectManager::constants.findValue(fa->args(i)) < 0)
        HALT("Reward uses unknown argument "<<fa->args(i));
    }
    fa = logicObjectManager::getFAorig(fa);
    return new MaximizeFunctionReward(fa);
  }
  else
    HALT("Unknown reward type " << type << " in file "<< filename);
  return NULL;
}






// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    SST
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


AtomL DEBUG_actions;

TL::Atom* TL::SST::generateAction(double& value, const TL::State& s0, const Reward& reward, uint branch, uint tau, double discount, const WorldAbstraction& wa) {
  uint DEBUG = 0;
  if (DEBUG_actions.N == 0)
    DEBUG_actions.resize(tau);
  if (DEBUG>0) {
    cout<<"+ SST - start  tau="<<tau<<endl;
    s0.write(); cout<<endl;
  }
  if (tau == 5)
    cout<<"."<<std::flush;
  double reward_s0 = reward.evaluate(s0);
  double reward_tree = 0.;
  uint i, b;
  TL::Atom* action = NULL;
  bool action_is_applicable;
  if (tau>0) {
    arr action_values(wa.ground_actions.N);
    FOR1D(wa.ground_actions, i) {
      double action_value = 0.;
      action_is_applicable = true;
      if (DEBUG>0) {DEBUG_actions(DEBUG_actions.N-tau) = wa.ground_actions(i);}
      for(b=0; b<branch; b++) {
        uint flag;
        TL::State s_suc;
        double ruleOutcome_reward = wa.sampleSuccessorState(s_suc, flag, s0, wa.ground_actions(i));
        action_is_applicable = (ruleOutcome_reward != TL_DOUBLE_NIL);
        if (!action_is_applicable) {
          if (DEBUG>1) {
            cout<<"++ Omitting at "<<tau<<" "; wa.ground_actions(i)->write(); cout<<" b="<<b<<endl;
          }
          action_value = -10000.;
          break;
        }
        else {
          double tree_value;
          if (DEBUG>1) {
            cout<<"++ going down tau="<<tau<<" "; wa.ground_actions(i)->write(); cout<<" b="<<b<<endl;
          }
          generateAction(tree_value, s_suc, reward, branch, tau-1, discount, wa);
          tree_value = wa.postprocessValue(tree_value, flag);
          tree_value += ruleOutcome_reward;
          action_value += tree_value;
        }
      }
      action_value /= branch;
      action_values(i) = action_value;
    }
    uint max_id = action_values.maxIndex();
    action = wa.ground_actions(max_id);
    reward_tree = action_values(max_id);
    if (DEBUG>1) {
      cout<<"Best Action for "; uint t; for (t=0; t<DEBUG_actions.N-tau; t++){DEBUG_actions(t)->write(); cout<<" ";}
      cout<<"  -->  ";action->write(); cout<<endl;
    }
  }
  value = reward_s0 + discount * reward_tree;
  if (DEBUG>0) {
    PRINT(reward_s0);
    PRINT(reward_tree);
    PRINT(value);
    cout<<"+ SST - end  tau="<<tau<<endl;
  }
  return action;
}









// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    NID Planner
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


TL::NID_Planner::NID_Planner(double noise_scaling_factor) {
  this->noise_scaling_factor = noise_scaling_factor;
  this->horizon = 1;
  this->discount = 0.95;
  this->use_ruleOutcome_rewards = false;
}

TL::NID_Planner::~NID_Planner() {
}

void TL::NID_Planner::setDiscount(double discount) {
  this->discount = discount;
  
  discount_pow.resize(horizon+1);
  uint i=0;
  for (i=0;i<=horizon; i++) {
    discount_pow(i) = pow(discount, i);
  }
}

void TL::NID_Planner::setHorizon(uint horizon) {
  this->horizon = horizon;
  if (discount_pow.N < horizon+1) {
    discount_pow.resize(horizon+1);
    uint i=0;
    for (i=0;i<=horizon; i++) {
      discount_pow(i) = pow(discount, i);
    }
  }
}

void TL::NID_Planner::setReward(Reward* reward) {
  this->reward = reward;
}

void TL::NID_Planner::setGroundRules(TL::RuleSet& ground_rules) {
  this->ground_rules = ground_rules;
  // fill actions list
  ground_actions.clear();
  is_manipulating_rule.clear();
  TL::Atom* last_action = NULL;
  uint i;
  FOR1D_(ground_rules, i) {
    if (last_action != ground_rules.elem(i)->action  &&
        ground_rules.elem(i)->action->pred->id != TL::DEFAULT_ACTION_PRED__ID) {
      last_action = ground_rules.elem(i)->action;
      ground_actions.setAppend(last_action);
    }
    bool manipulates = false;
    uint o;
    for (o=0; o<ground_rules.elem(i)->outcomes.N-1; o++) {
      if (ground_rules.elem(i)->outcomes(o).N > 0) {
        manipulates = true;
        break;
      }
    }
    is_manipulating_rule.append(manipulates);
  }
  CHECK(is_manipulating_rule.N == this->ground_rules.num(), "");
  
  FOR1D_(ground_rules, i) {
    if (ground_rules.elem(i)->outcome_rewards.N > 0) {
      this->use_ruleOutcome_rewards = true;
      break;
    }
  }
  
  if (this->use_ruleOutcome_rewards) {
    expected_rule_rewards.resize(ground_rules.num());
    uint k;
    FOR1D_(ground_rules, i) {
      TL::Rule* rule = ground_rules.elem(i);
      if (rule->outcome_rewards.N > 0) {
        double expected_reward = 0.;
//         rule->writeNice();
        FOR1D(rule->outcomes, k) {
          expected_reward += rule->probs(k) * rule->outcome_rewards(k);
        }
        expected_rule_rewards(i) = expected_reward;
      }
    }
  }
}


double TL::NID_Planner::postprocessValue(double value, uint flag) const {
  double value_processed;
  value_processed = value;
  if (flag == STATE_TRANSITION__NOISE_OUTCOME) {
    value_processed *= noise_scaling_factor;
  }
  return value_processed;
}


double TL::NID_Planner::sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::Atom* action) const {
  return ruleReasoning::calcSuccessorState(s_prev, ground_rules, action, flag, s_suc, true);
}



// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    NID-SST
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



TL::NID_SST::NID_SST(uint branch, double noise_scaling_factor) : NID_Planner(noise_scaling_factor) {
  this->branch = branch;
}


TL::Atom* TL::NID_SST::generateAction(const TL::State& current_state, uint max_runs) {
  double value;
  uint i;
  for (i=0; i<max_runs; i++) {
     TL::Atom* action = SST::generateAction(value, current_state, *reward, branch, horizon, discount, *this);
#define SST_THRESHOLD 0.005
     if (value > SST_THRESHOLD) {
      return action;
    }
  }
  MT_MSG("NID_SST: No good action found.");
  return NULL;
}









// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    NID-UCT
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------


TL::NID_UCT::NID_UCT(double noise_scaling_factor) : NID_Planner(noise_scaling_factor) {
  c = 1.0;
}


TL::NID_UCT::~NID_UCT() {
  killAtomLctionsInfo();
}

void TL::NID_UCT::killAtomLctionsInfo() {
  listDelete(s_a_infos);
}

AtomL DEBUG__UCT_ACTIONS;

void TL::NID_UCT::runEpisode(double& value, const TL::State& s, uint t) {
  uint DEBUG = 0;
  if (t==0)
    DEBUG = 0;
  else
    DEBUG = 0;
  if (DEBUG>0) {
    cout<<"NID_UCT::runEpisode() [START]"<<endl;
    PRINT(t);
    cout<<"State: "; s.write(cout, true); cout<<endl;
  }
  double reward_s = reward->evaluate(s);
//   if (reward_s > 0.) {
//     cout<<"DERBE GESCHICHTE!!!!!"<<endl;
//     writeNice(DEBUG__UCT_ACTIONS); cout<<endl;
//     PRINT(t);
//   }
  if (DEBUG>0) {PRINT(reward_s);}
  if (t==horizon) {
    value = reward_s;
  }
  else if (reward->satisfied(s)) {
    value = reward_s;
//     cout<<"finished:"<<endl;
//     cout<<"State: "; s.writeNice(); cout<<endl;
//     uint t2;
//     cout<<"Actions:  ";
//     for (t2=0; t2<t; t2++) {
//       DEBUG__UCT_ACTIONS(t2)->writeNice(cout); cout<<" ";
//     }
//     cout<<endl;
  }
  else {
    AtomLctionsInfo* s_a_info = getAtomLctionsInfo(s);
    uint i;
    arr UCB(ground_actions.N);
    TL::RuleSet rules;
    uintA untried_action_ids;
    if (DEBUG>0) {cout<<"visits(s)="<<s_a_info->getVisits()<<endl;}
    FOR1D(ground_actions, i) {
      TL::State dummy_state = s;
      TL::Rule* r = ground_rules.elem(ruleReasoning::uniqueCoveringRule_groundedRules_groundedAction(ground_rules, s, ground_actions(i)));
      if (!ruleReasoning::isDefaultRule(r)) {
        rules.append(r);
        if (s_a_info->getVisits(i) == 0) {
          untried_action_ids.append(i);
          UCB(i) = -11.;
        }
        else
          UCB(i) = s_a_info->getQvalue(i)  +  c * sqrt(log(s_a_info->getVisits()) / (1.0 * s_a_info->getVisits(i)));
      }
      else {
        rules.append(ruleReasoning::getDoNothingRule()); // just as a hack
        UCB(i) = -22.;
      }
      if (DEBUG>1) {
        ground_actions(i)->write(); cout<<" UCB="<<UCB(i)<<"   (q="<<s_a_info->getQvalue(i)<<",  visits="<<s_a_info->getVisits(i)<<")"<<endl;
      }
    }
    uint id_opt;
    if (untried_action_ids.N > 0)
      id_opt = untried_action_ids(rnd.num(untried_action_ids.N));
    else
      id_opt = UCB.maxIndex();
    if (DEBUG>0) {
      cout<<" --> Chosen action: "; ground_actions(id_opt)->write(); cout<<endl;
    }
    DEBUG__UCT_ACTIONS(t) = ground_actions(id_opt);
    uint flag;
    TL::State s_suc;
    double ruleOutcome_value = TL::ruleReasoning::calcSuccessorState(s, rules.elem(id_opt), flag, s_suc, true);
  
    double reward_suc = 0.;
    runEpisode(reward_suc, s_suc, t+1);  // recursive call
    reward_suc = postprocessValue(reward_suc, flag);
    value = reward_s +  discount * reward_suc;
    if (use_ruleOutcome_rewards) {
      value += ruleOutcome_value;
    }
    // update Q-value
    s_a_info->increaseVisits(id_opt);
    double old_q = s_a_info->getQvalue(id_opt);
    double new_q = old_q + (1 / (1.0 * s_a_info->getVisits(id_opt))) * (value - old_q);
    s_a_info->setQvalue(id_opt, new_q);
    if (DEBUG>0) {
      PRINT(reward_suc);
      PRINT(ruleOutcome_value);
      PRINT(value);
      PRINT(old_q);
      PRINT(new_q);
    }
  }
  
  if (DEBUG>0) {
    cout<<"NID_UCT::runEpisode() [END]"<<endl;
  }
}


TL::Atom* TL::NID_UCT::generateAction(const TL::State& s, uint max_runs) {
  uint DEBUG = 0;
  killAtomLctionsInfo(); // full replanning... comment if not desired
  uint i, k;
  DEBUG__UCT_ACTIONS.resize(horizon);
  for (k=0; k<max_runs; k++) {
    for (i=0; i<numEpisodes; i++) {
      DEBUG__UCT_ACTIONS.setUni(NULL);
      if (i%10 == 0) cout<<"."<<std::flush;
      double dummy_reward;
      runEpisode(dummy_reward, s, 0);
      if (DEBUG>1) {cout<<i<<":  "; write(DEBUG__UCT_ACTIONS); cout<<endl;}
    }
    // get maximum q value for s
    AtomLctionsInfo* s_a_info = getAtomLctionsInfo(s);
    if (DEBUG>1) {
      cout<<"Q values for starting state for states tried more than 0 times:"<<endl;
      FOR1D(ground_actions, i) {
        if (s_a_info->getVisits(i) > 0) {
          ground_actions(i)->write(); cout<<": "<<s_a_info->getQvalue(i)<<"   ("<<s_a_info->getVisits(i)<<" visits)"<<endl;
        }
      }
    }
    uint max_id = s_a_info->values.maxIndex();
#define THRESHOLD_UCT 0.0005
    if (s_a_info->values(max_id) > THRESHOLD_UCT)
      return ground_actions(max_id);
    MT_MSG("NID_UCT: No good action found --> Retry!");
  }
  MT_MSG("NID_UCT: Still no good action found. I'll give up :-(.");
  return NULL;
}


TL::AtomLctionsInfo* TL::NID_UCT::getAtomLctionsInfo(const TL::State& s) {
  uint i;
  FOR1D(s_a_infos, i) {
    if (s_a_infos(i)->s == s)
      return s_a_infos(i);
  }
  // create new one
  AtomLctionsInfo* s_a_info = new AtomLctionsInfo(s, ground_actions.N);
  s_a_infos.append(s_a_info);
  return s_a_info;
}


void TL::NID_UCT::setC(double c) {
  this->c = c;
}

void TL::NID_UCT::setNumEpisodes(uint numEpisodes) {
  this->numEpisodes = numEpisodes;
}



// --------------------------------------
//    AtomLctionsInfo
// --------------------------------------


TL::AtomLctionsInfo::AtomLctionsInfo(const TL::State& _s, uint num_actions) : s(_s) {
  values.resize(num_actions);
  values.setUni(0.);
  visits.resize(num_actions);
  visits.setUni(0.);
}

TL::AtomLctionsInfo::~AtomLctionsInfo() {
}

uint TL::AtomLctionsInfo::getVisits() {
  return sum(visits);
}

uint TL::AtomLctionsInfo::getVisits(uint action_id) {
  return visits(action_id);
}

void TL::AtomLctionsInfo::increaseVisits(uint action_id) {
  visits(action_id)++;
}

double TL::AtomLctionsInfo::getQvalue(uint action_id) {
  return values(action_id);
}

void TL::AtomLctionsInfo::setQvalue(uint action_id, double value) {
  values(action_id) = value;
}

