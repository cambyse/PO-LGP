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


#include "plan.h"





// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    Rewards
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

TL::Reward::Reward() {
}

TL::Reward::Reward(uint _reward_type) {
  reward_type = _reward_type;
}


// PredicateReward

TL::PredicateReward::PredicateReward(TL::PredicateInstance* _pt) : Reward(REWARD_TYPE__PREDICATE_INSTANCE) {
  CHECK(_pt->positive, "");
  pi = _pt;
}

double TL::PredicateReward::evaluate(const State& s) const {
  if (TL::LogicEngine::holds(s, pi))
    return 1.0;
  else
    return 0.0;
}

bool TL::PredicateReward::satisfied(const State& s) const {
  double result = evaluate(s);
  return TL::areEqual(1., result);
}

bool TL::PredicateReward::possible(const State& s, TL::LogicEngine* le) const {
  TL::Predicate* p_OUT = le->getPredicate(MT::String("out"));
  uint i;
  FOR1D(pi->args, i) {
    uintA args(1);  args(0)=pi->args(i);
    TL::PredicateInstance* pi_out = le->getPI(p_OUT, true, args);
    if (le->holds(s, pi_out))
      return false;
  }
  return true;
}

void TL::PredicateReward::writeNice(ostream& out) const {
  pi->writeNice(out);
}

void TL::PredicateReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# PredicateReward"<<endl;
  out<<"# "; pi->writeNice(out); out<<endl;
  pi->write(out); out<<endl;
  out.close();
}


void TL::PredicateReward::getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const {
  objects.clear();
  objects.setAppend(pi->args);
}



// PredicateListReward

TL::PredicateListReward::PredicateListReward(PredIA& _pis) : Reward(REWARD_TYPE__PREDICATE_INSTANCE_LIST) {
  pis = _pis;
}

double TL::PredicateListReward::evaluate(const State& s) const {
  if (TL::LogicEngine::holds(s, pis))
    return 1.0;
  else
    return 0.0;
}

bool TL::PredicateListReward::satisfied(const State& s) const {
  double result = evaluate(s);
  if (TL::areEqual(1., result))
    return true;
  else
    return false;
}

bool TL::PredicateListReward::possible(const State& s, TL::LogicEngine* le) const {
  TL::Predicate* p_OUT = le->getPredicate(MT::String("out"));
  uint i, k;
  if (p_OUT != NULL) {
    FOR1D(pis, k) {
      FOR1D(pis(k)->args, i) {
        uintA args(1);  args(0)=pis(k)->args(i);
        TL::PredicateInstance* pt_out = le->getPI(p_OUT, true, args);
        if (le->holds(s, pt_out))
          return false;
      }
    }
  }
  return true;
}

void TL::PredicateListReward::writeNice(ostream& out) const {
  uint i;
  out<<"[" << pis.N  << "]  ";
  FOR1D(pis, i) {
    pis(i)->writeNice(out);out<<" ";
  }
}

void TL::PredicateListReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# PredicateListReward"<<endl;
  uint i;
  FOR1D(pis, i) {
    out<<"# "; pis(i)->writeNice(out); out<<endl;
  }
  out<<pis.N<<endl;
  FOR1D(pis, i) {
    pis(i)->write(out); out<<endl;
  }
  out.close();
}

void TL::PredicateListReward::getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const {
  objects.clear();
  uint i;
  FOR1D(pis, i) {
    objects.setAppend(pis(i)->args);
  }
}


// DisjunctionReward


TL::DisjunctionReward::DisjunctionReward(PredIA& _pis) : Reward(REWARD_TYPE__ONE_OF_PREDICATE_INSTANCE_LIST) {
  this->pis = _pis;
  this->weights.resize(this->pis.N);
  this->weights.setUni(1.0);
}

TL::DisjunctionReward::DisjunctionReward(PredIA& _pis, arr& _weights) : Reward(REWARD_TYPE__ONE_OF_PREDICATE_INSTANCE_LIST) {
  this->pis = _pis;
  this->weights = _weights;
}

double TL::DisjunctionReward::evaluate(const State& s) const {
  uint i;
  double max = 0.0;
  FOR1D(pis, i) {
    if (TL::LogicEngine::holds(s, pis(i))) {
      max = TL_MAX(max, weights(i));
    }
  }
  return max;
}

bool TL::DisjunctionReward::satisfied(const State& s) const {
  uint i;
  FOR1D(pis, i) {
    if (TL::LogicEngine::holds(s, pis(i)))
      return true;
  }
  return false;
}

bool TL::DisjunctionReward::possible(const State& s, TL::LogicEngine* le) const {
  TL::Predicate* p_OUT = le->getPredicate(MT::String("out"));
  uint i, k;
  if (p_OUT != NULL) {
    FOR1D(pis, k) {
      FOR1D(pis(k)->args, i) {
        uintA args(1);  args(0)=pis(k)->args(i);
        TL::PredicateInstance* pt_out = le->getPI(p_OUT, true, args);
        if (le->holds(s, pt_out))
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
  FOR1D(pis, i) {
    out<<"# "; pis(i)->writeNice(out); out<<endl;
  }
  out<<pis.N<<endl;
  FOR1D(pis, i) {
    pis(i)->write(out); out<<endl;
  }
  out.close();
}

void TL::DisjunctionReward::writeNice(ostream& out) const {
  uint i;
  out<<"[" << pis.N  << "]  OR  ";
  FOR1D(pis, i) {
    out<<weights(i)<<":";pis(i)->writeNice(out); out << "  ";
  }
}

void TL::DisjunctionReward::getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const {
  objects.clear();
  uint i;
  FOR1D(pis, i) {
    objects.setAppend(pis(i)->args);
  }
}


// MaximizeFunctionReward

TL::MaximizeFunctionReward::MaximizeFunctionReward() : Reward(REWARD_TYPE__MAXIMIZE_FUNCTION) {
  fi = NULL;
}

TL::MaximizeFunctionReward::MaximizeFunctionReward(TL::FunctionInstance* _fi) : Reward(REWARD_TYPE__MAXIMIZE_FUNCTION) {
  fi = _fi;
}

double TL::MaximizeFunctionReward::evaluate(const State& s) const {
  uint i;
  if (fi->f->category == TL_PRIMITIVE) {
    FOR1D(s.fv_prim, i) {
      if (s.fv_prim(i)->f == fi->f && s.fv_prim(i)->args == fi->args)
        return s.fv_prim(i)->value;
    }
    CHECK(i==s.fv_prim.N, "");
  }
  else {
    CHECK(s.derivedDerived, "");
    FOR1D(s.fv_derived, i) {
      if (s.fv_derived(i)->f == fi->f && s.fv_derived(i)->args == fi->args)
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
  if (fi->f->type == TL_FUNC_COUNT) {
    CountFunction* cf = (CountFunction*) fi->f;
    if (cf->max_value == -1) {
      if (cf->countedPred->d == 1) {
        double value = LogicEngine::getValue(fi->f, s);
        uintA constants;
        LogicEngine::getConstants(s, constants);
        if (TL::areEqual(value, constants.N-1))
          return true;
        else
          return false;
      }
    }
    else {
      int value = (int) LogicEngine::getValue(fi->f, s);
//       PRINT(value);
//       PRINT(cf->max_value);
      if (value == cf->max_value)
        return true;
      else
        return false;
    }
  }
  else if (fi->f->type == TL_FUNC_REWARD) {
    RewardFunction* grf = dynamic_cast<RewardFunction*>(fi->f);
    return LogicEngine::holds(s, grf->grounded_pis);
  }
  return false; // is never satisfied...
}


bool TL::MaximizeFunctionReward::possible(const State& s, TL::LogicEngine* le) const {
  return true; // is always true
}


void TL::MaximizeFunctionReward::writeNice(ostream& out) const {
  if (fi->f->type == TL_FUNC_COUNT) {
    out<<"maximize #"<<((CountFunction*) (fi->f))->countedPred->name;
  }
  else {
    fi->writeNice(out);
    out<<"   ";
    fi->f->writeNice(out);
    out<<"   ";
    out<<"   ";
    out<<"MAXIMIZE_FUNCTION";
  }
}

void TL::MaximizeFunctionReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# MaximizeFunctionReward"<<endl;
  fi->writeNice(out); out<<endl;
  out<<REWARD_TYPE__MAXIMIZE_FUNCTION<<endl;
  fi->write(out); out<<endl;
  out.close();
}

void TL::MaximizeFunctionReward::getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const {
  objects.clear();
  if (fi->f->type == TL_FUNC_COUNT) {
    // Reward objects are those for which predicate instances don't hold yet
    PredIA pis;
    le->generatePredicateInstances(((CountFunction*) (fi->f))->countedPred, le->constants, pis);
    pis.memMove = true;
    uint i;
    FOR1D_DOWN(pis, i) {
      if (le->holds(*s, pis(i)))
        pis.remove(i);
    }
    FOR1D(pis, i) {
      objects.setAppend(pis(i)->args);
    }
  }
  else
    HALT("Warning: get reward objects has not been implemented in a korrekt way yet");
}




// NotTheseStatesReward


TL::NotTheseStatesReward::NotTheseStatesReward(const StateA& _undesired_states) : Reward(REWARD_TYPE__NOT_THESE_STATES) {
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


bool TL::NotTheseStatesReward::possible(const State& s, TL::LogicEngine* le) const {
  return true; // is always true
}


void TL::NotTheseStatesReward::writeNice(ostream& out) const {
  out<<"REWARD_TYPE__NOT_THESE_STATES  "<<endl;
  uint i;
  out<<undesired_states.N<<" undesired states:"<<endl;
  FOR1D(undesired_states, i) {
    out<<i<<": ";
    undesired_states(i)->writeNice(out, false, true);
    out<<endl;
  }
}

void TL::NotTheseStatesReward::write(const char* filename) const {
  ofstream out(filename);
  out<<reward_type<<endl;
  out<<"# NotTheseStatesReward"<<endl;
  out<<REWARD_TYPE__NOT_THESE_STATES<<endl;
  out<<"# Number of undesired states = "<<undesired_states.N << endl;
  uint i;
  FOR1D(undesired_states, i) {
    undesired_states(i)->write(out);
    out<<endl;
  }
  out.close();
}

void TL::NotTheseStatesReward::getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const {
  objects.clear();
  NIY;
}







// Reward helpers

TL::Reward* TL::readReward(const char* filename, TL::LogicEngine& le) {
  ifstream in(filename);
  MT::skip(in);
  uint type;
  in >> type;
  MT::skip(in);
  if (type == REWARD_TYPE__PREDICATE_INSTANCE) {
    PredicateInstance* pi = readPredicateInstance(in, le.p_prim, le.p_derived, le.p_comp, le.f_prim, le.f_derived);
    uint i;
    FOR1D(pi->args, i) {
      if (le.constants.findValue(pi->args(i)) < 0)
        HALT("Reward uses unknown argument "<<pi->args(i));
    }
    pi = le.getPIorig(pi);
    return new PredicateReward(pi);
  }
  else if (type == REWARD_TYPE__PREDICATE_INSTANCE_LIST) {
    PredIA pis;
    while (MT::skip(in) != -1) {
      PredicateInstance* pi = readPredicateInstance(in, le.p_prim, le.p_derived, le.p_comp, le.f_prim, le.f_derived);
      uint i;
      FOR1D(pi->args, i) {
        if (le.constants.findValue(pi->args(i)) < 0)
          HALT("Reward uses unknown argument "<<pi->args(i));
      }
      pi = le.getPIorig(pi);
      pis.append(pi);
    }
    return new PredicateListReward(pis);
  }
  else if (type == REWARD_TYPE__MAXIMIZE_FUNCTION) {
    FunctionInstance* fi = readFunctionInstance(in, le.f_prim, le.f_derived);
    uint i;
    FOR1D(fi->args, i) {
      if (le.constants.findValue(fi->args(i)) < 0)
        HALT("Reward uses unknown argument "<<fi->args(i));
    }
    fi = le.getFIorig(fi);
    return new MaximizeFunctionReward(fi);
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


PredIA DEBUG_actions;

TL::PredicateInstance* TL::SST::generateAction(double& value, const TL::State& s0, const Reward& reward, uint branch, uint tau, double discount, const WorldAbstraction& wa) {
  uint DEBUG = 0;
  if (DEBUG_actions.N == 0)
    DEBUG_actions.resize(tau);
  if (DEBUG>0) {
    cout<<"+ SST - start  tau="<<tau<<endl;
    s0.writeNice(); cout<<endl;
  }
  if (tau == 5)
    cout<<"."<<std::flush;
  double reward_s0 = reward.evaluate(s0);
  double reward_tree = 0.;
  uint i, b;
  TL::PredicateInstance* action = NULL;
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
            cout<<"++ Omitting at "<<tau<<" "; wa.ground_actions(i)->writeNice(); cout<<" b="<<b<<endl;
          }
          action_value = -10000.;
          break;
        }
        else {
          double tree_value;
          if (DEBUG>1) {
            cout<<"++ going down tau="<<tau<<" "; wa.ground_actions(i)->writeNice(); cout<<" b="<<b<<endl;
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
      cout<<"Best Action for "; uint t; for (t=0; t<DEBUG_actions.N-tau; t++){DEBUG_actions(t)->writeNice(); cout<<" ";}
      cout<<"  -->  ";action->writeNice(); cout<<endl;
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


TL::NID_Planner::NID_Planner(TL::LogicEngine* le, double noise_scaling_factor) {
  this->le = le;
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
  TL::PredicateInstance* last_action = NULL;
  uint i;
  FOR1D_(ground_rules, i) {
    if (last_action != ground_rules.elem(i)->action  &&
        ground_rules.elem(i)->action->pred->id != TL_DEFAULT_ACTION_PRED__ID) {
      last_action = ground_rules.elem(i)->action;
      ground_actions.setAppend(last_action);
    }
  }
  
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


double TL::NID_Planner::sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::PredicateInstance* action) const {
  return RuleEngine::calcSuccessorState(s_prev, ground_rules, action, flag, s_suc, true);
}



// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
//    NID-SST
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------



TL::NID_SST::NID_SST(TL::LogicEngine* le, uint branch, double noise_scaling_factor) : NID_Planner(le, noise_scaling_factor) {
  this->branch = branch;
}


TL::PredicateInstance* TL::NID_SST::generateAction(const TL::State& current_state, uint max_runs) {
  double value;
  uint i;
  for (i=0; i<max_runs; i++) {
     TL::PredicateInstance* action = SST::generateAction(value, current_state, *reward, branch, horizon, discount, *this);
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


TL::NID_UCT::NID_UCT(TL::LogicEngine* le, double noise_scaling_factor) : NID_Planner(le, noise_scaling_factor) {
  c = 1.0;
}


TL::NID_UCT::~NID_UCT() {
  killStateActionsInfo();
}

void TL::NID_UCT::killStateActionsInfo() {
  listDelete(s_a_infos);
}

PredIA DEBUG__UCT_ACTIONS;

void TL::NID_UCT::runEpisode(double& value, const TL::State& s, uint t) {
  uint DEBUG = 0;
  if (t==0)
    DEBUG = 0;
  else
    DEBUG = 0;
  if (DEBUG>0) {
    cout<<"NID_UCT::runEpisode() [START]"<<endl;
    PRINT(t);
    cout<<"State: "; s.writeNice(cout, true); cout<<endl;
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
    StateActionsInfo* s_a_info = getStateActionsInfo(s);
    uint i;
    arr UCB(ground_actions.N);
    TL::RuleSet rules;
    uintA untried_action_ids;
    if (DEBUG>0) {cout<<"visits(s)="<<s_a_info->getVisits()<<endl;}
    FOR1D(ground_actions, i) {
      TL::State dummy_state = s;
      TL::Rule* r = RuleEngine::uniqueCoveringRule_groundedRules_groundedAction(ground_rules, s, ground_actions(i));
      if (r != NULL) {
        rules.append(r);
        if (s_a_info->getVisits(i) == 0) {
          untried_action_ids.append(i);
          UCB(i) = -11.;
        }
        else
          UCB(i) = s_a_info->getQvalue(i)  +  c * sqrt(log(s_a_info->getVisits()) / (1.0 * s_a_info->getVisits(i)));
      }
      else {
        rules.append(RuleEngine::getDoNothingRule()); // just as a hack
        UCB(i) = -22.;
      }
      if (DEBUG>1) {
        ground_actions(i)->writeNice(); cout<<" UCB="<<UCB(i)<<"   (q="<<s_a_info->getQvalue(i)<<",  visits="<<s_a_info->getVisits(i)<<")"<<endl;
      }
    }
    uint id_opt;
    if (untried_action_ids.N > 0)
      id_opt = untried_action_ids(rnd.num(untried_action_ids.N));
    else
      id_opt = UCB.maxIndex();
    if (DEBUG>0) {
      cout<<" --> Chosen action: "; ground_actions(id_opt)->writeNice(); cout<<endl;
    }
    DEBUG__UCT_ACTIONS(t) = ground_actions(id_opt);
    uint flag;
    TL::State s_suc;
    double ruleOutcome_value = TL::RuleEngine::calcSuccessorState(s, rules.elem(id_opt), flag, s_suc, true);
  
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


TL::PredicateInstance* TL::NID_UCT::generateAction(const TL::State& s, uint max_runs) {
  uint DEBUG = 0;
  killStateActionsInfo(); // full replanning... comment if not desired
  uint i, k;
  DEBUG__UCT_ACTIONS.resize(horizon);
  for (k=0; k<max_runs; k++) {
    for (i=0; i<numEpisodes; i++) {
      DEBUG__UCT_ACTIONS.setUni(NULL);
      if (i%10 == 0) cout<<"."<<std::flush;
      double dummy_reward;
      runEpisode(dummy_reward, s, 0);
      if (DEBUG>1) {cout<<i<<":  "; writeNice(DEBUG__UCT_ACTIONS); cout<<endl;}
    }
    // get maximum q value for s
    StateActionsInfo* s_a_info = getStateActionsInfo(s);
    if (DEBUG>1) {
      cout<<"Q values for starting state for states tried more than 0 times:"<<endl;
      FOR1D(ground_actions, i) {
        if (s_a_info->getVisits(i) > 0) {
          ground_actions(i)->writeNice(); cout<<": "<<s_a_info->getQvalue(i)<<"   ("<<s_a_info->getVisits(i)<<" visits)"<<endl;
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


TL::StateActionsInfo* TL::NID_UCT::getStateActionsInfo(const TL::State& s) {
  uint i;
  FOR1D(s_a_infos, i) {
    if (s_a_infos(i)->s == s)
      return s_a_infos(i);
  }
  // create new one
  StateActionsInfo* s_a_info = new StateActionsInfo(s, ground_actions.N);
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
//    StateActionsInfo
// --------------------------------------


TL::StateActionsInfo::StateActionsInfo(const TL::State& _s, uint num_actions) : s(_s) {
  values.resize(num_actions);
  values.setUni(0.);
  visits.resize(num_actions);
  visits.setUni(0.);
}

TL::StateActionsInfo::~StateActionsInfo() {
}

uint TL::StateActionsInfo::getVisits() {
  return sum(visits);
}

uint TL::StateActionsInfo::getVisits(uint action_id) {
  return visits(action_id);
}

void TL::StateActionsInfo::increaseVisits(uint action_id) {
  visits(action_id)++;
}

double TL::StateActionsInfo::getQvalue(uint action_id) {
  return values(action_id);
}

void TL::StateActionsInfo::setQvalue(uint action_id, double value) {
  values(action_id) = value;
}

