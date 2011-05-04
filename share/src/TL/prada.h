/*  
    Copyright 2010   Tobias Lang
    
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
    

#ifndef TL_prada_h
#define TL_prada_h

#include "plan.h"



namespace TL {




// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          PRADA
//
// -------------------------------------------------------------
// -------------------------------------------------------------


// -------------------------------------------------------------
// RANDOM VARIABLES

#define RV_TYPE__PRED 1
#define RV_TYPE__FUNC 2
#define RV_TYPE__FUNC_EXPECT 3

class LogicRV {
  public:
    uint id;
    uint dim;
    uintA range;
    arr P; // over time;   dim-1: time,   dim-2: value
    bool changeable; // can change value over time
    uint type;
    
    virtual ~LogicRV() {}

    virtual void write(ostream& os = cout) = 0;
};

class PredicateRV : public LogicRV {
  public:
    TL::PredicateInstance* pi;
    PredicateRV() {type = RV_TYPE__PRED;}
    void write(ostream& os = cout);
};

class FunctionRV : public LogicRV {
  public:
    TL::FunctionInstance* fi;
    FunctionRV() {type = RV_TYPE__FUNC;}
    void write(ostream& os = cout);
};

// Calculates only Expected Value of Random Variable.
// No true distribution is maintained!!!
class ExpectationFunctionRV : public FunctionRV {
  // in array P, expectation values are stored
  public:
    ExpectationFunctionRV() {type = RV_TYPE__FUNC_EXPECT; dim=1;}
    void write(ostream& os = cout);
};


typedef MT::Array< LogicRV* > LVA;
typedef MT::Array< PredicateRV* > PredVA;
typedef MT::Array< FunctionRV* > FuncVarA;

void write(PredVA& vars);
void write(FuncVarA& vars);



// -------------------------------------------------------------
// RV MANAGER 
// manages RV objects for efficiency

class RV_Manager {
  
  // Random variables
  PredVA pvA; // 2dim for each predicate
  FuncVarA fvA;
  
  LVA vA; // redundant container for all variables; contains both, pvA and fvA
  
  // Function instances
  FuncIA fiA;
  
  public:
    RV_Manager(const PredA& preds, const FuncA& funcs, const uintA& constants);
    
    PredicateRV* pi2v(TL::PredicateInstance* pi) const;
    FunctionRV* fi2v(TL::FunctionInstance* fi) const;
    LogicRV* id2var(uint id_var) const;
    
    TL::FunctionInstance* getFVW(TL::Function* f, uintA& sa);
    
    void set(TL::PredicateInstance* pt, PredicateRV* var);
    void set(TL::FunctionInstance* pt, FunctionRV* var);
    
    PredA preds;
    FuncA funcs;
    uintA constants;
};






// -------------------------------------------------------------
// NID Dynamic Bayesian Network
//

class NID_DBN {
  public:
  
  TL::LogicEngine* le;
  TL::RuleSet ground_rules;
  double noise_softener;
  uint horizon;
  uintA objects;  // ojects over which net is built

  // Random variables
  RV_Manager* rvm;
  uint start_id_rv__predicate, start_id_rv__function;
  uint num_state_vars;
  PredVA rvs_state__p_prim;
  PredVA rvs_state__p_derived;
  FuncVarA rvs_state__f_prim;
  FuncVarA rvs_state__f_derived;
  PredVA rvs_action;
  LVA vars_context; // for faster code --> which vars are used as context
  
  arr rvs_rules_simple;   // P(\phi_r | s)
  arr rvs_rules; // P(\phi_r | -\phi_r', s)     2 dim: (1) timesteps,  (2) rules
  
  // Helping structures
  // Impacts only on primitives
  arr impacts_V_p; // on vars as a whole
  arr impacts_val_p; // on single values
  arr impacts_V_f; // on vars as a whole
  arr impacts_val_f; // on single values
  // dim_1 actions,  dim_2 arguments,  dim_3 rules
  uintA action2rules;
  uintA action2rules_no; // 2 dim; how many rules per constellation

  void create_dbn_structure(const uintA& constants, const PredA& preds, const FuncA& funcs, const PredA& actions);
  void create_dbn_params();
  
    
  public:
    NID_DBN(const uintA& objects, const PredA& preds, const FuncA& funcs, const PredA& actions, TL::RuleSet& ground_rules, double noise_softener, uint horizon, TL::LogicEngine* le);
    ~NID_DBN();
    
    // Inference
    void inferRules(uint t);  // from time-slice at t
    void inferState(uint t, TL::PredicateInstance* action);  // from action and time-slice at t-1
    void inferState(uint t, TL::PredicateInstance* action, double given_action_weight);  // from action and time-slice at t-1
    void inferStates(const PredIA& given_action_sequence); // for given_action_sequence
    
    // Set evidence
    void setAction(uint t, TL::PredicateInstance* action);
    void setState(const PredIA& pis, const FuncVA& fvs, uint t);
    void setStateUniform(uint t);
    
    // Comparing probabilities
    double log_probability(uint t, const State& state) const;
    double belief_difference(uint t, const arr& probs_p_prim, const arr& probs_f_prim);
    
    // Misc
    void calcDerived(uint t);
    void checkStateSoundness(uint t, bool omit_derived = false);
    void getBelief(uint t, arr& beliefs_p_prim, arr& beliefs_f_prim) const;
    
    // Writing
    void writeAllStates(bool prim_only = false, double threshold = 0.0, ostream& out = cout) const;
    void writeState(uint t, bool prim_only = false, double threshold = 0.0, ostream& out = cout) const;
    void writeStateSparse(uint t, bool prim_only, ostream& out = cout) const;
    void writeDAI(ostream& out = cout) const;
};





// -------------------------------------------------------------
//   PRADA rewards  -->  working on beliefs

class PRADA_Reward {
  public :
    // Random variables --> Reals
    virtual double evaluate_prada_reward(const NID_DBN& net, uint t) = 0;
};



// -------------------------------------------------------------
//    PRADA

class PRADA : public NID_Planner {

  protected:
    // PRADA's parameters
    uint num_samples;
    double noise_softener;
    double threshold_reward;
    
    bool any_sensible_action;
    
    // true:   value = sum_t P(r_t)
    // false:  value = max_t P(r_t)
    bool reward_calculation__sum;
    
    // true:   first action of max action seq
    // false:  normalized sum over all action seqs starting with that action
    bool action_choice__max;
    
    // for statistics
    uintA action_choices;
    
    PRADA_Reward* prada_reward;
    PRADA_Reward* convert_reward(PredicateReward* reward);
    PRADA_Reward* convert_reward(MaximizeFunctionReward* reward);
    PRADA_Reward* convert_reward(PredicateListReward* reward);
    PRADA_Reward* convert_reward(DisjunctionReward* reward);
    PRADA_Reward* convert_reward(NotTheseStatesReward* reward);
    
    void setState(const TL::State& s, uint t);
    
    // Rather high-level methods
    void sampleActionsAndInfer(PredIA& plan); // one plan
    // some actions are fixed
    void sampleActionsAndInfer(PredIA& plan, const PredIA& fixed_actions); // one plan
    // base function --> specify which net to sample on
    void sampleActionsAndInfer(PredIA& plan, const PredIA& fixed_actions, NID_DBN* net, uint local_horizon);
  public:
    // for complete action sequence (= all actions fixed), calculate posterior over hidden state variables
    void infer(const PredIA& plan); // one plan; infer until plan-length (no additional future action sampling)
    double inferStateRewards();
    double inferStateRewards(uint horizon);
    double inferStateRewards_limited_sum(uint horizon);
    double inferStateRewards_limited_max(uint horizon);
    double inferStateRewards_single(uint t);
    double calcRuleRewards(const PredIA& actions);
    // samples "num_samples" plans and returns best
    virtual bool plan(PredIA& best_actions, double& bestValue, uint num_samples); // various plans
    
    // DBN Construction
    void build_dbn(const uintA& constants, const PredA& preds, const FuncA& funcs, const PredA& actions);
    // Calculates from rules and rewards which predicates, functions, and actions to use
    void build_dbn(const uintA& constants);
    PredA dbn_preds;
    FuncA dbn_funcs;
    void calc_dbn_concepts();
    
    
  
  public :
    PRADA(TL::LogicEngine* le);
    ~PRADA();
    
    // *** HIGH-LEVEL ROUTINES ***
    void generatePlan(PredIA& plan, double& planValue, const TL::State& s, uint max_runs = 1);
    TL::PredicateInstance* generateAction(const TL::State& s, uint max_runs = 1);
    
    virtual void setReward(Reward* reward);
    void setReward(Reward* reward, PRADA_Reward* prada_reward); // use only if you have domain specific knowledge!
    void setNoiseSoftener(double noise_softener);
    void setNumberOfSamples(uint num_samples);
    void setThresholdReward(double threshold_reward);
    void setRewardCalculation(bool reward_calculation__sum);
    void setActionChoice(bool action_choice__max);
    virtual void setStartState(const TL::State& s0);
    
    void writeState(uint t, ostream& out = cout);
    void writeStateSparse(uint t, bool prim_only, ostream& out = cout);
    
    NID_DBN* net;
};






// -------------------------------------------------------------
//    A-PRADA

class A_PRADA : public PRADA {
  
  PredIA last_seq;
  double last_value;
  
  // manipulations
  double shorten_plan(PredIA& seq_best, const PredIA& seq, double value_old);

  public :
    A_PRADA(TL::LogicEngine* le);
    ~A_PRADA();
    
    virtual TL::PredicateInstance* generateAction(const TL::State& current_state, uint max_runs = 1);

    void reset();
};






}




#endif // TL_prada_h


