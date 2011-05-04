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
    

#ifndef TL_plan_h
#define TL_plan_h

#include "ruleEngine.h"



namespace TL {



// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          REWARDS
//
// -------------------------------------------------------------
// -------------------------------------------------------------

// REWARD_TYPE__PREDICATE_INSTANCE = predicate instance such as on(a,b)=true
// REWARD_TYPE__PREDICATE_INSTANCE_LIST = list of predicate instances
// MAXIMIZE_FUNCTION = try to make function as big as possible
//     --> Advantage: can work on expectations in case of beliefs over states
#define REWARD_TYPE__PREDICATE_INSTANCE 1
#define REWARD_TYPE__PREDICATE_INSTANCE_LIST 2
#define REWARD_TYPE__MAXIMIZE_FUNCTION 3
#define REWARD_TYPE__NOT_THESE_STATES 4
#define REWARD_TYPE__ONE_OF_PREDICATE_INSTANCE_LIST 5


class Reward {
  public:
    Reward();
    Reward(uint reward_type);
    virtual ~Reward() {}
    
    virtual double evaluate(const State& s) const = 0;
    virtual bool satisfied(const State& s) const = 0;
    virtual bool possible(const State& s, TL::LogicEngine* le) const = 0;
    
    virtual void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const = 0;
    
    virtual void writeNice(ostream& out = cout) const {};
    virtual void write(const char* filename) const = 0;
    
//     virtual void sampleFinalState(PredIA& fixed_properties, const State& s0) const = 0; // needed for backwards reasoning
    
    uint reward_type;
};


class PredicateReward : public Reward {
  public:
    TL::PredicateInstance* pi;
    
    PredicateReward(TL::PredicateInstance* pi);
    ~PredicateReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s, TL::LogicEngine* le) const;
    
    void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s = NULL) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class PredicateListReward : public Reward {
  public:
    PredIA pis;
    
    PredicateListReward(PredIA& pis);
    ~PredicateListReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s, TL::LogicEngine* le) const;
    
    void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};


class DisjunctionReward : public Reward {
  public:
    PredIA pis;
    arr weights;

    DisjunctionReward(PredIA& pis);
    DisjunctionReward(PredIA& pis, arr& weights);
    ~DisjunctionReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s, TL::LogicEngine* le) const;
    
    void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class MaximizeFunctionReward : public Reward {
  public:
    TL::FunctionInstance* fi;
    
    MaximizeFunctionReward(); // --> maximize function
    MaximizeFunctionReward(TL::FunctionInstance* fv); // --> maximize function
    ~MaximizeFunctionReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s, TL::LogicEngine* le) const;
    
    void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class NotTheseStatesReward : public Reward {
  public:
    StateA undesired_states;
    
    NotTheseStatesReward(const StateA& undesired_states); // --> maximize function
    ~NotTheseStatesReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s, TL::LogicEngine* le) const;
    
    void getRewardObjects(uintA& objects, LogicEngine* le, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};

Reward* readReward(const char* filename, TL::LogicEngine& le);











// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          World Abstraction
//
// -------------------------------------------------------------
// -------------------------------------------------------------


// Provides knowledge about actions and state transitions.
// Used by SST planner.
class WorldAbstraction {
  public:
    PredIA ground_actions;
    
    // returns false if action not applicable
    virtual double sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::PredicateInstance* action) const  = 0;
    virtual double postprocessValue(double value, uint flag) const {return value;}
};




// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          SST
//
// -------------------------------------------------------------
// -------------------------------------------------------------

namespace SST {
  TL::PredicateInstance* generateAction(double& value, const TL::State& current_state, const Reward& reward, uint branch, uint T, double discount, const WorldAbstraction& wa);
}




// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          NID Planner
//
// -------------------------------------------------------------
// -------------------------------------------------------------


class NID_Planner : public WorldAbstraction {
  protected:
    TL::LogicEngine* le;
    TL::RuleSet ground_rules;
    double discount;
    arr discount_pow; // for faster code
    uint horizon;
    Reward* reward;
    double noise_scaling_factor;
    bool use_ruleOutcome_rewards;
    arr expected_rule_rewards;
  
  public:
    NID_Planner(TL::LogicEngine* le, double noise_scaling_factor);
    virtual ~NID_Planner();
    
    virtual TL::PredicateInstance* generateAction(const TL::State& current_state, uint max_runs = 1) = 0;
    
    void setDiscount(double discount);
    virtual void setHorizon(uint horizon); // planning horizon
    void setGroundRules(TL::RuleSet& ground_rules);
    virtual void setReward(TL::Reward* reward);
    TL::Reward* getReward() {return reward;}
    
    double sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::PredicateInstance* action) const;
    double postprocessValue(double value, uint flag) const;
};






// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          NID_SST
//
// -------------------------------------------------------------
// -------------------------------------------------------------

class NID_SST : public NID_Planner {
  uint branch;
  
  public :
    NID_SST(TL::LogicEngine* le, uint branch, double noise_scaling_factor);
    
    TL::PredicateInstance* generateAction(const TL::State& current_state, uint max_runs = 1);
};








// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          NID_UCT
//
// -------------------------------------------------------------
// -------------------------------------------------------------


struct StateActionsInfo {
  public:
    const TL::State& s;
    arr values;
    uintA visits;

    StateActionsInfo(const TL::State& s, uint num_actions);
    ~StateActionsInfo();
    
    uint getVisits();
    uint getVisits(uint action_id);
    void increaseVisits(uint action_id);
    
    double getQvalue(uint action_id);
    void setQvalue(uint action_id, double value);
};


class NID_UCT : public NID_Planner {
  double c;
  uint numEpisodes;
  
  MT::Array< StateActionsInfo* > s_a_infos;
  
  StateActionsInfo* getStateActionsInfo(const TL::State& s);
  void killStateActionsInfo();
  
  void runEpisode(double& reward, const TL::State& s, uint t);
  
  public:
    NID_UCT(TL::LogicEngine* le, double noise_scaling_factor);
    ~NID_UCT();
    
    void setC(double c);
    void setNumEpisodes(uint numEpisodes);
    
    TL::PredicateInstance* generateAction(const TL::State& current_state, uint max_runs = 1);
};





}




#endif // TL_plan_h

