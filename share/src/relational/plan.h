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
    

#ifndef TL_plan_h
#define TL_plan_h

#include "ruleReasoning.h"



namespace TL {



// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          REWARDS
//
// -------------------------------------------------------------
// -------------------------------------------------------------


class Reward {
  
  public:
    // reward_literal = literal such as on(a,b)=true
    // reward_literalList = list of literals
    // MAXIMIZE_FUNCTION = try to make function as big as possible
    //     --> Advantage: can work on expectations in case of beliefs over states
    enum RewardType {reward_literal, reward_literalList, reward_maximize_function, reward_not_these_states, reward_one_of_literal_list};
    
    
    Reward();
    Reward(RewardType reward_type);
    virtual ~Reward() {}
    
    virtual double evaluate(const State& s) const = 0;
    virtual bool satisfied(const State& s) const = 0;
    virtual bool possible(const State& s) const = 0;
    
    virtual void getRewardObjects(uintA& objects, const TL::State* s) const = 0;
    
    virtual void writeNice(ostream& out = cout) const {};
    virtual void write(const char* filename) const = 0;
    
//     virtual void sampleFinalState(LitL& fixed_properties, const State& s0) const = 0; // needed for backwards reasoning
    
    RewardType reward_type;
};


class LiteralReward : public Reward {
  public:
    TL::Literal* lit;
    
    LiteralReward(TL::Literal* lit);
    ~LiteralReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s) const;
    
    void getRewardObjects(uintA& objects, const TL::State* s = NULL) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class LiteralListReward : public Reward {
  public:
    LitL lits;
    
    LiteralListReward(LitL& lits);
    ~LiteralListReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s) const;
    
    void getRewardObjects(uintA& objects, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};


class DisjunctionReward : public Reward {
  public:
    LitL lits;
    arr weights;

    DisjunctionReward(LitL& lits);
    DisjunctionReward(LitL& lits, arr& weights);
    ~DisjunctionReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s) const;
    
    void getRewardObjects(uintA& objects, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class MaximizeFunctionReward : public Reward {
  public:
    TL::FunctionAtom* fa;
    LitL important_literals; // set by hand!
    
    MaximizeFunctionReward(); // --> maximize function
    MaximizeFunctionReward(TL::FunctionAtom* fa); // --> maximize function
    ~MaximizeFunctionReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s) const;
    
    void getRewardObjects(uintA& objects, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};



class NotTheseStatesReward : public Reward {
  public:
    StateL undesired_states;
    
    NotTheseStatesReward(const StateL& undesired_states); // --> maximize function
    ~NotTheseStatesReward() {}
    
    double evaluate(const State& s) const ;
    bool satisfied(const State& s) const;
    bool possible(const State& s) const;
    
    void getRewardObjects(uintA& objects, const TL::State* s) const;
    
    void writeNice(ostream& out = cout) const;
    void write(const char* filename) const;
};

Reward* readReward(const char* filename);











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
    AtomL ground_actions;
    
    // returns false if action not applicable
    virtual double sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::Atom* action) const  = 0;
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
  TL::Atom* generateAction(double& value, const TL::State& current_state, const Reward& reward, uint branch, uint T, double discount, const WorldAbstraction& wa);
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
    TL::RuleSet ground_rules;
    boolA is_manipulating_rule;  // predicts non-noise changes?
    double discount;
    arr discount_pow; // for faster code
    uint horizon;
    Reward* reward;
    double noise_scaling_factor;
    bool use_ruleOutcome_rewards;
    arr expected_rule_rewards;
  
  public:
    NID_Planner(double noise_scaling_factor);
    virtual ~NID_Planner();
    
    virtual TL::Atom* generateAction(const TL::State& current_state, uint max_runs = 1) = 0;
    
    void setDiscount(double discount);
    virtual void setHorizon(uint horizon); // planning horizon
    void setGroundRules(TL::RuleSet& ground_rules);
    virtual void setReward(TL::Reward* reward);
    TL::Reward* getReward() {return reward;}
    
    double sampleSuccessorState(TL::State& s_suc, uint& flag, const TL::State& s_prev, TL::Atom* action) const;
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
    NID_SST(uint branch, double noise_scaling_factor);
    
    TL::Atom* generateAction(const TL::State& current_state, uint max_runs = 1);
};








// -------------------------------------------------------------
// -------------------------------------------------------------
//
//          NID_UCT
//
// -------------------------------------------------------------
// -------------------------------------------------------------


struct AtomLctionsInfo {
  public:
    const TL::State& s;
    arr values;
    uintA visits;

    AtomLctionsInfo(const TL::State& s, uint num_actions);
    ~AtomLctionsInfo();
    
    uint getVisits();
    uint getVisits(uint action_id);
    void increaseVisits(uint action_id);
    
    double getQvalue(uint action_id);
    void setQvalue(uint action_id, double value);
};


class NID_UCT : public NID_Planner {
  double c;
  uint numEpisodes;
  
  MT::Array< AtomLctionsInfo* > s_a_infos;
  
  AtomLctionsInfo* getAtomLctionsInfo(const TL::State& s);
  void killAtomLctionsInfo();
  
  void runEpisode(double& reward, const TL::State& s, uint t);
  
  public:
    NID_UCT(double noise_scaling_factor);
    ~NID_UCT();
    
    void setC(double c);
    void setNumEpisodes(uint numEpisodes);
    
    TL::Atom* generateAction(const TL::State& current_state, uint max_runs = 1);
};





}




#endif // TL_plan_h

