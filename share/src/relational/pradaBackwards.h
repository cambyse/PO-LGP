#include "prada.h"

namespace TL {

void setUnchangeableValues(NID_DBN& net, const State& state);

void setGoalBelief(Reward* reward, NID_DBN& net, uint t, const State& s0);





class ZICKZACK_PRADA : public PRADA {
  
  State s0;
  RuleSet ground_rules_backwards;
  uint num_samples_forward;
  uint num_samples_backward;
  uint horizon_forward;
  uint horizon_backward;
  uint num_goal_state_samples;
  double final_state_determinism_softener;
  NID_DBN* backward_net;
  
  void forward_sampling(AtomL& sampled_actions, NID_DBN& net, uint horizon);
  void backward_sampling(AtomL& sampled_actions, NID_DBN& net);
  void sampleGoalState(NID_DBN& net, uintA& usedComponents, const TL::State& s);
  void createBetas(arr& betas_p, arr& betas_f, NID_DBN& backward_net, uintA& usedComponents, uint horizon, uint num_plan_samples);
  void combine_alpha_beta(double& total_value, uint& max_d_forward, const NID_DBN& net, uint horizon_forward, const arr& betas_p, const arr& betas_f, double discount);
//   void combine_alpha_beta(double& total_value, uint& max_d_forward, const NID_DBN& net, uint horizon_forward, const MT::Array< arr > & all_betas_p, const MT::Array< arr > & all_betas_f, double discount);
//   void combine_alpha_beta(double& max_value, uint& max_d_forward, uint& max_d_backward, const NID_DBN& net, uint horizon_forward, const arr& betas_p, const arr& betas_f, double discount);

  
  public:
    ZICKZACK_PRADA();
    ~ZICKZACK_PRADA();
    void setStartState(const TL::State& s0);
    bool plan(AtomL& best_actions, double& bestValue, uint num_samples); // various plans
    
    void setGroundRulesBackwards(TL::RuleSet& ground_rules_backwards);
    void setNumberOfSamplesBackward(uint num_samples_backward);
    void setNumberOfSamplesForward(uint num_samples_forward);
    void setHorizonBackward(uint horizon_backward);
    void setHorizonForward(uint horizon_forward);
    void setNumGoalStateSamples(uint num_goal_state_samples);
    void setFinalStateDeterminismSoftener(double final_state_determinism_softener);
    
    void resetGoalMixture(bool within = false);
};



}