#include "ruleExplorer.h"
#include "robotManipulationDomain.h"
#include "logicReasoning.h"
#include "experiments_fixedContexts.h"

// #define ENFORCED_SILENCE

bool USING_IPPC_DOMAIN() {
  return TL::logicObjectManager::getPredicate(MT::String("table")) == NULL;
}


namespace TL {


RuleExplorer::RuleExplorer(RepresentationType _representation, double _complexity_penalty_coeff, 
                           double _p_lower_bound__noise_outcome, double _p_lower_bound__noise_outcome_in_default_rule) {
  this->representation = _representation;
  this->complexity_penalty_coeff = _complexity_penalty_coeff;
  this->p_lower_bound__noise_outcome = _p_lower_bound__noise_outcome;
  this->p_lower_bound__noise_outcome_in_default_rule = _p_lower_bound__noise_outcome_in_default_rule;
  
  moves_exploit.resize(0);
  moves_explore_direct.resize(0);
  moves_explore_planned.resize(0);
  moves.resize(0);
  
  all_experiences.clear();
  
  planned_explore_reward = NULL;
  planned_explore_reward__timestamp = 0;
  
  greedy_epsilon_start = 0.5;
}


RuleExplorer::~RuleExplorer() {
}





// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
//   Confidence calculations (--> true state novelty calculation / job is done here)


double RuleExplorer::calcRuleConfidence(uint rule_id) {
  if (fixedActions.findValue(getRules().elem(rule_id)->action) >= 0)  // fixed actions
    return 99.;
  if (TL::ruleReasoning::isDefaultRule(getRules().elem(rule_id))) // default rule:  never confident
    return 0.;
  return getNumberOfCoveredExperiences(rule_id);
}


void RuleExplorer::calcExploreWeights(arr& explore_weights, TL::Atom* taboo_action) {
  uint DEBUG = 2;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {
    cout<<"calcExploreWeights [START]"<<endl;
    cout<<"taboo_action = "; if (taboo_action != NULL) cout<<*taboo_action; else cout<<" NULL"<<endl;
  }
  
  explore_weights.resize(possibleGroundActions.N);
  
  uint i;
  
  // ----------------------------------------
  // Tabooize all actions of previous direct explorers which have not shown any effects
  uintA number_of_times__action_has_been_direct_explored_without_effect(possibleGroundActions.N);
  number_of_times__action_has_been_direct_explored_without_effect.setUni(0);
  // Tabooizing only IPPC (exclude Robot-Manipulation-Domain)
  if (USING_IPPC_DOMAIN()) {
    FOR1D_DOWN(all_experiences, i) {
      if (!all_experiences(i)->noChange())
        break;
      number_of_times__action_has_been_direct_explored_without_effect(possibleGroundActions.findValue(all_experiences(i)->action))++;
    }
    if (DEBUG>0) {cout<<"Gone down till experience #i="<<i<<" from  "<<(all_experiences.N-1)<<endl;}
  }
  
  AtomL tabooed__direct_explore__actions;
  AtomL strongly_tabooed__direct_explore__actions;
  FOR1D(possibleGroundActions, i) {
    if (number_of_times__action_has_been_direct_explored_without_effect(i) > 0) {
      tabooed__direct_explore__actions.append(possibleGroundActions(i));
    }
    if (number_of_times__action_has_been_direct_explored_without_effect(i) > 1) {
      strongly_tabooed__direct_explore__actions.append(possibleGroundActions(i));
    }
  }
  tabooed__direct_explore__actions.append(fixedActions);
  strongly_tabooed__direct_explore__actions.append(fixedActions);
  if (taboo_action != NULL) {
    tabooed__direct_explore__actions.append(taboo_action);
    strongly_tabooed__direct_explore__actions.append(taboo_action);
  }
  
  TL::logicReasoning::sort(tabooed__direct_explore__actions);
  TL::logicReasoning::sort(strongly_tabooed__direct_explore__actions);
  
  if (DEBUG>0) {
    PRINT(number_of_times__action_has_been_direct_explored_without_effect);
    cout<<"Tabooed direct explore actions:  "<<tabooed__direct_explore__actions<<endl;
    cout<<"Strongly tabooed direct explore actions:  "<<strongly_tabooed__direct_explore__actions<<endl;
  }
  // If all unknown actions are tabooed, consider only strong taboos.
  bool only_consider_strong_taboos = false;
  uint num_unknown_actions = 0;
  uint num_unknown_and_tabooed_actions = 0;
  FOR1D(possibleGroundActions, i) {
    if (!action__is_known(i)) num_unknown_actions++;
    if (!action__is_known(i)  &&  tabooed__direct_explore__actions.findValue(possibleGroundActions(i)) >= 0) num_unknown_and_tabooed_actions++;
  }
  if (num_unknown_actions == num_unknown_and_tabooed_actions)
    only_consider_strong_taboos = true;
  if (DEBUG>0) {PRINT(num_unknown_actions);  PRINT(num_unknown_and_tabooed_actions);  PRINT(only_consider_strong_taboos);}

  
  double min_weight = 10e8;
  FOR1D(possibleGroundActions, i) {
    // Rule confidence
    explore_weights(i) = actions__confidences(i);
    // Taboo actions
    if (taboo_action == possibleGroundActions(i)  ||  tabooed__direct_explore__actions.findValue(possibleGroundActions(i)) >= 0)
      explore_weights(i) = -1.;
    if (explore_weights(i) >= 0.  &&  explore_weights(i) < min_weight)
      min_weight = explore_weights(i);
//       cout<<*possibleGroundActions(i)<<"  "<<explore_weights(i)<<endl;
  }

  if (TL::areEqual(min_weight, 10e8)) {
    // May happen if unknown actions without effect and thus get tabooed
    // (and hence state cannot be come known)
    // and correct action among fixedActions which are also tabooed.
    PRINT(possibleGroundActions);
    PRINT(actions__confidences);
    PRINT(tabooed__direct_explore__actions);
    PRINT(fixedActions);
    cout<<"setting min_weight didn't work:  min_weight="<<min_weight<<endl;
    MT_MSG("setting min_weight didn't work:  min_weight="<<min_weight);
    explore_weights.setUni(min_weight);
  }
  FOR1D(possibleGroundActions, i) {
    if (TL::areEqual(explore_weights(i), min_weight))
      explore_weights(i) = 1.0;
    else
      explore_weights(i) = 0.;
  }
  
  if (DEBUG>0) {
    cout<<"ACTION:  isKnown   rule   #Ex   rule-conf";
    cout <<"   -->  explore-w  : ";
    cout<<"     [all-rules]";
    cout<<endl;
    uint i;
    FOR1D(possibleGroundActions, i) {
      MT::String name;   possibleGroundActions(i)->name(name);  printf("%-12s",(char*)name);
      if (action__is_known(i)) printf("  +");
      else printf("   ");
      printf("  %2i", actions__covering_rules(i));
      printf("  %2i", getNumberOfCoveredExperiences(actions__covering_rules(i)));
      printf("   %5.2f", actions__confidences(i));
      printf("    -->   %6.2f  ", explore_weights(i));
      if (taboo_action == possibleGroundActions(i)  ||  tabooed__direct_explore__actions.findValue(possibleGroundActions(i)) >= 0)
        cout<< "  T";
      else
        cout<< "   ";
      if (actions__covering_rules__all.N > i) {cout<<"    "<<actions__covering_rules__all(i);}
      cout<<endl;
    }
  }
  
  if (TL::isZero(sum(explore_weights))) HALT("no non-zero exploration weight");
  
  if (DEBUG>0) {
    cout<<"calcExploreWeights [END]"<<endl;
  }
}






// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
//   Propagate confidence calculations


void TL::RuleExplorer::updateActionAndStateMeasures() {
  uint DEBUG = 2;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"updateActionAndStateMeasures [START]"<<endl;}
  uint i, k;
  
  //-------------------------------------
  // Which actions have interesting arguments?
  action__has_interesting_argument.resize(possibleGroundActions.N);
  FOR1D(action__has_interesting_argument, i) {
    action__has_interesting_argument(i) = false;
    FOR1D(possibleGroundActions(i)->args, k) {
      if (reward_constants.findValue(possibleGroundActions(i)->args(k)) >= 0
            ||  reward_constants_neighbors.findValue(possibleGroundActions(i)->args(k)) >= 0) {
        action__has_interesting_argument(i) = true;
        break;
      }
    }
  }
  
  //-------------------------------------
  // By which rulesC are actions covered?
  const TL::RuleSet& rules = getRules();
  if (representation == relational)
    TL::ruleReasoning::coveringRules(actions__covering_rules, rules, possibleGroundActions, current_state);
  else
    TL::ruleReasoning::coveringGroundedRules(actions__covering_rules, rules, possibleGroundActions, current_state);

  //-------------------------------------
  // Action confidences
  actions__confidences.resize(possibleGroundActions.N);
  FOR1D(possibleGroundActions, i) {
//     if (representation == flat) {
//       TL::FlatExplorer* flat = (TL::FlatExplorer*) this;
//       // every (action x state)-combo have their individual rule
//       int idx = flat->action_state_to_flat_id(possibleGroundActions(i), &current_state);
// //       int idx = flat->action_state_to_flat_id(modeledActions(actions__covering_rules(i)), &current_state);
//       actions__confidences(i) = rules__confidences(idx);
//     }
//     else
      actions__confidences(i) = rules__confidences(actions__covering_rules(i));
  }

  //-------------------------------------
  // Actions are known?
  action__is_known.resize(possibleGroundActions.N);
  FOR1D(possibleGroundActions, i) {
    action__is_known(i) = actionIsKnown(current_state, possibleGroundActions(i));
  }
  
  //-------------------------------------
  // State is known?
  current_state_is_known = stateIsKnown(current_state, possibleGroundActions);
  current_state_is_known_partially = stateIsKnown_partially(current_state, possibleGroundActions);
  
  
  //-------------------------------------
  // Action dissimilarity to previous experiences - Local
  // TODO --> see AbstractRuleExplorer
  
  if (DEBUG>0) {
    cout<<"ACTION MEASURES:  rule   |   conf   |   dissim-local"<<endl;
    FOR1D(possibleGroundActions, i) {
      MT::String name;
      possibleGroundActions(i)->name(name);
      printf("%-12s",(char*)name);
      printf("  %2i", actions__covering_rules(i));
      printf("   %6.2f", actions__confidences(i));
      if (actions__dissimilarity_to_previous_experiences__local.N > i) {
        printf("  %6.2f", actions__dissimilarity_to_previous_experiences__local(i));
      }
cout<<endl;
    }
    
    PRINT(current_state_is_known);
    PRINT(current_state_is_known_partially);
  }
  
  if (DEBUG>0) {cout<<"updateActionAndStateMeasures [END]"<<endl;}
}







// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
//   Access state informaction


void RuleExplorer::getKnownStates(StateL& known_states) {
  known_states.clear();
  uint i;
  FOR1D(visited_pre_states, i) {
    if (stateIsKnown(*visited_pre_states(i), possibleGroundActions))
      known_states.append(visited_pre_states(i));
  }
}

bool RuleExplorer::stateIsKnown(const TL::State& state, const AtomL& ground_actions) {
  uint i;
  FOR1D(ground_actions, i) {
    if (!actionIsKnown(state, ground_actions(i)))
      return false;
  }
  return true;
}

bool RuleExplorer::stateIsKnown_partially(const TL::State& state, const AtomL& ground_actions) {
  uint i;
  FOR1D(ground_actions, i) {
    if (action__has_interesting_argument(i) &&  !actionIsKnown(state, ground_actions(i)))
      return false;
  }
  return true;
}

bool RuleExplorer::actionIsKnown(const TL::State& state, TL::Atom* action) {
  int id = possibleGroundActions.findValue(action);
  CHECK(id >= 0  &&  id < (int) possibleGroundActions.N, "");
  if (fixedActions.findValue(action) >= 0)
    return true;
  else if (actions__confidences(id) < RULE_CONFIDENCE_THRESHOLD)
    return false;
  else
    return true;
}





// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
//   Decision-making

TL::Atom* RuleExplorer::getExploitAction(AtomL& plan, TL::PRADA* prada, const TL::State& state) {
  uint DEBUG = 0;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"getPlannedExploitAction [START]"<<endl;}
  double value;
  prada->generatePlan(plan, value, state, 1);
//   cout<<"PRADA's used actions:   ";  writeNice(prada->ground_actions);  cout<<endl;
  if (DEBUG>0) {cout<<"Exploit-Plan: value="<<value<<"  "<<plan<<endl;}
  if (DEBUG>0) {cout<<"getPlannedExploitAction [END]"<<endl;}
  if (plan.N>0  &&  plan(0) != TL::logicObjectManager::getAtom_doNothing())  // TODO, hmm, maybe doNothing-action should lead to a HALT(.)?
    return plan(0);
  else
    return NULL;
}




uint getIndex(const uintA& constants, const uintA& args) {
  uint args_idx=0;
  uint i;
  FOR1D(args, i) {
    args_idx += ((uint) pow(constants.N, i)) * constants.findValue(args(i));
  }
//   cout<<"getIndex: constants="<<constants<<"  args="<<args<<"    args_idx="<<args_idx<<endl;
  return args_idx;
}



// ------------------------------------
//  Reward for Unknown Contexts
// ------------------------------------

double evaluate_prada_reward__unknown_contexts(const AtomL& ground_actions, const NID_DBN& net, uint t, uint depth, uint verbosity = 0) {
  uint DEBUG = 0;
  if (DEBUG<verbosity)
    DEBUG = verbosity;
  if (t>depth)
    return 0.0;
  arr action_coverage(ground_actions.N);
  uint a, r;
  FOR1D(ground_actions, a) {
    action_coverage(a) = 0.0;
    uint id_pred = TL::logicObjectManager::p_actions.findValue(ground_actions(a)->pred);
    uint id_args = getIndex(TL::logicObjectManager::constants, ground_actions(a)->args);
    for (r=0; r<net.action2rules_no(id_pred, id_args); r++) {
      //         arr rvs_rules_simple;   // P(\phi_r | s)
//         arr rvs_rules; // P(\phi_r | -\phi_r', s)     2 dim: (1) timesteps,  (2) rules
//           action_coverage(a) += net.rvs_rules_simple(t, net.action2rules(id_pred, id_args, r));
      action_coverage(a) += net.rvs_rules(t, net.action2rules(id_pred, id_args, r));
    }
    if (DEBUG>0) {if (action_coverage(a)<0.99) {printf("%3.2f", action_coverage(a)); cout<<" "<<*ground_actions(a)<<"  ";}}
  }
  if (DEBUG>0) {cout<<"t="<<t<<":  ";}
  double reward = (1.0 - action_coverage.min());
  if (DEBUG>0) {cout<<endl;  PRINT(reward);}

  return reward;
}



class PRADA_Reward__Unknown_Context : public PRADA_Reward {
  
  protected:
    AtomL ground_actions;
    uint depth;
    
  public :
    
    PRADA_Reward__Unknown_Context(AtomL& _ground_actions, uint _depth) {
      ground_actions = _ground_actions;
      depth = _depth;
    }
    
    // Random variables --> Reals
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      return evaluate_prada_reward__unknown_contexts(ground_actions, net, t, depth, 0);
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t, uint verbosity) {
      return evaluate_prada_reward__unknown_contexts(ground_actions, net, t, depth, verbosity);
    }
};



// ------------------------------------
//  Reward for specific R-max instance
// ------------------------------------

class PRADA_Reward__Rmax : public PRADA_Reward__Unknown_Context {
  
  PRADA_Reward* exploit_PRADA_reward;
  
  double evaluate_prada_reward__rmax(const NID_DBN& net, uint t, uint verbosity = 0) {
    uint DEBUG = 0;
    double reward_value_exploit = exploit_PRADA_reward->evaluate_prada_reward(net, t);
    double reward_value_explore = evaluate_prada_reward__unknown_contexts(ground_actions, net, t, depth, verbosity);
    double reward_value = reward_value_exploit + reward_value_explore;
    if (DEBUG>0) {PRINT(reward_value_exploit);  PRINT(reward_value_explore);  PRINT(reward_value);}
    return reward_value;
  }
  
  public :
    
    PRADA_Reward__Rmax(TL::Reward* exploit_reward, AtomL& _ground_actions, uint _depth)
            : PRADA_Reward__Unknown_Context(_ground_actions, _depth) {
      exploit_PRADA_reward = TL::PRADA::create_PRADA_Reward(exploit_reward);
    }
    
    // Random variables --> Reals
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      return evaluate_prada_reward__rmax(net, t, 0);
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t, uint verbosity) {
      return evaluate_prada_reward__rmax(net, t, verbosity);
    }
};





TL::Atom* RuleExplorer::getExploreAction_planned__contexts(AtomL& explore_plan, TL::PRADA* prada, const TL::State& state) {
  uint DEBUG = 1;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"getExploreAction_planned__contexts [START]"<<endl;}
  MaximizeFunctionReward fake_reward;
  PRADA_Reward__Unknown_Context* planned_explore_reward = new PRADA_Reward__Unknown_Context(possibleGroundActions, PLANNED_EXPLORE__HORIZON);
  prada->setReward(&fake_reward, planned_explore_reward);
  prada->setHorizon(PLANNED_EXPLORE__HORIZON+1);
  prada->setNumberOfSamples(PLANNED_EXPLORE__NUMBER_PRADA_SAMPLES);
  prada->setThresholdReward(0.05);
  
  double value;
  prada->generatePlan(explore_plan, value, state, 1);
  
  TL::Atom* action = NULL;
  if (explore_plan.N > 0  && value > PLANNED_EXPLORE__THRESHOLD) {
    action = explore_plan(0);
    explore_plan.memMove = true;
    explore_plan.remove(explore_plan.N-1);
  }
  
  if (DEBUG>0) {
    PRINT(value);
    cout<<"Explore plan:   "<<explore_plan<<endl;
    PRINT((value>PLANNED_EXPLORE__THRESHOLD));
    cout<<"planned explore action:  ";
    if (action != NULL) {
      cout<<*action<<endl;
      // HINSCHREIBEN, was sich planned explore erhofft!
      AtomL explore_plan__extended;
      explore_plan__extended = explore_plan;
//       explore_plan__extended.append(); // TODO mit irgendwas auffuellen; doNothing geht nicht immer
      prada->infer(explore_plan__extended);
      prada->inferStateRewards();
      uint t;
      for (t=0; t<=PLANNED_EXPLORE__HORIZON; t++) {
        planned_explore_reward->evaluate_prada_reward(*prada->net, t, 2);
        cout<<"t"<<endl;
      }
    }
    else cout<<"NULL"<<endl;
    

  }
  if (DEBUG>0) {cout<<"getExploreAction_planned__contexts [END]"<<endl;}
  return action;
}


TL::Atom* RuleExplorer::getExploreAction_planned(AtomL& explore_plan, TL::PRADA* prada, const TL::State& state, uint type) {
  if (type == 1) {
    return getExploreAction_planned__contexts(explore_plan, prada, state);
  }
  else
    NIY;
}



TL::Atom* RuleExplorer::getExploreAction_direct(TL::PRADA* prada, const TL::State& state, TL::Atom* taboo_action) {
  uint DEBUG = 1;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"DIRECT EXPLORATION."<<endl;}
  arr explore_weights;
  calcExploreWeights(explore_weights, taboo_action);
  TL::Atom* action = possibleGroundActions(TL::basic_sample(explore_weights));
  if (DEBUG>0) {cout<<"Direct-Explore action: "<<*action<<endl;}
  return action;
}








// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
//  Methods for EXTERNAL use

TL::Atom* RuleExplorer::decideAction(const TL::State& state, NID_Planner* planner, uint behavior_type, bool use_known_state_partial) {
  uint DEBUG = 2;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"RuleExplorer::decideAction [START]"<<endl;}
  uint i, k;
  double t_start, t_finish;
  
  current_state = state;
  if (confident_ground_rules.num() == 0)  // This is required if world has changed. Then ground rules need to be recalculated.
    updateRules();
  
  message.clr();
//   message << "[" << visited_actions.N << "]  ";
  
  // (1) reward_constants
  planner->getReward()->getRewardObjects(reward_constants, &state);
  reward_constants_neighbors.clear();
  FOR1D(reward_constants, i) {
    uintA local_neighbors;
    logicReasoning::getGeneralRelatedObjects(local_neighbors, reward_constants(i), state);
    FOR1D(local_neighbors, k) {
      if (reward_constants.findValue(local_neighbors(k)) < 0)
        reward_constants_neighbors.setAppend(local_neighbors(k));
    }
  }
  if (DEBUG>0) {PRINT(reward_constants);  PRINT(reward_constants_neighbors);}

  
  
  // (2) Statistics
  updateActionAndStateMeasures();
//   if (DEBUG>0) {
//     cout<<"Action:   known?    rule   confidence"<<endl;
//     FOR1D(possibleGroundActions, i) {
//       MT::String name;
//       possibleGroundActions(i)->name(name);
//       printf("%-12s",(char*)name);
//       if (action__is_known(i)) printf("  +");
//       else printf("   ");
//       printf("  %2i", actions__covering_rules(i));
//       printf("   %6.2f", actions__confidences(i));
//       cout<<endl;
//     }
//   }
  
  // (3) Decide action
  t_start = MT::cpuTime();
  TL::Atom* action = NULL;
  // only use confident rules
  planner->setGroundRules(confident_ground_rules);

  
  // ----------------------------
  //    Epsilon-Greedy
  // ----------------------------
  if (behavior_type == epsilon_greedy) {
    if (DEBUG>0) {cout<<"Epsilon-Greedy"<<endl;}
    double zufall = rnd.uni();
    // f(x) = 2 * (1. - 1 / (1 + exp(-0.02 * x)))
    double greedy_epsilon_dynamic = greedy_epsilon_start * 2. * (1. - (1. / (1. + exp(-0.02 * all_experiences.N))));
    if (DEBUG>0) {PRINT(zufall);  PRINT(1. - greedy_epsilon_dynamic);  PRINT(all_experiences.N);}
    if (zufall < 1. - greedy_epsilon_dynamic) {
      if (DEBUG>0) {cout<<"TRYING TO EXPLOIT."<<endl;   cerr<<"TRYING TO EXPLOIT."<<endl;}
      // (1) TRY EXPLOIT
      AtomL exploit_plan;
      action = getExploitAction(exploit_plan, (TL::PRADA*) planner, state);
      if (action != NULL) {
        last_exploit_plan = exploit_plan;
        moves_exploit.append(visited_pre_states.N);
        moves.append(MOVE_TYPE__EXPLOIT);
        message << "Exploit";
        if (DEBUG>0) {
          cout << "Exploit plan!"<<endl;
          cerr << "--> Exploit plan:  "<<exploit_plan<<endl;
        }
      }
    }
    // (2) EXPLORE VIA RANDOM ACTION
    if (action == NULL) {
      if (DEBUG>0) {cout<<"RANDOM DIRECT EXPLORE."<<endl;   cerr<<"RANDOM DIRECT EXPLORE."<<endl;}
      while (action == NULL) {
        action = possibleGroundActions(rnd.num(possibleGroundActions.N));
        if (fixedActions.findValue(action) >= 0) { action = NULL;}
      }
      moves_explore_direct.append(visited_pre_states.N);
      moves.append(MOVE_TYPE__EXPLORE_DIRECT);
      message << "Random Direct explore";
    }
  }
  // ----------------------------
  //    E^3
  // ----------------------------
  else if (behavior_type == ecube) {
    if (DEBUG>0) {cout<<"E^3-Exploration"<<endl;}
    TL::Atom* bad_action = NULL;
    //   (E^3)  (1) STATE KNOWN --> PLAN
    if ((use_known_state_partial && current_state_is_known_partially)
              || (!use_known_state_partial && current_state_is_known)) {
      if (DEBUG>0) {cout<<"*** STATE IS KNOWN --> PLAN. ***"<<endl;  cerr<<"*** STATE IS KNOWN --> PLAN. ***"<<endl;}
      //   (E^3)  (1.1)  TRY TO EXPLOIT
      if (DEBUG>0) {cout<<"*** EXPLOIT TRY. ***"<<endl;   cerr<<"*** EXPLOIT TRY. ***"<<endl;}
      // (1) TRY EXPLOIT
      AtomL exploit_plan;
      bool IPPC__dont_try_planning = false;
      if (USING_IPPC_DOMAIN()) {
        // Special heuristic for IPPC domains [START]
        MT_MSG("special heuristic for EXPLOITING");
        if (DEBUG>0) {cout<<"special heuristic for EXPLOITING"<<endl;}
        // check whether we have tried to exploit in this state already while the state has not changed since
        uint q;
        FOR1D_DOWN(visited_pre_states, q) {
          if (visited_pre_states.N != moves.N) HALT("visited_pre_states.N="<<visited_pre_states.N<<"  vs.  moves.N="<<moves.N);
          if (*visited_pre_states(q) != current_state)
            break;
        }
        cout<<"State was the same onwards from state q+1 = "<<(q+1)<<"  (now is "<<visited_pre_states.N<<")"<<endl;
        if (visited_pre_states.N - (q+1) >= 1 &&  !is_major_experience.last()) {
          MT_MSG("HOPELESS EXPLOITING: (i) current state equals previous states and (ii) no major insight from last action");
          cout<<"HOPELESS EXPLOITING: (i) current state equals previous states and (ii) no major insight from last action"<<endl;
          IPPC__dont_try_planning = true;
          action = NULL;
        }
        else {
          action = getExploitAction(exploit_plan, (TL::PRADA*) planner, state);
        }
      }
      // Special heuristic [END]
      else
        action = getExploitAction(exploit_plan, (TL::PRADA*) planner, state);
      if (action != NULL) {
        if (DEBUG>0) {cout<<"Candidate exploit plan:  "<<exploit_plan<<endl;}
        last_exploit_plan = exploit_plan;
        // Check whether this action is part of a pattern [START]
        AtomL last_actions;
        last_actions.append(action);
        for (i=visited_actions.N; i>0 && last_actions.N < 10; i--) {
          last_actions.append(visited_actions(i-1));
        }
        uint found_max_pattern_length = logicReasoning::findPattern(last_actions, MIN_PATTERN_ACTION_REPEATS);
        if (DEBUG>0) {cout<<"  -->  Pattern of length "<<found_max_pattern_length<<" has been seen at least "<<MIN_PATTERN_ACTION_REPEATS<<" times in last_actions="<<last_actions<<endl;}
        // Check whether this action is part of a pattern [END]
        if (found_max_pattern_length >= BAD_PATTERN_MIN_LENGTH) {
          bad_action = action;
          action = NULL;
          if (DEBUG>0) {
            cout << "--> CAN'T USE THIS EXPLOIT PLAN!"<<endl;
            cerr << "--> CAN'T USE THIS EXPLOIT PLAN!"<<endl;
          }
        }
        else {
          moves_exploit.append(visited_pre_states.N);
          moves.append(MOVE_TYPE__EXPLOIT);
          message << "Exploit";
          if (DEBUG>0) {
            cout << "Exploit plan!"<<endl;
            cerr << "--> Exploit plan:  "<<exploit_plan<<endl;
          }
        }
      }
      //   (E^3)  (1.2)  PLANNED EXPLORE
      // No good plan has been found: plan in modified DBN.
      else if (!IPPC__dont_try_planning) {
        if (DEBUG>0) {cout<<"*** PLANNED EXPLORE TRY. ***"<<endl;   cerr<<"*** PLANNED EXPLORE TRY. ***"<<endl;}
          AtomL explore_plan;
          action = getExploreAction_planned(explore_plan, (TL::PRADA*) planner, state);
        if (action != NULL) {
          if (DEBUG>0) {cout<<"Candidate planned explore plan:  "<<explore_plan<<endl;}
          // Check whether this action is part of a pattern [START]
          AtomL last_actions;
          last_actions.append(action);
          for (i=visited_actions.N; i>0 && last_actions.N < 10; i--) {
            last_actions.append(visited_actions(i-1));
          }
          uint found_max_pattern_length = TL::logicReasoning::findPattern(last_actions, MIN_PATTERN_ACTION_REPEATS);
          if (DEBUG>0) {cout<<"  -->  Pattern of length "<<found_max_pattern_length<<" has been seen at least "<<MIN_PATTERN_ACTION_REPEATS<<" times."<<endl;}
          // Check whether this action is part of a pattern [END]
          if (found_max_pattern_length >= BAD_PATTERN_MIN_LENGTH) {
            bad_action = action;
            action = NULL;
            if (DEBUG>0) {
              cout << "--> CAN'T USE THIS PLANNED EXPLORE PLAN!"<<endl;
              cerr << "--> CAN'T USE THIS PLANNED EXPLORE PLAN!"<<endl;
            }
          }
          else {
            moves_explore_planned.append(visited_pre_states.N);
            moves.append(MOVE_TYPE__EXPLORE_PLANNED);
            message << "Planned explore";
            if (DEBUG>0) {
              cout << "Planned explore plan!"<<endl;
              cerr << "--> Planned explore plan:  "<<explore_plan<<endl;
            }
          }
        }
      }
    }
    //   (E^3)  (2) STATE UNKNOWN OR NO PLANNING SUCCESSFUL  -->   TRY OUT LEAST KNOWN ACTION
    if (action == NULL) {
      if (DEBUG>0) {
        if((use_known_state_partial && !current_state_is_known_partially)
              || (!use_known_state_partial && !current_state_is_known)) {
          cout<<"*** STATE IS UNKNOWN *** "<<endl;  cerr<<"*** STATE IS UNKNOWN *** "<<endl;}
      }
      if (DEBUG>0) {
        AtomL unknown_actions;
        FOR1D(possibleGroundActions, i) {
          if (!action__is_known(i))
            unknown_actions.append(possibleGroundActions(i));
        }
        if (use_known_state_partial) {cout<<"Relevant ";  cerr<<"Relevant ";}
        cout<<"unknown_actions:  "<<unknown_actions<<endl;
        cerr<<"unknown_actions:  "<<unknown_actions<<endl;
        cout << "*** DIRECT EXPLORE ACTION. ***"<<endl;   cerr<<"*** DIRECT EXPLORE ACTION. ***"<<endl;
      }
      // Never use clever explore weights: --> performed with smallest confidence
      action = getExploreAction_direct((TL::PRADA*) planner, state, bad_action);
      moves_explore_direct.append(visited_pre_states.N);
      moves.append(MOVE_TYPE__EXPLORE_DIRECT);
    }
  }
  // ----------------------------
  //    R-MAX
  // ----------------------------
  else if (behavior_type == rmax) {
    if (DEBUG>0) {cout<<"R-max"<<endl;}
    
    TL::Reward* exploit_reward = planner->getReward();
    
    // build r-max reward
    TL::PRADA_Reward__Rmax* rmax_reward = new TL::PRADA_Reward__Rmax(exploit_reward, possibleGroundActions, PLANNED_EXPLORE__HORIZON);
    
    MaximizeFunctionReward fake_reward;
    ((TL::PRADA*) planner)->setReward(exploit_reward, rmax_reward);
    
    // start "get_exploit_action"
    AtomL plan;
    action = getExploitAction(plan, (TL::PRADA*) planner, state);
    
    if (action != NULL) {
      if (DEBUG>0) {PRINT(*action);}
      if (DEBUG>0) {cout<<"Exploit plan:  "<<plan<<endl;}
      moves.append(MOVE_TYPE__EXPLOIT);
    }
    else {
      // Starting situation without manipulating rules!
      cout<<"No action found; do simple direct explore step  (#rules="<<getRules().num()<<")."<<endl;
      action = getExploreAction_direct((TL::PRADA*) planner, state, NULL);
      moves.append(MOVE_TYPE__EXPLORE_DIRECT);
    }
    
    
    // TODO Pattern-heuristic of E^3: several times the same action --> bad action
  }
  else
    NIY;
  t_finish = MT::cpuTime();
  if (DEBUG>0) {
    cout<<"Action decision took " << (t_finish - t_start) << "s"<<endl;
    cerr<<"Action decision took " << (t_finish - t_start) << "s"<<endl;
  }
  
//   message << "    " << *action;
  
  if (DEBUG>0) {cout<<"***** Decided Action:  "<<*action<<"   *****"<<endl;}
  if (DEBUG>0) {cout<<"RuleExplorer::decideAction [END]"<<endl;}
  
  if (possibleGroundActions.findValue(action) < 0) {
    cerr<<"Unknown action decided for:  "<<*action<<endl;
  }
  
  return action;
}


void RuleExplorer::addObservation__helper(TL::State* state_pre, TL::Atom* action, TL::State* state_post) {
  visited_pre_states.append(state_pre);
  visited_actions.append(action);
  SymbolicExperience* exp = new SymbolicExperience(*state_pre, action, *state_post);
  all_experiences.append(exp);
  is_major_experience.append(false);
  experience_weights.append(1.0);
  experiences_per_modeledAction(action_to_learner_id(action)).append(visited_pre_states.N-1);
  learners_uptodate(action_to_learner_id(action)) = false;
}

void RuleExplorer::addObservations(const TL::Trial& trial) {
  uint i;
  for (i=0; i<trial.states.N-1; i++) {
    addObservation(trial.states(i), trial.actions(i), trial.states(i+1));
  }
}

void RuleExplorer::updateLogicEngineConstants() {
  // Set up all possible actions
  AtomL potential_ground_actions;
  logicObjectManager::getAtoms_actions(potential_ground_actions, logicObjectManager::constants);
  
  // filter
  uint i;
  possibleGroundActions.clear();
  FOR1D(potential_ground_actions, i) {
    if (potential_ground_actions(i)->args.N == 2  &&  potential_ground_actions(i)->args(0) == potential_ground_actions(i)->args(1))
      continue;
    if (potential_ground_actions(i)->pred->id != TL::DEFAULT_ACTION_PRED__ID) {
      possibleGroundActions.append(potential_ground_actions(i));
    }
  }
  
  confident_ground_rules.clear();
}

  
void RuleExplorer::reset() {
}








  
  
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================


AbstractRuleExplorer::AbstractRuleExplorer(double complexity_penalty_coeff,
                                           double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                                           TL::RuleSet& fixed_rules_for_fixed_actions, uint _density_estimation_type)
                    : RuleExplorer(relational, complexity_penalty_coeff, p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule) {
  uint DEBUG = 1;
  density_estimation_type = _density_estimation_type;
  init_rules();
  
  modeledActions.clear();
  uint i;
  FOR1D(logicObjectManager::p_actions, i) {
    if (logicObjectManager::p_actions(i)->id == TL::DEFAULT_ACTION_PRED__ID)
      continue;
    if (logicObjectManager::p_actions(i)->d == 0) {
      uintA empty;
      modeledActions.append(logicObjectManager::getAtom(logicObjectManager::p_actions(i), empty));
    }
    else if (logicObjectManager::p_actions(i)->d == 1) {
      uintA args = TUP((uint) 0);
      modeledActions.append(logicObjectManager::getAtom(logicObjectManager::p_actions(i), args));
    }
    else if (logicObjectManager::p_actions(i)->d == 2) {
      uintA args = TUP((uint) 0, (uint) 1);
      modeledActions.append(logicObjectManager::getAtom(logicObjectManager::p_actions(i), args));
    }
    else {NIY;}
  }
  
  FOR1D(modeledActions, i) {
    // Set up rule-learner for each abstract action
    double alpha_PEN = complexity_penalty_coeff;
    double p_min = p_lower_bound__noise_outcome;
    double p_min_noisyDefaultRule = p_lower_bound__noise_outcome_in_default_rule;
    uint so_choice_type = 2;
    learners.append(new RuleLearner(alpha_PEN, p_min, p_min_noisyDefaultRule, so_choice_type));
  }
  if (DEBUG>0) {cout<<"modeledActions:  "<<modeledActions<<endl;}
  
  experiences_per_modeledAction.resize(modeledActions.N);
  learners_uptodate.resize(modeledActions.N);
  learners_uptodate.setUni(true);
  
  updateLogicEngineConstants();
  updateRules();
  
  
  // fixed_rules_for_fixed_actions
  fixedActions.clear();
  FOR1D_(fixed_rules_for_fixed_actions, i) {
    fixedActions.setAppend(fixed_rules_for_fixed_actions.elem(i)->action);
  }
  if (DEBUG>0) {cout<<"fixedActions:  "<<fixedActions<<endl;}
  uint k, l;
  FOR1D(fixedActions, k) {
    FOR1D_(fixed_rules_for_fixed_actions, i) {
      if (fixed_rules_for_fixed_actions.elem(i)->action == fixedActions(k)) {
        uintA empty1;
        MT::Array< uintA > empty2; // HACK spaeter removen
        FOR1D(fixed_rules_for_fixed_actions.elem(i)->outcomes, l) {
          empty2.append(empty1);
        }
        rulesC.append(fixed_rules_for_fixed_actions.elem(i), empty1, empty2);
      }
    }
  }
  if (DEBUG>0) {
    cout<<"fixed_rules_for_fixed_actions:"<<endl;
    fixed_rules_for_fixed_actions.write();
//     cout<<"Starting RuleSetContainer:"<<endl;
//     rulesC.writeNice();
  }
}


AbstractRuleExplorer::~AbstractRuleExplorer() {
  uint i;
  FOR1D(learners, i) {
    delete learners(i);
  }
}

void AbstractRuleExplorer::init_rules() {
  // default rule
  rulesC.init(&all_experiences);
  rulesC.rules.append(TL::ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();
  updateRules();
}


uint AbstractRuleExplorer::action_to_learner_id(TL::Atom* action) {
  uint i;
  FOR1D(modeledActions, i) {
    TL::Substitution sub;
    if (logicReasoning::unify(action, modeledActions(i), sub)) {
      return i;
    }
  }
  HALT("Did not find learner for action "<<*action);
}


void rule_write_hack(TL::Rule* rule, MT::Array< uintA >& outcome_tripletts, bool with_action, ostream& os);

void AbstractRuleExplorer::updateRules(bool always_re_learning) {
  uint DEBUG = 1;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"updateRules [START]"<<endl;}
    
  uint i, k, l;
  
  // (1) LEARN RULE-SET
  //     --> Take only rule-sets with new examples into account.
  FOR1D(modeledActions, i) {
    if (learners_uptodate(i)) {
    }
    else {
      if (DEBUG>0) {
        cout<<endl<<"Relearning for action "<<*modeledActions(i)<<"   (i="<<i<<")"<<endl;
        if (DEBUG>2) {
          cout<<"BEFORE rule-set:"<<endl;
          rulesC.writeNice();
        }
      }
      
      if (fixedActions.findValue(modeledActions(i)) >= 0)
        continue;
      
      // always_re_learning=false  means:  Potentially re-learning NOT done..
      // NO re-learning if experience can straight-forwardly be added to an existing rule.
      if (!always_re_learning) {
        if (DEBUG>1) {cout<<"Check whether we can renounce on re-learning!"<<endl;}
        bool relearning_necessary = false;
        FOR1D(experiences_per_modeledAction(i), k) {
          uint experience_id = experiences_per_modeledAction(i)(k);
          bool used_for_rule_learning = false;
          // ACHTUNG: TODO zurzeit testen wir nur fuer die letzte aktion
          // retrieve latest experiences:
          //    ->  check whether experience has been used to learn one of the rules
          FOR1D(rulesC.experiences_per_rule, l) {
            if (rulesC.experiences_per_rule(l).findValue(experience_id) >= 0) {
              used_for_rule_learning = true;
              break;
            }
          }
          if (used_for_rule_learning)
            continue;
          // if experience has not been found, then this experience has not been used to learn a rule
          if (DEBUG>1) {cout<<"The following experience for action "<<*modeledActions(i)<<" has not been used:"<<endl;  PRINT(experience_id);
                        all_experiences(experience_id)->write();}
          if (experience_id != all_experiences.N-1)  HALT("not last experience -- aber nur dafuer derzeit programmiert");
          TL::State& s_pre = all_experiences(experience_id)->pre;
          TL::Atom* action = all_experiences(experience_id)->action;
          uint rule_idx = TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(rulesC.rules, s_pre, action);
          if (DEBUG>1) {cout<<"Modeling rule:"<<endl;  rulesC.rules.elem(rule_idx)->write();}
          if (rule_idx == 0) {
            if (DEBUG>0) {cout<<"Relearning required: no existing unique covering rule."<<endl;}
            relearning_necessary = true;
            break;
          }
          // Check whether new experience is similar enough to old experiences
          // collect other graphs
          MT::Array< RelationalStateGraph* > other_graphs;
          AtomL other_actions;
          uint q;
          FOR1D(rulesC.experiences_per_rule(rule_idx), q) {
            other_actions.append(visited_actions(rulesC.experiences_per_rule(rule_idx)(q)));
            other_graphs.append(graphs(rulesC.experiences_per_rule(rule_idx)(q)));
          }
          CHECK(s_pre == graphs.last()->state, "");
          TL::RelationalStateGraph* graph = RelationalStateGraph::createSubgraph(*action, *graphs.last(), *rulesC.rules.elem(rule_idx), ENTROPY_RELATED_OBJECTS_DEPTH);
          double min_distance = RelationalStateGraph::getMinDistance(*graph, *action, other_graphs, other_actions);
          if (DEBUG>0) {cout<<"Min distance of new experience to old experiences :  "<<min_distance<<endl;}
          if (!TL::isZero(min_distance) ||  rnd.uni() < 0.05)  {  // allow some randomness for relearning
            if (rulesC.experiences_per_rule(rule_idx).N <= 5) {
              if (DEBUG>0) {cout<<"Relearning required:  min distance > 0  and  #Exp <= 5."<<endl;}
              relearning_necessary = true;
              break;
            }
            else if (rulesC.experiences_per_rule(rule_idx).N < 15) { // 20% possibility to not re-learn
              if (rnd.uni() < 0.8 * min_distance)  { // >=20% possibility to not re-learn
                if (DEBUG>0) {cout<<"Relearning required:  min distance > 0  and  5 < #Exp < 15  and  random decision."<<endl;}
                relearning_necessary = true;
                break;
              }
            }
            else if (rnd.uni() < 0.25 * min_distance) {
              if (DEBUG>0) {cout<<"Relearning required:  min distance > 0  and  15 <= #Exp  and  random decision."<<endl;}
              relearning_necessary = true;
              break;
            }
            else {
              if (DEBUG>0) {cout<<"No-relearning despite of min_distance > 0; due to random decision and  #Exp >15 ."<<endl;}
            }
          }
          // add to existing non-default unique covering rule
          rulesC.nonDefaultRules_per_experience.last().append(rule_idx);
          rulesC.experiences_per_rule(rule_idx).append(experiences_per_modeledAction(i)(k));
          uintA covering_outcomes;
          TL::ruleReasoning::coveringOutcomes(rulesC.rules.elem(rule_idx), s_pre, action, all_experiences(experience_id)->post, covering_outcomes);
          uint o;
          FOR1D(covering_outcomes, o) {
            rulesC.experiences_per_ruleOutcome(rule_idx)(o).append(experiences_per_modeledAction(i)(k));
          }
        }
        if (!relearning_necessary) {
          if (DEBUG>0) {cout << "No re-learning necessary :-) for "<<*modeledActions(i)<<endl;}
          cerr << "No re-learning necessary :-) for "<<*modeledActions(i)<<endl;
          learners_uptodate(i) = true;
          continue;
        }
      }
      if (DEBUG>0) {cout<<"Learning for "<<*modeledActions(i)<<endl;  cerr<<"Learning for "<<*modeledActions(i)<<endl;}
      
      // (i) save old example coverage statistics
      MT::Array< uintA > old_experience_partitions;
      rulesC.getPartitionsForAction(old_experience_partitions, modeledActions(i));
      if (DEBUG>1) {
        cout<<"Old partitions (" << old_experience_partitions.N << "):"<<endl;
        FOR1D(old_experience_partitions, k) {
          uintA rules_ids = rulesC.nonDefaultRules_per_experience(old_experience_partitions(k)(0));
          if (rules_ids.N == 1) {
            TL::write(rulesC.rules.elem(rules_ids(0))->context);
          }
          else
            cout<<"Default rule";
          cout<<endl<<"   "<<old_experience_partitions(k)<<endl;
        }
      }
      // save old rule for latest example
      TL::Rule* old_covering_rule_for_latest_example = NULL;
      uint old_covering_rule_for_latest_example__id = TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(rulesC.rules, *visited_pre_states.last(), visited_actions.last());
      if (old_covering_rule_for_latest_example__id != 0) {
        old_covering_rule_for_latest_example = new TL::Rule;
        old_covering_rule_for_latest_example->copyBody(*rulesC.rules.elem(old_covering_rule_for_latest_example__id));
      }
      
      // (ii) remove old rules for this action
      FOR1D_DOWN_(rulesC.rules, k) {
        if (rulesC.rules.elem(k)->action == modeledActions(i))
          rulesC.remove(k);
      }
      
      if (DEBUG>2) {
        cout<<endl<<"FILTERED rule-set:"<<endl;
        rulesC.writeNice();
      }
      
      // (iii) learn new rules for this action
      TL::RuleSetContainer rulesC_for_action;
      SymbolicExperienceL action_experiences;
      arr action_experience_weights;
      FOR1D(experiences_per_modeledAction(i), k) {
        action_experiences.append(all_experiences(experiences_per_modeledAction(i)(k)));
//         action_experience_weights.append(experience_weights(experiences_per_modeledAction(i)(k)));
        double cutoff_for_min_weight = 1. -0.05 * experiences_per_modeledAction(i).N;
        double min_weight = TL_MAX(0.3, cutoff_for_min_weight);  // good old choice
        double weight = min_weight + (1. - min_weight) * (1.0 * k) / experiences_per_modeledAction(i).N;
//         cout<<k<<" --> "<<weight<<endl;
        action_experience_weights.append(weight);
//         action_experience_weights.append((1.0 * (k + experiences_per_modeledAction(i).N) / (2.0 * experiences_per_modeledAction(i).N)));
      }
      cerr<<"Rule learning uses "<<action_experiences.N<<" experiences."<<endl;
      if (DEBUG>2) {PRINT(experiences_per_modeledAction(i));  cout<<"Starting to learn..."<<endl;}


      if (USING_IPPC_DOMAIN()) {
        learners(i)->setAlphaPEN(10e-6 + 0.01 * action_experiences.N);  // TODO das hier ist fuer die IPC-Experimente
      }
      else {
        // ROBOT MANIPULATION DOMAIN
        // gnuplot> f(x) = 0.3 * exp(0.005 * x) - 0.25
        learners(i)->setAlphaPEN(0.3 * exp(0.005 * action_experiences.N) - 0.25);  // dw-series1

      // gnuplot> h(x) = 0.5 * exp(0.005 * x) - 0.4
//       learners(i)->setAlphaPEN(0.5 * exp(0.005 * action_experiences.N) - 0.4);  // Curriculum neu -- TEST_ENTROPY_1  und auch TEST_ENTROPY_2
      // gnuplot> g(x) = 0.4 * exp(0.005 * x)
//       learners(i)->setAlphaPEN(0.4 * exp(0.005 * action_experiences.N));  // Box-clearance  und  Curriculum alt
      //       learners(i)->setAlphaPEN(0.3 * exp(0.005 * action_experiences.N));  // Stack -- BALLS
//       learners(i)->setAlphaPEN(0.5 * exp(0.005 * action_experiences.N));  // Stack -- BLOCKS; aber mit BOXEN geht's anscheinend auch gut
      }
      
      learners(i)->learn_rules(rulesC_for_action, action_experiences, action_experience_weights);
      if (DEBUG>2) {cout<<"... finished learning."<<endl;}
//       if (DEBUG>2) {cout<<"New rules survive sanity check?"<<endl;}
//       if (DEBUG>0) {rulesC_for_action.sanityCheck();}
//       if (DEBUG>2) {cout<<"Yes, new rules survived sanity check."<<endl;}
      if (DEBUG>2) {
        cout<<endl<<"EXPERIENCES for action:"<<endl;
        PRINT(experiences_per_modeledAction(i));
        if (DEBUG>3) {TL::write(action_experiences);}
        cout<<endl<<"NEW RULES for action:"<<endl;
        rulesC_for_action.writeNice();
      }
      
      // (iv) insert new rules into overall container
      FOR1D_(rulesC_for_action.rules, k) {
        if (TL::ruleReasoning::isDefaultRule(rulesC_for_action.rules.elem(k)))
          continue;
        
        // Interfacing between different experience-ids
        
        uintA experiences_per_rule__converted;
        FOR1D(rulesC_for_action.experiences_per_rule(k), l) {
          experiences_per_rule__converted.append(experiences_per_modeledAction(i)(rulesC_for_action.experiences_per_rule(k)(l)));
        }
        
        MT::Array < uintA > experiences_per_ruleOutcome__converted;
        experiences_per_ruleOutcome__converted.resize(rulesC_for_action.experiences_per_ruleOutcome(k).N);
        uint o;
        FOR1D(experiences_per_ruleOutcome__converted, o) {
          FOR1D(rulesC_for_action.experiences_per_ruleOutcome(k)(o), l) {
            experiences_per_ruleOutcome__converted(o).append(experiences_per_modeledAction(i)(rulesC_for_action.experiences_per_ruleOutcome(k)(o)(l)));
          }
        }
        
        if (DEBUG>2) {
          PRINT(rulesC_for_action.experiences_per_rule(k));
          PRINT(experiences_per_rule__converted);
          PRINT(rulesC_for_action.experiences_per_ruleOutcome(k));
          PRINT(experiences_per_ruleOutcome__converted);
        }
        
        rulesC.append(rulesC_for_action.rules.elem(k), experiences_per_rule__converted, experiences_per_ruleOutcome__converted);
      }
      
//       if (DEBUG>2) {cout<<"AFTER rule-set survives sanity check?"<<endl;}
//       if (DEBUG>0) {rulesC.sanityCheck(true); if (DEBUG>2) {cout<<"AFTER rule-set survived sanity check."<<endl;}}
//       if (DEBUG>2) {cout<<"Yes, AFTER rule-set survived sanity check!"<<endl;}
      if (DEBUG>2) {
        cout<<endl<<"AFTER rule-set:"<<endl;
        rulesC.writeNice();
      }
      
      // (v) investigate whether major experience
      // (v-1) compare new partitions
      MT::Array< uintA > new_experience_partitions;
      rulesC.getPartitionsForAction(new_experience_partitions, modeledActions(i));
      if (DEBUG>1) {
        cout<<"New partitions (" << new_experience_partitions.N << "):"<<endl;
        FOR1D(new_experience_partitions, k) {
          uintA rules_ids = rulesC.nonDefaultRules_per_experience(new_experience_partitions(k)(0));
          if (rules_ids.N == 1) {
            TL::write(rulesC.rules.elem(rules_ids(0))->context);
          }
          else
            cout<<"Default rule";
          FOR1D(old_experience_partitions, l) {
            uintA helfer = new_experience_partitions(k);
            helfer.removeValueSafe(all_experiences.N-1);
            if (helfer == old_experience_partitions(l)) {
              break;
            }
          }
          if (old_experience_partitions.N == l) {
            cout<<"  <-----";
          }
          cout<<endl<<"   "<<new_experience_partitions(k)<<endl;
        }
      }
      bool partitions_changed = false;
      FOR1D(new_experience_partitions, k) {
        new_experience_partitions(k).removeValueSafe(all_experiences.N-1);  // remove latest experience-id
        FOR1D(old_experience_partitions, l) {
          if (new_experience_partitions(k) == old_experience_partitions(l)) {
            break;
          }
        }
        if (old_experience_partitions.N == l) {
          partitions_changed = true;
          break;
        }
      }
      if (DEBUG>1) {PRINT(partitions_changed);}
      // (v-2) covering rule for last-experience changed?
      bool covering_rule_for_last_experience_changed = false;
      if (!partitions_changed) {
        TL::Rule* new_covering_rule_for_latest_example = NULL;
        if (rulesC.nonDefaultRules_per_experience.last().N == 1)
          new_covering_rule_for_latest_example = rulesC.rules.elem(rulesC.nonDefaultRules_per_experience.last()(0));
        if (new_covering_rule_for_latest_example != NULL  &&  old_covering_rule_for_latest_example != NULL) {
          if (new_covering_rule_for_latest_example->context != old_covering_rule_for_latest_example->context) {
            covering_rule_for_last_experience_changed = true;
            if (DEBUG>1) {
              cout<<"Major experience since context has changed"<<endl;
              cout<<"old_covering_rule_for_latest_example->context:  "<<old_covering_rule_for_latest_example->context<<endl;
              cout<<"new_covering_rule_for_latest_example->context:  "<<new_covering_rule_for_latest_example->context<<endl;
            }
          }
        }
        else if (new_covering_rule_for_latest_example != NULL  &&  old_covering_rule_for_latest_example == NULL) {
          covering_rule_for_last_experience_changed = true;
          if (DEBUG>1) {cout<<"Major experience since before we had a null rule."<<endl;}
        }
      }
      
      
      if (partitions_changed  ||  covering_rule_for_last_experience_changed)
        is_major_experience.last() = true;
      
      if (DEBUG>1) {PRINT(is_major_experience.last());}
      
      
      learners_uptodate(i) = true;
    }
  }
  
  cout<<"Did it survive true learning?"<<endl;
  rulesC.sanityCheck(true);
  cout<<"Survived true learning. Now only postprocessing."<<endl;
  rulesC.recomputeDefaultRule();
  rulesC.sanityCheck();
  cout<<"Survived recomputeDefaultRule()"<<endl;
  rulesC.sort();
  rulesC.sanityCheck();
//   cout<<"Survived sort()"<<endl;
  if (DEBUG>0) {cout<<"Rules have been successfully learned."<<endl;}
  
  // (2) CREATE RULE CONFIDENCES
 
  rules__confidences.resize(rulesC.rules.num());
  rule_experiences_entropies.resize(rulesC.rules.num());
  FOR1D_(rulesC.rules, i) {
    rules__confidences(i) = calcRuleConfidence(i);
  }
  
  
  // (3) GROUND THE CONFIDENT RULES (for the planner)
  confident_ground_rules.clear();
  // hack: koennen nicht einfach leeren zustand uebergeben, da funktionswerte trotzdem ausgerechnet werden
  if (current_state.lits_prim.N > 0) {
    TL::RuleSet confident_rules;
    FOR1D_(rulesC.rules, i) {
      if (rules__confidences(i) >= RULE_CONFIDENCE_THRESHOLD) {
        confident_rules.append(rulesC.rules.elem(i));
      }
    }
    TL::ruleReasoning::ground_with_filtering(confident_ground_rules, confident_rules, logicObjectManager::constants, current_state);
  }
  else {
    confident_ground_rules.append(rulesC.rules.elem(0));
  }
  confident_ground_rules.sort_using_args();
  TL::ruleReasoning::removeDoubleLiterals(confident_ground_rules);
  TL::ruleReasoning::checkRules(confident_ground_rules);
  if (DEBUG>1) {
    cout<<"GROUND RULES: (plenty!!)"<<endl;
    cout<<"# = "<<confident_ground_rules.num()<<endl;
//     confident_ground_rules.writeNice();
  }
   
  
  if (DEBUG>0) {
    cout<<"LEARNED RULE-SET (" << visited_pre_states.N << " examples)"<<endl;
    FOR1D_(rulesC.rules, i) {
      if (i==0  ||  (rulesC.rules.elem(i)->action != rulesC.rules.elem(i-1)->action)) {
        cout<<"######  "<<*rulesC.rules.elem(i)->action<<" ######"<<endl;
      }
//       if (rulesC.rules.elem(i)->outcomes.N == 2  &&  rulesC.rules.elem(i)->outcomes(0).N == 0)
//         continue;
      cout<<"-----   #"<<i<<":    conf="<<rules__confidences(i)
          <<"  (" << rulesC.experiences_per_rule(i).N << "/" 
          << visited_pre_states.N << " = " << (1.0 * rulesC.experiences_per_rule(i).N) / visited_pre_states.N << ") " 
            <<   (rules__confidences(i)<RULE_CONFIDENCE_THRESHOLD? "  INCONFIDENT  ": "")
            <<   "[";
      FOR1D(rulesC.experiences_per_rule(i), k) {
        if (k>10) {
          cout<<"...";
          break;
        }
        cout << rulesC.experiences_per_rule(i)(k) << " ";
      }
      cout << "]" << endl;
      rule_write_hack(rulesC.rules.elem(i), rulesC.experiences_per_ruleOutcome(i), false, cout);
    }
    cout<<endl;
  
//     cerr<<"LEARNED RULE-SET (" << visited_pre_states.N << " examples)"<<endl;
//     FOR1D_(rulesC.rules, i) {
//       if (i==0  ||  (rulesC.rules.elem(i)->action != rulesC.rules.elem(i-1)->action)) {
//         cerr<<"######  "; rulesC.rules.elem(i)->action->writeNice(cerr);  cerr<<" ######"<<endl;
//       }
//       if (rulesC.rules.elem(i)->outcomes.N == 2  &&  rulesC.rules.elem(i)->outcomes(0).N == 0)
//         continue;
//       cerr<<"-----  #"<<i<<":    conf="<<rules__confidences(i)
//           <<"  (" << rulesC.experiences_per_rule(i).N << "/" 
//           << visited_pre_states.N << " = " << (1.0 * rulesC.experiences_per_rule(i).N) / visited_pre_states.N << ") " 
//           << rulesC.experiences_per_rule(i) << endl;
//       rule_write_hack(rulesC.rules.elem(i), rulesC.experiences_per_ruleOutcome(i), false, cerr);
//     }
//     cerr<<endl;
  }
  
  if (DEBUG>0) {cout<<"updateRules [END]"<<endl;}
}

  
void AbstractRuleExplorer::addObservation(State* state_pre, Atom* action, State* state_post) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"AbstractRuleExplorer::addObservation [START]"<<endl;}
  if (fixedActions.findValue(action) >= 0) {
    visited_pre_states.append(state_pre);
    visited_actions.append(action);
    SymbolicExperience* exp = new SymbolicExperience(*state_pre, action, *state_post);
    all_experiences.append(exp);
    is_major_experience.append(false);
    experience_weights.append(1.0);
    experiences_per_modeledAction(action_to_learner_id(action)).append(visited_pre_states.N-1);
    // calculate which rule is covering
    uintA coveringRuleIDs;
    TL::ruleReasoning::coveringRules_groundedAction(rulesC.rules, *state_pre, action, coveringRuleIDs);
    coveringRuleIDs.removeValueSafe(0);
    rulesC.nonDefaultRules_per_experience.append(coveringRuleIDs);
    if (DEBUG>0) {PRINT(coveringRuleIDs);}
    // covering outcomes
    uint i, k;
    FOR1D(coveringRuleIDs, i) {
      if (coveringRuleIDs(i) != 0) {
        if (DEBUG>0) {PRINT(coveringRuleIDs(i));}
        rulesC.experiences_per_rule(coveringRuleIDs(i)).append(all_experiences.N-1);
        uintA covering_outcomes;
        TL::ruleReasoning::coveringOutcomes(rulesC.rules.elem(coveringRuleIDs(i)), *state_pre, action, *state_post, covering_outcomes);
        FOR1D(covering_outcomes, k) {
          if (covering_outcomes.N > 1  &&  k == covering_outcomes.N - 1)
            continue;
          rulesC.experiences_per_ruleOutcome(coveringRuleIDs(i))(covering_outcomes(k)).append(all_experiences.N-1);
        }
      }
    }
    if (DEBUG>0) {
      rulesC.sanityCheck();
      rulesC.writeNice();
    }
  }
  else {
    addObservation__helper(state_pre, action, state_post);
    uintA empty;
    rulesC.nonDefaultRules_per_experience.append(empty);
  }
  graphs.append(new RelationalStateGraph(*state_pre));
  if (DEBUG>0) {cout<<"new graph:"<<endl;  graphs.last()->writeNice();}
  if (DEBUG>0) {
      cout<<"AbstractRuleExplorer::addObservation [END]"<<endl;
  }
}


double AbstractRuleExplorer::calcRuleConfidence(uint rule_id) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"calcRuleConfidence [START]"<<endl;}
  if (DEBUG>0) {PRINT(density_estimation_type);  PRINT(rule_id);  getRules().elem(rule_id)->write();}
  if (fixedActions.findValue(getRules().elem(rule_id)->action) >= 0) { // fixed actions
    if (DEBUG>0) {cout<<"confidence = "<<99.<<endl;  cout<<"calcRuleConfidence [END]"<<endl;}
    return 99.;
  }
  if (TL::ruleReasoning::isDefaultRule(getRules().elem(rule_id))) {   // default rule:  never confident
    if (DEBUG>0) {cout<<"confidence = "<<0.<<endl;  cout<<"calcRuleConfidence [END]"<<endl;}
      return 0.;
  }
  if (!density_estimation_type == density_entropy)
    return getNumberOfCoveredExperiences(rule_id);
  uint i;
  MT::Array<RelationalStateGraph*> filtered_graphs;
  AtomL filtered_actions;
  FOR1D(rulesC.experiences_per_rule(rule_id), i) {
    uint experience_id = rulesC.experiences_per_rule(rule_id)(i);
    RelationalStateGraph* graph_i__filtered = RelationalStateGraph::createSubgraph(*visited_actions(experience_id), *graphs(experience_id),
                                        *rulesC.rules.elem(rule_id), ENTROPY_RELATED_OBJECTS_DEPTH);
    filtered_graphs.append(graph_i__filtered);
    filtered_actions.append(visited_actions(experience_id));
  }
  rule_experiences_entropies(rule_id) = TL::RelationalStateGraph::entropy(filtered_actions, filtered_graphs);
  
  uintA rule_involved_objects;
  TL::logicReasoning::getConstants(rulesC.rules.elem(rule_id)->context, rule_involved_objects);
  rule_involved_objects.setAppend(rulesC.rules.elem(rule_id)->action->args);
  double coeff__experience_number = (7. - TL_MIN(rule_involved_objects.N*2, 4)) / 10.;
//   cout<<rule_id<<" -->  "<<rule_involved_objects<< "  " <<coeff__experience_number<<endl;
  
  double rule_confidence =  rule_experiences_entropies(rule_id) + coeff__experience_number * rulesC.experiences_per_rule(rule_id).N;
  // TODO Handgesetzt
  rule_confidence =  rulesC.experiences_per_rule(rule_id).N;
  if (DEBUG>0) {PRINT(rule_confidence);}
  if (DEBUG>0) {cout<<"calcRuleConfidence [END]"<<endl;}
  return rule_confidence;
}




void TL::AbstractRuleExplorer::updateActionAndStateMeasures() {
  uint DEBUG = 0;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"AbstractRuleExplorer::updateActionAndStateMeasures [START]"<<endl;}
  uint i, k;
  
  //-------------------------------------
  // Which actions have interesting arguments?
  action__has_interesting_argument.resize(possibleGroundActions.N);
  FOR1D(action__has_interesting_argument, i) {
    action__has_interesting_argument(i) = false;
    FOR1D(possibleGroundActions(i)->args, k) {
      if (reward_constants.findValue(possibleGroundActions(i)->args(k)) >= 0
            ||  reward_constants_neighbors.findValue(possibleGroundActions(i)->args(k)) >= 0) {
        action__has_interesting_argument(i) = true;
        break;
      }
    }
  }
  
  //-------------------------------------
  // By which rulesC are actions covered?
  const TL::RuleSet& rules = getRules();
  if (representation == relational)
    TL::ruleReasoning::coveringRules(actions__covering_rules, rules, possibleGroundActions, current_state);
  else
    HALT("");
  
  // auxiliary container
  actions__covering_rules__all.clear();
  FOR1D(possibleGroundActions, i) {
    uintA coveringRuleIDs;
    TL::ruleReasoning::coveringRules_groundedAction(rules, current_state, possibleGroundActions(i), coveringRuleIDs);
    actions__covering_rules__all.append(coveringRuleIDs);
  }

  //-------------------------------------
  // Action confidences
  actions__confidences.resize(possibleGroundActions.N);
  FOR1D(possibleGroundActions, i) {
//     if (representation == flat) {
//       TL::FlatExplorer* flat = (TL::FlatExplorer*) this;
//       // every (action x state)-combo have their individual rule
//       int idx = flat->action_state_to_flat_id(possibleGroundActions(i), &current_state);
// //       int idx = flat->action_state_to_flat_id(modeledActions(actions__covering_rules(i)), &current_state);
//       actions__confidences(i) = rules__confidences(idx);
//     }
//     else
      actions__confidences(i) = rules__confidences(actions__covering_rules(i));
//       cout<<*possibleGroundActions(i)<<"  "<<actions__covering_rules(i)<<endl;
//       cout<<"--> Regel:  "<<endl;  rules.elem(actions__covering_rules(i))->writeNice();
  }

  //-------------------------------------
  // Actions are known?
  action__is_known.resize(possibleGroundActions.N);
  FOR1D(possibleGroundActions, i) {
    action__is_known(i) = actionIsKnown(current_state, possibleGroundActions(i));
  }
  
  //-------------------------------------
  // State is known?
  current_state_is_known = stateIsKnown(current_state, possibleGroundActions);
  current_state_is_known_partially = stateIsKnown_partially(current_state, possibleGroundActions);
   
  
  //-------------------------------------
  // Action dissimilarity to previous experiences - Local
  if (density_estimation_type == density_entropy) {
    actions__dissimilarity_to_previous_experiences__local.resize(possibleGroundActions.N);
    actions__dissimilarity_to_previous_experiences__local.setUni(1.0);
    TL::RelationalStateGraph graph__current_state(current_state);
    if (visited_pre_states.N > 0) {
      if (DEBUG>0) {cout<<"Calculating actions__dissimilarity_to_previous_experiences__local"<<endl;}
      FOR1D(possibleGroundActions, k) {
        if (DEBUG>1) {cout<<"--- "<<*possibleGroundActions(k)<<endl;}
        uint rule_id = actions__covering_rules(k);
        if (DEBUG>1) {PRINT(rule_id);  rulesC.rules.elem(rule_id)->write();}
        if (rule_id == 0) {
          actions__dissimilarity_to_previous_experiences__local(k) = 1.0;
          continue;
        }
        // Collect related graphs
        MT::Array< TL::RelationalStateGraph* > other_graphs;
        AtomL other_actions;
        FOR1D(rulesC.experiences_per_rule(rule_id), i) {
          uint experience_id = rulesC.experiences_per_rule(rule_id)(i);
          other_actions.append(visited_actions(experience_id));
          other_graphs.append(RelationalStateGraph::createSubgraph(*visited_actions(experience_id), *graphs(experience_id),
                                                      *rulesC.rules.elem(rule_id), ENTROPY_RELATED_OBJECTS_DEPTH));
        }
        // new graph
        RelationalStateGraph* graph_k__filtered = RelationalStateGraph::createSubgraph(*possibleGroundActions(k), graph__current_state,
                                                      *rulesC.rules.elem(rule_id), ENTROPY_RELATED_OBJECTS_DEPTH);
//         if (possibleGroundActions(k)->pred->name == MT::String("grab")) {
// //         if (possibleGroundActions(k) == le->getPI(MT::String("grab(70)"))) {
//           cout<<endl<<endl<<endl<<endl;
//           cout<<*possibleGroundActions(k)<<endl;
//           cout<<"GRAPH:"<<endl;
//           graph_k__filtered->writeNice();
//         }
        double min_diff = RelationalStateGraph::getMinDistance(*graph_k__filtered, *possibleGroundActions(k), other_graphs, other_actions);
        CHECK(!(rulesC.experiences_per_rule(rule_id).N == 0  &&  !TL::areEqual(min_diff, 1.0)), "");
        if (actions__confidences(k) != 99  &&  (min_diff > 1.01  ||  min_diff < -0.01))
          HALT("min_diff="<<min_diff<<" is strange");
        actions__dissimilarity_to_previous_experiences__local(k) = min_diff;
        if (DEBUG>1) {cout<<*possibleGroundActions(k)<<":  min_diff="<<min_diff<<endl;}
      }
    }
  }
  
  if (DEBUG>0) {
    cout<<"ACTION MEASURES:  rule   |  #E    |  conf   |   dissim-local   |    all-rules"<<endl;
    FOR1D(possibleGroundActions, i) {
      MT::String name;
      possibleGroundActions(i)->name(name);
      printf("%-12s",(char*)name);
      printf("  %2i", actions__covering_rules(i));
      printf("  %2i", rulesC.experiences_per_rule(actions__covering_rules(i)).N);
      printf("   %6.2f", actions__confidences(i));
      if (actions__dissimilarity_to_previous_experiences__local.N > i) {
        printf("  %6.2f", actions__dissimilarity_to_previous_experiences__local(i));
      }
      cout<<"  "<<actions__covering_rules__all(i);
      cout<<endl;
    }
    
    PRINT(current_state_is_known);
    PRINT(current_state_is_known_partially);
  }
  
  if (DEBUG>0) {cout<<"AbstractRuleExplorer::updateActionAndStateMeasures [END]"<<endl;}
}


void AbstractRuleExplorer::set_p_lower_bounds(double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule) {
  this->p_lower_bound__noise_outcome = p_lower_bound__noise_outcome;
  this->p_lower_bound__noise_outcome_in_default_rule = p_lower_bound__noise_outcome_in_default_rule;
  uint i;
  FOR1D(learners, i) {
    learners(i)->set_p_mins(p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule);
  }
}




// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================


FactoredRuleExplorer::FactoredRuleExplorer(double complexity_penalty_coeff,
                                           double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                                           TL::RuleSet& fixed_rules_for_fixed_actions)
                    : RuleExplorer(factored, complexity_penalty_coeff,p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule) {
  uint DEBUG = 1;
  init_rules();
  // fixed_rules_for_fixed_actions
  uint i;
  fixedActions.clear();
  FOR1D_(fixed_rules_for_fixed_actions, i) {
    fixedActions.setAppend(fixed_rules_for_fixed_actions.elem(i)->action);
  }
  if (DEBUG>0) {cout<<"fixedActions:  "<<fixedActions<<endl;}
  uint k, l;
  FOR1D(fixedActions, k) {
    FOR1D_(fixed_rules_for_fixed_actions, i) {
      if (fixed_rules_for_fixed_actions.elem(i)->action == fixedActions(k)) {
        uintA empty1;
        MT::Array< uintA > empty2; // HACK spaeter removen
        FOR1D(fixed_rules_for_fixed_actions.elem(i)->outcomes, l) {
          empty2.append(empty1);
        }
        rulesC.append(fixed_rules_for_fixed_actions.elem(i), empty1, empty2);
      }
    }
  }
  updateRules();
  if (DEBUG>0) {
    cout<<"fixed_rules_for_fixed_actions:"<<endl;
    fixed_rules_for_fixed_actions.write();
    cout<<"Starting RuleSetContainer:"<<endl;
    rulesC.writeNice();
  }
}

FactoredRuleExplorer::~FactoredRuleExplorer() {
  uint i;
  FOR1D(learners, i) {
    delete learners(i);
  }
}

void FactoredRuleExplorer::init_rules() {
  // default rule
  all_experiences.clear();
  rulesC.init(&all_experiences);
  rulesC.rules.append(TL::ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();
  updateRules();
}


// uint CALLS__updateLogicEngineConstants = 0;
void FactoredRuleExplorer::updateLogicEngineConstants() {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"updateLogicEngineConstants [START]"<<endl;}
//   CALLS__updateLogicEngineConstants++;
//   CHECK(CALLS__updateLogicEngineConstants <= 1, "uiuiui, das muss hier noch angepasst werden, digger; letztlich sind alle learner zurueckzusetzen");
  all_experiences.clear();
  experiences_per_modeledAction.clear();
  experience_weights.clear();
  is_major_experience.clear();
  visited_pre_states.clear();
  visited_actions.clear();
  
  // Create possibleGroundActions
  // Set up all possible actions
  AtomL potential_ground_actions;
  logicObjectManager::getAtoms_actions(potential_ground_actions, logicObjectManager::constants);
  if (DEBUG>0) {cout<<"potential_ground_actions:  "<<potential_ground_actions<<endl;}
  // filter
  uint i;
  modeledActions.clear();
  FOR1D(potential_ground_actions, i) {
    if (potential_ground_actions(i)->args.N == 2  &&  potential_ground_actions(i)->args(0) == potential_ground_actions(i)->args(1))
      continue;
    if (potential_ground_actions(i)->pred->id != TL::DEFAULT_ACTION_PRED__ID) {
      modeledActions.append(potential_ground_actions(i));
    }
  }
  if (DEBUG>0) {cout<<"modeledActions:  "<<modeledActions<<endl;}
  
  this->possibleGroundActions = modeledActions;
  
  
  // kill old learners
  FOR1D(learners, i) {
    delete learners(i);
  }
  learners.clear();
  
  // Set up new learners
  FOR1D(this->modeledActions, i) {
    // Set up rule-learner for each ground action
    double alpha_PEN = complexity_penalty_coeff;
    double p_min = p_lower_bound__noise_outcome;
    double p_min_noisyDefaultRule = p_lower_bound__noise_outcome_in_default_rule;
    uint so_choice_type = 2;
    learners.append(new RuleLearner_ground(alpha_PEN, p_min, p_min_noisyDefaultRule, so_choice_type));
  }
  
  experiences_per_modeledAction.resize(modeledActions.N);
  learners_uptodate.resize(modeledActions.N);
  learners_uptodate.setUni(true);
  
  possibleGroundActions = modeledActions;
  
  if (DEBUG>0) {cout<<"updateLogicEngineConstants [END]"<<endl;}
}



void FactoredRuleExplorer::reset() {
  confident_ground_rules.clear();
  updateLogicEngineConstants();
  
//   bool used_doNothing_action = rulesC.rules.elem(rulesC.rules.num()-1)->action == le->getPI_doNothing();
  
  rulesC.init(&all_experiences);
  rulesC.rules.append(TL::ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();

//   // doNothing rule
//   if (used_doNothing_action ) {
//     uintA fake__experiences_of_this_rule;
//     MT::Array< uintA >  fake__experiences_per_outcome_of_this_rule;
//     fake__experiences_per_outcome_of_this_rule.resize(2);
//     rulesC.append(TL::ruleReasoning::getDoNothingRule(), fake__experiences_of_this_rule, fake__experiences_per_outcome_of_this_rule);
//   }
  
  updateRules();
}


uint FactoredRuleExplorer::action_to_learner_id(TL::Atom* action) {
  int id = possibleGroundActions.findValue(action);
  CHECK(id >= 0, "action "<<*action<<" not found");
  return id;
}


void FactoredRuleExplorer::updateRules(bool always_re_learning) {
  uint DEBUG = 0;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"updateRules [START]"<<endl;}
  uint i, k, l;
  
  // (1) LEARN RULE-SET
  //     --> Take only rule-sets with new examples into account.
  FOR1D(possibleGroundActions, i) {
    if (learners_uptodate(i)) {
    }
    else {
      if (DEBUG>1) {
        cout<<endl<<"Relearning for action "<<*possibleGroundActions(i)<<"   (i="<<i<<")"<<endl;
        if (DEBUG>2) {
          cout<<"BEFORE rule-set:"<<endl;
          rulesC.writeNice();
        }
      }
      if (fixedActions.findValue(modeledActions(i)) >= 0)
        HALT("");
      
//       // (i) save old example coverage statistics
//       MT::Array< uintA > old_experience_partitions;
//       rulesC.getPartitionsForAction(old_experience_partitions, possibleGroundActions(i));
//       if (DEBUG>1) {
//         cout<<"Old partitions (" << old_experience_partitions.N << "):"<<endl;
//         FOR1D(old_experience_partitions, k) {
//           uintA rules_ids = rulesC.nonDefaultRules_per_experience(old_experience_partitions(k)(0));
//           if (rules_ids.N == 1) {
//             TL::writeNice(rulesC.rules.elem(rules_ids(0))->context);
//           }
//           else
//             cout<<"Default rule";
//           cout<<endl<<"   "<<old_experience_partitions(k)<<endl;
//         }
//       }
//       // save old rule for latest example
//       TL::Rule* old_covering_rule_for_latest_example = NULL;
//       uint old_covering_rule_for_latest_example__id = TL::ruleReasoning::uniqueAbstractCoveringRule_groundedAction(rulesC.rules, *visited_pre_states.last(), visited_actions.last());
//       if (old_covering_rule_for_latest_example__id != 0) {
//         old_covering_rule_for_latest_example = new TL::Rule;
//         old_covering_rule_for_latest_example->copyBody(*rulesC.rules.elem(old_covering_rule_for_latest_example__id));
//       }
//       
      // (ii) remove old rules for this action
      FOR1D_DOWN_(rulesC.rules, k) {
        if (rulesC.rules.elem(k)->action == possibleGroundActions(i))
          rulesC.remove(k);
      }
      
      if (DEBUG>2) {
        cout<<endl<<"FILTERED rule-set:"<<endl;
        rulesC.writeNice();
      }
      
      // (iii) learn new rules for this action
      TL::RuleSetContainer_ground rulesC_for_action;
      SymbolicExperienceL action_experiences;
      arr action_experience_weights;
      FOR1D(experiences_per_modeledAction(i), k) {
        action_experiences.append(all_experiences(experiences_per_modeledAction(i)(k)));
        action_experience_weights.append(experience_weights(experiences_per_modeledAction(i)(k)));
      }
      learners(i)->setAlphaPEN(complexity_penalty_coeff * 0.1 * action_experiences.N);  // TODO anpassen?
//       learners(i)->setAlphaPEN(complexity_penalty_coeff * exp(0.005 * action_experiences.N));  // TODO anpassen?
//       learners(i)->setAlphaPEN(complexity_penalty_coeff * (log(action_experiences.N)+1.));  // TODO anpassen?
      learners(i)->learn_rules(rulesC_for_action, action_experiences, action_experience_weights);
      if (DEBUG>2) {
        cout<<endl<<"NEW RULES for action:"<<endl;
        rulesC_for_action.writeNice();
      }
      
      // (iv) insert new rules into overall container
      FOR1D_(rulesC_for_action.rules, k) {
        if (TL::ruleReasoning::isDefaultRule(rulesC_for_action.rules.elem(k)))
          continue;
        
        // Interfacing between different experience-ids
        
        uintA experiences_per_rule__converted;
        FOR1D(rulesC_for_action.experiences_per_rule(k), l) {
          experiences_per_rule__converted.append(experiences_per_modeledAction(i)(rulesC_for_action.experiences_per_rule(k)(l)));
        }
        
        MT::Array < uintA > experiences_per_ruleOutcome__converted;
        experiences_per_ruleOutcome__converted.resize(rulesC_for_action.experiences_per_ruleOutcome(k).N);
        uint o;
        FOR1D(experiences_per_ruleOutcome__converted, o) {
          FOR1D(rulesC_for_action.experiences_per_ruleOutcome(k)(o), l) {
            experiences_per_ruleOutcome__converted(o).append(experiences_per_modeledAction(i)(rulesC_for_action.experiences_per_ruleOutcome(k)(o)(l)));
          }
        }
        
        if (DEBUG>2) {
          PRINT(rulesC_for_action.experiences_per_rule(k));
          PRINT(experiences_per_rule__converted);
          PRINT(rulesC_for_action.experiences_per_ruleOutcome(k));
          PRINT(experiences_per_ruleOutcome__converted);
        }
        
        rulesC.append(rulesC_for_action.rules.elem(k), experiences_per_rule__converted, experiences_per_ruleOutcome__converted);
      }
      
      if (DEBUG>2) {
        cout<<endl<<"AFTER rule-set:"<<endl;
        rulesC.writeNice();
      }
      
//       // (v) investigate whether major experience
//       // compare new partitions
//       MT::Array< uintA > new_experience_partitions;
//       rulesC.getPartitionsForAction(new_experience_partitions, abstract_actions(i));
//       if (DEBUG>1) {
//         cout<<"New partitions (" << new_experience_partitions.N << "):"<<endl;
//         FOR1D(new_experience_partitions, k) {
//           uintA rules_ids = rulesC.nonDefaultRules_per_experience(new_experience_partitions(k)(0));
//           if (rules_ids.N == 1) {
//             TL::writeNice(rulesC.rules.elem(rules_ids(0))->context);
//           }
//           else
//             cout<<"Default rule";
//           FOR1D(old_experience_partitions, l) {
//             uintA helfer = new_experience_partitions(k);
//             helfer.removeValueSafe(all_experiences.N-1);
//             if (helfer == old_experience_partitions(l)) {
//               break;
//             }
//           }
//           if (old_experience_partitions.N == l) {
//             cout<<"  <-----";
//           }
//           cout<<endl<<"   "<<new_experience_partitions(k)<<endl;
//         }
//       }
//       bool partitions_changed = false;
//       FOR1D(new_experience_partitions, k) {
//         new_experience_partitions(k).removeValueSafe(all_experiences.N-1);  // remove latest experience-id
//         FOR1D(old_experience_partitions, l) {
//           if (new_experience_partitions(k) == old_experience_partitions(l)) {
//             break;
//           }
//         }
//         if (old_experience_partitions.N == l) {
//           partitions_changed = true;
//           break;
//         }
//       }
//       if (DEBUG>1) {PRINT(partitions_changed);}
//       bool covering_rule_for_last_experience_changed = false;
//       if (!partitions_changed) {
//         TL::Rule* new_covering_rule_for_latest_example = NULL;
//         if (rulesC.nonDefaultRules_per_experience.last().N == 1)
//           new_covering_rule_for_latest_example = rulesC.rules.elem(rulesC.nonDefaultRules_per_experience.last()(0));
//         if (new_covering_rule_for_latest_example != NULL  &&  old_covering_rule_for_latest_example != NULL) {
//           if (new_covering_rule_for_latest_example->context != old_covering_rule_for_latest_example->context) {
//             covering_rule_for_last_experience_changed = true;
//             if (DEBUG>1) {
//               cout<<"Major experience since context has changed"<<endl;
//               cout<<"old_covering_rule_for_latest_example->context:  ";  TL::writeNice(old_covering_rule_for_latest_example->context);  cout<<endl;
//               cout<<"new_covering_rule_for_latest_example->context:  ";  TL::writeNice(new_covering_rule_for_latest_example->context);  cout<<endl;
//             }
//           }
//         }
//         else if (new_covering_rule_for_latest_example != NULL  &&  old_covering_rule_for_latest_example == NULL) {
//           covering_rule_for_last_experience_changed = true;
//           if (DEBUG>1) {cout<<"Major experience since before we had a null rule."<<endl;}
//         }
//       }
//       
//       
//       if (partitions_changed  ||  covering_rule_for_last_experience_changed)
//         is_major_experience.last() = true;
      
//       if (DEBUG>1) {PRINT(is_major_experience.last());}
      
      
      learners_uptodate(i) = true;
    }
  }
  
  rulesC.recomputeDefaultRule();
  rulesC.sort();
  
  //   rulesC.writeNice();
//   rulesC.write_experiencesWithRules();
  rulesC.sanityCheck();
  
  

  // (2) CREATE RULE CONFIDENCES
 
  rules__confidences.resize(rulesC.rules.num());
  FOR1D_(rulesC.rules, i) {
    rules__confidences(i) = calcRuleConfidence(i);
  }


  // (3) CONFIDENT GROUND RULES
  rules__confidences.resize(rulesC.rules.num());
  confident_ground_rules.clear();
  FOR1D_(rulesC.rules, i) {
    rules__confidences(i) = calcRuleConfidence(i);
    if (rules__confidences(i) >= RULE_CONFIDENCE_THRESHOLD) {
      confident_ground_rules.append(rulesC.rules.elem(i));
    }
  }
  
  if (DEBUG>0) {
    cout<<"LEARNED RULE-SET (" << visited_pre_states.N << " examples)"<<endl;
    FOR1D_(rulesC.rules, i) {
      if (i==0  ||  (rulesC.rules.elem(i)->action != rulesC.rules.elem(i-1)->action)) {
        cout<<"######  "<<*rulesC.rules.elem(i)->action<<" ######";
      }
      if (rulesC.rules.elem(i)->outcomes.N == 2   &&  rulesC.rules.elem(i)->outcomes(0).N == 0) {
        cout<<"   (no-effect-rule)"<<endl;
        continue;
      }
      cout<<endl;
      cout<<"-----   #"<<i<<":    conf="<<rules__confidences(i)
          <<"  (" << rulesC.experiences_per_rule(i).N << "/" 
          << visited_pre_states.N << " = " << (1.0 * rulesC.experiences_per_rule(i).N) / visited_pre_states.N << ") " 
          << rulesC.experiences_per_rule(i) << endl;
      rule_write_hack(rulesC.rules.elem(i), rulesC.experiences_per_ruleOutcome(i), false, cout);
    }
    cout<<endl;
  
//     cerr<<"LEARNED RULE-SET (" << visited_pre_states.N << " examples)"<<endl;
//     FOR1D_(rulesC.rules, i) {
//       if (i==0  ||  (rulesC.rules.elem(i)->action != rulesC.rules.elem(i-1)->action)) {
//         cerr<<"######  "; rulesC.rules.elem(i)->action->writeNice(cerr);  cerr<<" ######"<<endl;
//       }
//       cerr<<"-----  #"<<i<<":    conf="<<rules__confidences(i)
//           <<"  (" << rulesC.experiences_per_rule(i).N << "/" 
//           << visited_pre_states.N << " = " << (1.0 * rulesC.experiences_per_rule(i).N) / visited_pre_states.N << ") " 
//           << rulesC.experiences_per_rule(i) << endl;
//       rule_write_hack(rulesC.rules.elem(i), rulesC.experiences_per_ruleOutcome(i), false, cerr);
//     }
//     cerr<<endl;
  }
  
  if (DEBUG>0) {cout<<"updateRules [END]"<<endl;}
}


void FactoredRuleExplorer::addObservation(State* state_pre, Atom* action, State* state_post) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"addObservation [START]"<<endl;}
  if (fixedActions.findValue(action) >= 0) {
    visited_pre_states.append(state_pre);
    visited_actions.append(action);
    SymbolicExperience* exp = new SymbolicExperience(*state_pre, action, *state_post);
    all_experiences.append(exp);
    is_major_experience.append(false);
    experience_weights.append(1.0);
    experiences_per_modeledAction(action_to_learner_id(action)).append(visited_pre_states.N-1);
    // calculate which rule is covering
    uintA coveringRuleIDs;
    TL::ruleReasoning::coveringGroundedRules_groundedAction(rulesC.rules, *state_pre, action, coveringRuleIDs);
    coveringRuleIDs.removeValueSafe(0);
    if (DEBUG>0) {PRINT(coveringRuleIDs);}
    rulesC.nonDefaultRules_per_experience.append(coveringRuleIDs);
    // covering outcomes
    uint i, k;
    FOR1D(coveringRuleIDs, i) {
      if (coveringRuleIDs(i) != 0) {
        rulesC.experiences_per_rule(coveringRuleIDs(i)).append(all_experiences.N-1);
        uintA covering_outcomes;
        TL::ruleReasoning::coveringOutcomes(rulesC.rules.elem(coveringRuleIDs(i)), *state_pre, *state_post, covering_outcomes);
        if (DEBUG>0) {rulesC.rules.elem(coveringRuleIDs(i))->write();  cout<<endl;  PRINT(covering_outcomes);}
        FOR1D(covering_outcomes, k) {
          if (covering_outcomes.N > 1  &&  k == covering_outcomes.N - 1)
            continue;
          rulesC.experiences_per_ruleOutcome(coveringRuleIDs(i))(covering_outcomes(k)).append(all_experiences.N-1);
        }
      }
    }
    if (DEBUG>0) {cout<<"addObservation [END]"<<endl;}
  }
  else {
    addObservation__helper(state_pre, action, state_post);
    uintA empty;
    rulesC.nonDefaultRules_per_experience.append(empty);
  }
}








// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================


FlatExplorer::FlatExplorer(TL::RuleSet& fixed_rules_for_fixed_actions) 
          : RuleExplorer(flat, 0., 0., 0.) {
  init_rules();
  fixed_rules_memory = fixed_rules_for_fixed_actions;
}

FlatExplorer::~FlatExplorer() {
}

void FlatExplorer::init_rules() {
//   if (withDoNothingAction) {
//     rules_hierarchy.resize(modeledActions.N+2);  // +2 fuer default + doNothing
//     rules_hierarchy(rules_hierarchy.N-2).append(TL::ruleReasoning::getDoNothingRule());
//   }
//   else {
    rules_hierarchy.resize(modeledActions.N+1);
//   }
  rules_hierarchy(rules_hierarchy.N-1).append(TL::ruleReasoning::generateDefaultRule());
  
  updateRules();
}


// uint CALLS__updateLogicEngineConstants_FLAT = 0;
void FlatExplorer::updateLogicEngineConstants() {
  uint DEBUG = 1;
  if (DEBUG>0) {cout<<"updateLogicEngineConstants [START]"<<endl;}
//   CALLS__updateLogicEngineConstants_FLAT++;
//   CHECK(CALLS__updateLogicEngineConstants_FLAT <= 1, "uiuiui, das muss hier noch angepasst werden, digger; letztlich sind alle learner zurueckzusetzen");
  
  // Create possibleGroundActions
  // Set up all possible actions
  AtomL potential_ground_actions;
  logicObjectManager::getAtoms_actions(potential_ground_actions, logicObjectManager::constants);
  // filter
  uint i;
  modeledActions.clear();
  FOR1D(potential_ground_actions, i) {
    if (potential_ground_actions(i)->args.N == 2  &&  potential_ground_actions(i)->args(0) == potential_ground_actions(i)->args(1))
      continue;
    if (potential_ground_actions(i)->pred->id != TL::DEFAULT_ACTION_PRED__ID) {
      modeledActions.append(potential_ground_actions(i));
    }
  }
  if (DEBUG>0) {cout<<"modeledActions:  "<<modeledActions<<endl;}
  
  this->possibleGroundActions = modeledActions;
  
  // update rule-set
  TL::RuleSet default_rule_wrapper;
  default_rule_wrapper.append(rules_hierarchy.last().elem(0));
//   TL::RuleSet doNothing_rule_wrapper;
//   if (rules_hierarchy.N > 1)
//     doNothing_rule_wrapper.append(rules_hierarchy(rules_hierarchy.N-2).elem(0));
  rules_hierarchy.clear();
  rule_confidences__hierarchy.clear();
  experiences_per_rule__hierarchy.clear();
  experiences_per_rule__flat.clear();
//   if (doNothing_rule_wrapper.num() > 0) {
//     if (modeledActions.findValue(le->getPI_doNothing()) >= 0) {
//       rules_hierarchy.resize(modeledActions.N+1);  // +1 fuer default
//       rules_hierarchy(modeledActions.findValue(le->getPI_doNothing())) = doNothing_rule_wrapper;
//     }
//     else {
//       rules_hierarchy.resize(modeledActions.N+2);  // +2 fuer default + doNothing
//       rules_hierarchy(rules_hierarchy.N-2) = doNothing_rule_wrapper;
//     }
//     rules_hierarchy.last() = default_rule_wrapper;
//   }
//   else {
    rules_hierarchy.resize(modeledActions.N+1);  // +1 fuer default
    rules_hierarchy.last() = default_rule_wrapper;
//   }
  rule_confidences__hierarchy.resize(rules_hierarchy.N);
  experiences_per_rule__hierarchy.resize(rules_hierarchy.N);
  
  
  // fixed_rules_for_fixed_actions
  fixedActions.clear();
  FOR1D_(fixed_rules_memory, i) {
    fixedActions.setAppend(fixed_rules_memory.elem(i)->action);
  }
  if (DEBUG>0) {cout<<"fixedActions:  "<<fixedActions<<endl;}
  uint k;
  FOR1D(fixedActions, k) {
    int action_id = modeledActions.findValue(fixedActions(k));
    FOR1D_(fixed_rules_memory, i) {
      if (fixed_rules_memory.elem(i)->action == fixedActions(k)) {
        rules_hierarchy(action_id).append(fixed_rules_memory.elem(i));
        rule_confidences__hierarchy(action_id).append(TUP(99));
        experiences_per_rule__hierarchy(action_id).append(TUP());
      }
    }
  }
  updateRules();
  if (DEBUG>0) {
    cout<<"fixed_rules_memory:"<<endl;
    fixed_rules_memory.write();
    cout<<"Starting RuleSetContainer:"<<endl;
    FOR1D(rules_hierarchy, i) {
      if (i<rules_hierarchy.N-2)
        cout<<"---  i="<<i<<"     "<<*modeledActions(i)<<"  -----------------------"<<endl;
      else
        cout<<"---  i="<<i<<"     -----------------------"<<endl;
      FOR1D_(rules_hierarchy(i), k) {
        if (k>0) cout<<"-------"<<endl;
        rules_hierarchy(i).elem(k)->write();
      }
    }
  }
  if (DEBUG>0) {cout<<"updateLogicEngineConstants [END]"<<endl;}
}



void FlatExplorer::reset() {
  confident_ground_rules.clear();
  updateLogicEngineConstants();
  rules_flat.clear();
  updateRules();
}


uint FlatExplorer::action_to_learner_id(TL::Atom* action) {
  NIY;
  return -1;
//   int id = possibleGroundActions.findValue(action);
//   CHECK(id >= 0, "action "<<*action<<" not found");
//   return id;
}


void FlatExplorer::updateRules(bool always_re_learning) {
  uint DEBUG = 1;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"updateRules [START]"<<endl;}
  uint i, k;
  
  // Flatten out hierarchy
  if (DEBUG>0) {
    cout<<"Hierarchy:"<<endl;
    FOR1D(rules_hierarchy, i) {
      cout<<"---  i="<<i<<"   (" <<rules_hierarchy(i).num()<< ")   ";
      if (i < modeledActions.N)
        cout<<*modeledActions(i);
      cout<<"  -----------------------"<<endl;
      FOR1D_(rules_hierarchy(i), k) {
        if (k>0) cout<<"-------"<<endl;
        rules_hierarchy(i).elem(k)->write();
      }
    }
  }
  
  if (DEBUG>0) {cout<<"Flatten out hierarchy"<<endl;}
  uint total_num_rules = 0;
  FOR1D(rules_hierarchy, i) {
    total_num_rules += rules_hierarchy(i).num();
  }
  if (DEBUG>0) {PRINT(total_num_rules);}
  learners_uptodate.resize(total_num_rules);
  learners_uptodate.setUni(true);
  experiences_per_rule__flat.resize(total_num_rules);
  rules__confidences.resize(total_num_rules);
  uint q=0;
  // default rule
  rules_flat.clear();
  rules_flat.append(rules_hierarchy.last().elem(0));
  rules__confidences(q) = 0.;
  q++;
//   // doNothing rule
//   rules_flat.append(rules_hierarchy(rules_hierarchy.N-2).elem(0));
//   rules__confidences(q) = 99.;
//   q++;
  for (i=0; (int) i<(int) rules_hierarchy.N-2; i++) {  // omit default rule
    FOR1D_(rules_hierarchy(i), k) {
      experiences_per_rule__flat(q) = experiences_per_rule__hierarchy(i)(k);
      rules_flat.append(rules_hierarchy(i).elem(k));
      rules__confidences(q) = rule_confidences__hierarchy(i)(k);
      q++;
    }
  }
  
  if (DEBUG>2) {
    FOR1D_(rules_flat, i) {
      cout<<"Rule #i="<<i<<endl;
      rules_flat.elem(i)->write();
      PRINT(rules__confidences(i));
      PRINT(experiences_per_rule__flat(i));
    }
  }

  
  // ----------------------------------------
  // Set confident rules
  confident_ground_rules.clear();
  FOR1D_(rules_flat, i) {
    if (rules__confidences(i) >= RULE_CONFIDENCE_THRESHOLD) {
      confident_ground_rules.append(rules_flat.elem(i));
    }
  }
  
  
  if (DEBUG>0) {cout<<"updateRules [END]"<<endl;}
}




void FlatExplorer::addObservation(TL::State* state_pre, TL::Atom* action, TL::State* state_post) {
  uint DEBUG = 1;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"FlatExplorer::addObservation [START]"<<endl;}
  if (DEBUG>0) {
    cout<<"S_PRE:   "<<endl;  state_pre->write();  cout<<endl;
    cout<<"ACTION:  "<<endl;  action->write();  cout<<endl;
    cout<<"S_POST:  "<<endl;  state_post->write();  cout<<endl;
  }
  // General stuff
  visited_pre_states.append(state_pre);
  visited_actions.append(action);
  SymbolicExperience* exp = new SymbolicExperience(*state_pre, action, *state_post);
  all_experiences.append(exp);
  is_major_experience.append(false);
  experience_weights.append(1.0);

  // Rule-hierarchy
  int action_id = modeledActions.findValue(action);
  CHECK(action_id >= 0, "action " << *action << " not among modeled actions");
  if (DEBUG>0) {PRINT(action_id);}
  uint i;
  FOR1D_(rules_hierarchy(action_id), i) {
    if (TL::ruleReasoning::cover_groundRule_groundedAction(*state_pre, action, rules_hierarchy(action_id).elem(i)))
      break;
  }
  if (DEBUG>0) {PRINT(i);  PRINT(rules_hierarchy(action_id).num()); }
  // (Case 1) Fixed action
  if (fixedActions.findValue(action) >= 0) {
    if (DEBUG>0) {cout<<"Is a fixed action."<<endl;}
    if (i<rules_hierarchy(action_id).num()) {
      experiences_per_rule__hierarchy(action_id)(i).append(visited_pre_states.N-1);
    }
    else {
      if (experiences_per_rule__hierarchy.last().N == 0) {
        experiences_per_rule__hierarchy.last().append(TUP());
      }
      experiences_per_rule__hierarchy.last()(0).append(visited_pre_states.N-1);
    }
    if (DEBUG>0) {cout<<"FlatExplorer::addObservation [END]"<<endl;}
    return;
  }
  // (Case 2) Rule exists already
  else if (i<rules_hierarchy(action_id).num()) {
    TL::Rule* existing_rule = rules_hierarchy(action_id).elem(i);
    if (DEBUG>0) {cout<<"Integrating into existing rule:"<<endl;  existing_rule->write();}
    // integrate somehow
    experiences_per_rule__hierarchy(action_id)(i).append(visited_pre_states.N-1);
    rule_confidences__hierarchy(action_id)(i)++;
    // construct potential new outcome
    LitL lits_diff_1to2;  FuncVL fv_diff_1to2;  LitL lits_diff_2to1;  FuncVL fv_diff_2to1;
    logicReasoning::calcDifferences(lits_diff_1to2, fv_diff_1to2, lits_diff_2to1, fv_diff_2to1, *state_pre, *state_post);
    LitL new_outcome;
    FOR1D(lits_diff_2to1, i) {
      if (lits_diff_2to1(i)->atom->pred->category == category_primitive)
        new_outcome.append(lits_diff_2to1(i));
    }
    FOR1D(lits_diff_1to2, i) {
      if (lits_diff_1to2(i)->atom->pred->category == category_primitive)
        new_outcome.append(logicObjectManager::getLiteralNeg(lits_diff_1to2(i)));
    }
    if (DEBUG>0) {cout<<"Potential new_outcome:   "<<new_outcome<<endl;}
    FOR1D(existing_rule->outcomes, i) {
      if (existing_rule->outcomes(i) == new_outcome) {
        if (DEBUG>0) {cout<<"Same as outcome #i="<<i<<endl;}
        existing_rule->probs(i) *= 2.0;  // TODO das ist nicht richtig, aber scheiss egal
        break;
      }
    }
    // New outcome
    if (i==existing_rule->outcomes.N) {
      if (DEBUG>0) {cout<<"Indeed a new outcome!"<<endl;}
      MT::Array< LitL > new_outcomes(existing_rule->outcomes.N+1);
      arr new_probs(existing_rule->outcomes.N+1);
      uint k;
      for(k=0; k<existing_rule->outcomes.N-1; k++) {
        new_outcomes(k) = existing_rule->outcomes(k);
        new_probs(k) = existing_rule->probs(k);
      }
      new_outcomes(k) = new_outcome;
      new_probs(k) = existing_rule->probs.max();    // TODO das ist nicht richtig, aber scheiss egal
      new_outcomes.last() = existing_rule->outcomes.last();
      new_probs.last() = existing_rule->probs.last();
      existing_rule->outcomes = new_outcomes;
      existing_rule->probs = new_probs;
    }
    existing_rule->probs /= sum(existing_rule->probs);
    if (DEBUG>0) {cout<<"Updated rule:"<<endl;  existing_rule->write();  cout<<endl;}
  }
  // (Case 3) Build new rule
  else {
    if (DEBUG>0) {cout<<"Building new rule"<<endl;}
    // administration
    rule_confidences__hierarchy(action_id).append(TUP(1));
    experiences_per_rule__hierarchy(action_id).append(TUP(visited_pre_states.N-1));
    // new rule
    TL::Rule* rule = new TL::Rule;
    // Context
    FOR1D(state_pre->lits_prim, i) {
      rule->context.append(state_pre->lits_prim(i));
    }
    LitL other_lits__positive;
    logicObjectManager::getLiterals(other_lits__positive, logicObjectManager::constants, true);
    FOR1D(other_lits__positive, i) {
      if (rule->context.N > 18)  // HACK PRADA verarbeitet Kontexte nur bis maximal 18 Konzepten
        break;
      TL::Literal* lit_neg = logicObjectManager::getLiteralNeg(other_lits__positive(i));
      if (logicReasoning::holds(*state_pre, lit_neg )) {
        rule->context.append(lit_neg );
      }
    }
    if (DEBUG>0) {cout<<"rule->context:    "<<rule->context<<endl;}
    // TODO reischt das?
    // Action
    rule->action = action;
    // Outcomes
    rule->outcomes.resize(2);
    LitL lits_diff_1to2;  FuncVL fv_diff_1to2;  LitL lits_diff_2to1;  FuncVL fv_diff_2to1;
    logicReasoning::calcDifferences(lits_diff_1to2, fv_diff_1to2, lits_diff_2to1, fv_diff_2to1, *state_pre, *state_post);
    FOR1D(lits_diff_2to1, i) {
      if (lits_diff_2to1(i)->atom->pred->category == category_primitive)
        rule->outcomes(0).append(lits_diff_2to1(i));
    }
    FOR1D(lits_diff_1to2, i) {
      if (lits_diff_1to2(i)->atom->pred->category == category_primitive)
        rule->outcomes(0).append(logicObjectManager::getLiteralNeg(lits_diff_1to2(i)));
    }
    rule->probs.resize(2);
    rule->probs(0) = 1.0;
    rule->probs(1) = 0.0;
    if (DEBUG>0) {cout<<"New rule:"<<endl;  rule->write();}
    rules_hierarchy(action_id).append(rule);
    bool coverage_test =  TL::ruleReasoning::cover_groundRule_groundedAction(*state_pre, action, rules_hierarchy(action_id).elem(rules_hierarchy(action_id).num()-1));
    PRINT(coverage_test);
    CHECK(coverage_test, "");
  }
  if (DEBUG>0) {cout<<"FlatExplorer::addObservation [END]"<<endl;}
}


int TL::FlatExplorer::action_state_to_flat_id(TL::Atom* action, TL::State* state) {
  uint i, k;
  int idx_actions = modeledActions.findValue(action);
  CHECK(idx_actions>=0, "");
  int num = 0;
  num += 1; // fuer default rule
  FOR1D(rules_hierarchy, i) {
    if (i < (uint) idx_actions)
      num += rules_hierarchy(i).num();
    else {
      FOR1D_(rules_hierarchy(i), k) {
        if (TL::ruleReasoning::cover_groundRule_groundedAction(*state, action, rules_hierarchy(i).elem(k)))
          break;
      }
      if (k<rules_hierarchy(i).num())
        num += k;
      else
        num = 0;  // default rule
      break;
    }
  }
  return num;
}


void TL::FlatExplorer::get_nonDefaultRules_for_last_experience(uintA& rule_ids) {
  int idx = action_state_to_flat_id(visited_actions.last(), visited_pre_states.last());
  rule_ids.clear();
  rule_ids.append(idx);
}







// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================
// ====================================================================================================


// ---------------------------------------------------------------------------
//   RelationalStateGraph

RelationalStateGraph::RelationalStateGraph(const TL::State& _state) : state(_state) {
  // read constants
  logicReasoning::getConstants(state, constants);
  uint i;
  
  // Predicates
  
  lits_zeroary.clear();
  lits_unary.resize(constants.N);
  lits_binary.resize(constants.N, constants.N);
  lits_binary_matrix.resize(constants.N, constants.N);
  lits_binary_matrix.setUni(false);
  
  FOR1D(state.lits_prim, i) {
    // ignore homies
    if (state.lits_prim(i)->atom->pred->name == MT::String("homies")) continue;
    if (state.lits_prim(i)->atom->args.N == 0) {
      lits_zeroary.append(state.lits_prim(i));
    }
    else if (state.lits_prim(i)->atom->args.N == 1) {
      uint id_arg = constants.findValue(state.lits_prim(i)->atom->args(0));
      lits_unary(id_arg).append(state.lits_prim(i));
    }
    else if (state.lits_prim(i)->atom->args.N == 2) {
      uint id_arg_1 = constants.findValue(state.lits_prim(i)->atom->args(0));
      uint id_arg_2 = constants.findValue(state.lits_prim(i)->atom->args(1));
      lits_binary(id_arg_1, id_arg_2).append(state.lits_prim(i));
      lits_binary_matrix(id_arg_1, id_arg_2) = true;
    }
    else
      NIY;
  }
  
  FOR1D(state.lits_derived, i) {
    if (state.lits_derived(i)->atom->args.N == 0) {
      lits_zeroary.append(state.lits_derived(i));
    }
    else if (state.lits_derived(i)->atom->args.N == 1) {
      uint id_arg = constants.findValue(state.lits_derived(i)->atom->args(0));
      lits_unary(id_arg).append(state.lits_derived(i));
    }
    else if (state.lits_derived(i)->atom->args.N == 2) {
      uint id_arg_1 = constants.findValue(state.lits_derived(i)->atom->args(0));
      uint id_arg_2 = constants.findValue(state.lits_derived(i)->atom->args(1));
      lits_binary(id_arg_1, id_arg_2).append(state.lits_derived(i));
      lits_binary_matrix(id_arg_1, id_arg_2) = true;
    }
    else
      NIY;
  }
  
  // Functions
  
  fvs_zeroary.clear();
  fvs_unary.resize(constants.N);

  FOR1D(state.fv_prim, i) {
    if (state.fv_prim(i)->atom->args.N == 0) {
      fvs_zeroary.append(state.fv_prim(i));
    }
    else if (state.fv_prim(i)->atom->args.N == 1) {
      uint id_arg = constants.findValue(state.fv_prim(i)->atom->args(0));
      fvs_unary(id_arg).append(state.fv_prim(i));
    }
    else
      NIY;
  }
  
  FOR1D(state.fv_derived, i) {
    if (state.fv_derived(i)->atom->args.N == 0) {
      fvs_zeroary.append(state.fv_derived(i));
    }
    else if (state.fv_derived(i)->atom->args.N == 1) {
      uint id_arg = constants.findValue(state.fv_derived(i)->atom->args(0));
      fvs_unary(id_arg).append(state.fv_derived(i));
    }
    else
      NIY;
  }

}

RelationalStateGraph::~RelationalStateGraph() {
}

// TODO ineffizient
void RelationalStateGraph::getRelatedConstants(uintA& related_constants, uint obj, uint depth) const {
  related_constants.clear();
  related_constants.append(obj);
  uint i, k;
  for (i=0; i<depth; i++) {
    uintA all_neighbors;
    FOR1D(related_constants, k) {
      uintA neighbors;
      getDirectNeighbors(neighbors, related_constants(k));
      all_neighbors.setAppend(neighbors);
    }
    related_constants.setAppend(all_neighbors);
  }
  related_constants.removeValue(obj);
}

// TODO ineffizient
void RelationalStateGraph::getRelatedConstants(uintA& related_constants, const uintA& objs, uint depth) const {
  related_constants.clear();
  related_constants.append(objs);
  uint i;
  FOR1D(objs, i) {
    uintA local_related_constants;
    getRelatedConstants(local_related_constants, objs(i), depth);
    related_constants.setAppend(local_related_constants);
  }
  FOR1D(objs, i) {
    related_constants.removeValue(objs(i));
  }
}


void RelationalStateGraph::getDirectNeighbors(uintA& neighbors, uint obj) const {
  neighbors.clear();
  int i;
  int id = constants.findValue(obj);
  if (id < 0) MT_MSG("obj="<<obj<<" not found in constants="<<constants);
  if (id >= 0) {
    FOR1D(constants, i) {
      if (i == id)
        continue;
      if (lits_binary_matrix(i, id) || lits_binary_matrix(id, i))
        neighbors.setAppend(constants(i));
    }
  }
}

RelationalStateGraph* RelationalStateGraph::getSubgraph(const uintA& objects) const {
  TL::State s_filtered;
  logicReasoning::filterState_full(s_filtered, state, objects, false);  // only "filter_objects" as args
  RelationalStateGraph* graph = new RelationalStateGraph(s_filtered);
  return graph;
}



void RelationalStateGraph::writeNice(ostream& os) const {
  uint i, k;
  os<<"0-ary pis:  "<<lits_zeroary<<"  |  0-ary fvs:  "<<fvs_zeroary<<endl;
  FOR1D(constants, i) {
    os<<"("<<i<<") "<<constants(i)<<":   "<<lits_unary(i)<<"  "<<fvs_unary(i)<<endl;
    FOR1D(constants, k) {
      if (i == k)
        continue;
      if (lits_binary_matrix(i, k)) {
        os<<"    -->   "<<constants(k)<<"  "<<lits_binary(i,k)<<endl;
      }
      if (lits_binary_matrix(k, i)) {
        os<<"    <--   "<<constants(k)<<"  "<<lits_binary(k,i)<<endl;
      }
    }
  }
}


// TODO achtung! this is rather _relative_ distance as it is dependent on graph size
double RelationalStateGraph::distance(const RelationalStateGraph& g1, const TL::Atom& a1,
                         const RelationalStateGraph& g2, const TL::Atom& a2) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RelationalStateGraph::distance [START]"<<endl;}
  if (DEBUG>0) {
    cout<<"*** Action 1:  "<<a1<<endl;  cout<<"Graph 1:"<<endl;  g1.writeNice();
    cout<<"*** Action 2:  "<<a2<<endl;  cout<<"Graph 2:"<<endl;  g2.writeNice();
  }
  CHECK(a1.pred == a2.pred, "bad actions:   a1="<<a1<<"  vs.  a2="<<a2);
  TL::Substitution* init_sub = new TL::Substitution;
  if (a1.args.N > 0) {init_sub->addSubs(a1.args(0), a2.args(0));}
  TL::SubstitutionSet subs;
  
  #if 1
  // HACK fuer das homie-Praedikat [START]
  uint i;
  TL::State hack_g1_state = g1.state;
  hack_g1_state.lits_prim.memMove = true;
  FOR1D_DOWN(hack_g1_state.lits_prim, i) {
    if (hack_g1_state.lits_prim(i)->atom->pred->name == MT::String("homies"))
      hack_g1_state.lits_prim.remove(i);
  }
  
  TL::State hack_g2_state = g2.state;
  hack_g2_state.lits_prim.memMove = true;
  FOR1D_DOWN(hack_g2_state.lits_prim, i) {
    if (hack_g2_state.lits_prim(i)->atom->pred->name == MT::String("homies"))
      hack_g2_state.lits_prim.remove(i);
  }
  uint min_differing_attributes = logicReasoning::unifyAsMuchAsPossible(subs, hack_g1_state, hack_g2_state, init_sub);
  // HACK fuer das homie-Praedikat [END]
  #else
  uint min_differing_attributes = logicReasoning::unifyAsMuchAsPossible(subs, g1.state, g2.state, init_sub);
  #endif
  // TODO Heuristik: if   min_differing_attributes >= FULL_DIFFERENCE_THRESHOLD   -->   distance = 1
  // distance should also depend on graph sizes: if sizes are small, then already minor differences make them different
  double FULL_DIFFERENCE_THRESHOLD = TL_MIN(TL_MAX(g1.constants.N, g2.constants.N) * 2.0, 10.);
  double distance = (min_differing_attributes * 1.0) / FULL_DIFFERENCE_THRESHOLD;
  distance = TL_MIN(distance, 1.0);
  if (DEBUG>0) {PRINT(min_differing_attributes);  PRINT(distance);}
  if (min_differing_attributes < 0.) {HALT("minDiff="<<min_differing_attributes<<"  is non-neg");}
  if (distance < 0.  ||  distance > 1.0) {HALT("distance="<<distance<<"  is behindert");}
  if (DEBUG>0) {cout<<"RelationalStateGraph::distance [END]"<<endl;}
  return distance;
}


double RelationalStateGraph::entropy(const AtomL& actions, const MT::Array< RelationalStateGraph* >& graphs) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RelationalStateGraph::entropy [START]"<<endl;}
  if (DEBUG>0) {PRINT(graphs.N); PRINT(actions); }
  uint i;
  int k;
  double entropy = 0.0;
  FOR1D(graphs, i) {
    CHECK(actions(i)->pred == actions(0)->pred, "bad actions:   actions(i)="<<*actions(i)<<"  vs.  actions(0)="<<*actions(0));
    if (DEBUG>1) {cout<<"+++ i="<<i<<endl;}
    double min_distance_to_others = 1.0;
    // comparison to previous states
    MT::Array< RelationalStateGraph* > other_graphs;
    AtomL other_actions;
    if (i > 0) {
      for (k=i-1; k>=0; k--) {
        other_graphs.append(graphs(k));
        other_actions.append(actions(k));
      }
      min_distance_to_others = getMinDistance(*graphs(i), *actions(i), other_graphs, other_actions);
    }
    if (min_distance_to_others < -0.01  ||  min_distance_to_others > 1.01) HALT("bad min_distance_to_others="<<min_distance_to_others);
    entropy += min_distance_to_others;
    if (DEBUG>1) {PRINT(min_distance_to_others);  PRINT(entropy);}
  }
  if (DEBUG>0) {PRINT(entropy);}
  if (DEBUG>0) {cout<<"RelationalStateGraph::entropy [END]"<<endl;}
  return entropy;
}


RelationalStateGraph* RelationalStateGraph::createSubgraph(const TL::Atom& action, const TL::RelationalStateGraph& full_graph, const TL::Rule& rule, uint depth) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RelationalStateGraph::createSubgraph [START]"<<endl;}
  if (DEBUG>0) {PRINT(action);   cout<<"Full graph:"<<endl; full_graph.writeNice();   cout<<"Rule:"<<endl; rule.write();   PRINT(depth);}
  // Calculate local objects [START]
  uintA interesting_objects;
  interesting_objects.setAppend(action.args);
  uintA drefs;
  TL::ruleReasoning::calcGroundDeicticReferences(drefs, full_graph.state, &action, &rule);
  interesting_objects.setAppend(drefs);
  uintA helper;
  full_graph.getRelatedConstants(helper, interesting_objects, depth);
  interesting_objects.setAppend(helper);
  if (DEBUG>0) {PRINT(interesting_objects);}
  // Calculate local objects [END]
  // Get subgraph
  RelationalStateGraph* sub_graph = full_graph.getSubgraph(interesting_objects);
  if (DEBUG>0) {cout<<"sub_graph:"<<endl;  sub_graph->writeNice();}
  if (DEBUG>0) {cout<<"RelationalStateGraph::createSubgraph [END]"<<endl;}
  return sub_graph;
}


double RelationalStateGraph::getMinDistance(const RelationalStateGraph& graph, const TL::Atom& action,
                                                   const MT::Array< RelationalStateGraph* > other_graphs, const AtomL& other_actions) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RelationalStateGraph::getMinDistance [START]"<<endl;}
  if (DEBUG>0) {cout<<"Action:  "<<action<<endl;}
  double min_diff = 1.0;
  uint i;
  FOR1D(other_graphs, i) {
    double distance = RelationalStateGraph::distance(graph, action, *other_graphs(i), *other_actions(i));
    min_diff = TL_MIN(min_diff, distance);
    if (DEBUG>0) {cout<<*other_actions(i)<<"  -> diff="<<distance<<endl;}
    if (TL::isZero(min_diff))
      break;
  }
  if (DEBUG>0) {PRINT(min_diff);}
  if (DEBUG>0) {cout<<"RelationalStateGraph::getMinDistance [END]"<<endl;}
  return min_diff;
}




}
