#include "experiments_fixedContexts.h"
#include <math.h>
#include <MT/algos.h>
#include "TL/ruleReasoning.h"
#include "TL/logicReasoning.h"


namespace TL {
  
bool USING_IPPC_DOMAIN() {
  return TL::logicObjectManager::getPredicate(MT::String("table")) == NULL;
}


// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    R U L E   L E A R N E R  FIXED CONTEXTS
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------


RuleLearner_FixedContexts::RuleLearner_FixedContexts(TL::Atom* action, const MT::Array< LitL >& contexts, double alpha, double p_min, double p_min_noisyDefaultRule) :
      RuleLearner(alpha, p_min, p_min_noisyDefaultRule) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"RuleLearner_FixedContexts [START]"<<endl;}
  this->action = action;
  this->contexts = contexts;
  if (DEBUG>0) {
    cout<<"Action: "<<*this->action<<endl;
    cout<<"Contexts: "<<this->contexts<<endl;
  }
  experiences_per_context.resize(this->contexts.N);
  experienceIds_per_context.resize(this->contexts.N);
  
  uint i;
  FOR1D(searchOperators, i) {
    delete searchOperators(i);
  }
  if (DEBUG>0) {cout<<"RuleLearner_FixedContexts [END]"<<endl;}
}



void RuleLearner_FixedContexts::learn_rules(TL::RuleSetContainer& rulesC, TL::ExperienceA& experiences, const char* logfile) {
  arr experience_weights(experiences.N);
  experience_weights.setUni(1.);
  learn_rules(rulesC, experiences, experience_weights, logfile);
}


void RuleLearner_FixedContexts::learn_rules(TL::RuleSetContainer& rulesC, TL::ExperienceA& experiences, arr& experience_weights, const char* logfile) {
  uint DEBUG = 0; //  2 ist gut
  if (DEBUG>0) {cout<<"RuleLearner_FixedContexts::learn_rules [START]"<<endl;}
  uint i, k;
  if (DEBUG > 0) {
    PRINT(p_min);
    PRINT(p_min_noisyDefaultRule);
    PRINT(experiences.N);
  }
  if (DEBUG > 2) {
    cout << "Expiernces:" << endl;
    FOR1D(experiences, i) {
      cout << "--- [" << i << "] ---" << endl;
      cout<<"PRE-STATE:  "; experiences(i)->pre.write(cout); cout<<endl;
      cout<<"ACTION:     "; experiences(i)->action->write(cout); cout<<endl;
      cout<<"POST-STATE: "; experiences(i)->post.write(cout); cout<<endl;
    }
  }  
  
  rulesC.clear();
  FOR1D(experiences_per_context, i) {
    experiences_per_context(i).clear();
    experienceIds_per_context(i).clear();
  }
  
  FOR1D(experiences, i) {
    TL::logicReasoning::sort(experiences(i)->pre.lits_prim);
  }
  TL::logicReasoning::sort(experiences.last()->post.lits_prim);
  
  
  // assumes there is exactly one covering context
  if (DEBUG>0) {cout<<endl<<"Distributing experiences on contexts..."<<endl;}
  FOR1D(experiences, i) {
    if (DEBUG>1) {cout<<"Experience #"<<i<<":"<<endl;  experiences(i)->write();}
    FOR1D(contexts, k) {
      if (DEBUG>1) {cout<<"Contexts #k="<<k<<":  "<<contexts(k)<<endl;}
      TL::SubstitutionSet subs;
      TL::Rule fake_rule;
      fake_rule.action = action;
      fake_rule.context = contexts(k);
      if (ruleReasoning::cover_rule_groundedAction(experiences(i)->pre, experiences(i)->action, &fake_rule, subs)) {
        if (DEBUG>1) {cout<<"Subs ("<<subs.num()<<"):"<<endl; subs.write();}
        CHECK(subs.num() == 1, "too many subs: subs.num()="<<subs.num());
        experiences_per_context(k).append(experiences(i));
        experienceIds_per_context(k).append(i);
        break;
      }
    }
//     CHECK(k<contexts.N, "Experience not covered by any rule.");
    // ignore uncovered experiences
  }
  
  
  if (DEBUG>0) {cout<<endl<<"Learning rules (i.e., outcomes)..."<<endl;}
  rulesC.init(&experiences);
  // Init default rule
  rulesC.rules.append(TL::ruleReasoning::generateDefaultRule());
  rulesC.recomputeDefaultRule();  
  FOR1D(contexts, i) {
    if (DEBUG>1) {cout<<"Contexts #i="<<i<<":  "<<contexts(i)<<endl;}
    if (DEBUG>1) {PRINT(experiences_per_context(i));}
    TL::Rule* rule = new TL::Rule;
    rule->context = contexts(i);
    rule->action = action;
    MT::Array< uintA > coveredExperiences_per_outcome;
    double pen_sum = 20.;
    double pen_pos = 20.;
    if (experiences_per_context(i).N > 0) {
      SearchOperator::induceOutcomes(rule, coveredExperiences_per_outcome, experiences_per_context(i), experienceIds_per_context(i),
                                   alpha_PEN, p_min, pen_sum, pen_pos, TL::SearchOperator::rprop);
    }
    else {
      rule->outcomes.resize(2);
      rule->probs.resize(2);
      rule->probs(0) = 0.;  rule->probs(1) = 1.;
      coveredExperiences_per_outcome.append(TUP());  coveredExperiences_per_outcome.append(TUP());
    }
    if (DEBUG>0) {rule->write();}
    rulesC.append(rule, experienceIds_per_context(i), coveredExperiences_per_outcome);
  }
  
  if (DEBUG>0) {
    cout<<endl<<"Puh, that was it."<<endl;
    cout<<"Learned rules:"<<endl;
    rulesC.writeNice();
  }
  
  if (DEBUG>0) {cout<<"RuleLearner_FixedContexts::learn_rules [START]"<<endl;}
}



// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------
//    R U L E  EXPLORER  FIXED CONTEXTS
// --------------------------------------------------------------------
// --------------------------------------------------------------------
// --------------------------------------------------------------------



AbstractRuleExplorer_FixedContexts::AbstractRuleExplorer_FixedContexts(const RuleSet& _fixed_partial_rules, double complexity_penalty_coeff,
                                           double p_lower_bound__noise_outcome, double p_lower_bound__noise_outcome_in_default_rule,
                                           TL::RuleSet& fixed_rules_for_fixed_actions, uint _density_estimation_type)
                    : AbstractRuleExplorer(complexity_penalty_coeff, p_lower_bound__noise_outcome, p_lower_bound__noise_outcome_in_default_rule,
                                            fixed_rules_for_fixed_actions, _density_estimation_type) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"AbstractRuleExplorer_FixedContexts [START]"<<endl;}
  uint i, k;
  
  fixed_partial_rules = _fixed_partial_rules;
  if (DEBUG>0) {
    cout<<"fixed_partial_rules:"<<endl;
    fixed_partial_rules.write();
  }
  
  FOR1D(learners, i) {
    delete learners(i);
  }
  learners.clear();
  
  // lerner ersetzen
  FOR1D(modeledActions, i) {
    if (DEBUG>0) {cout<<"Modeled Action #"<<i<<":  "<<*modeledActions(i)<<endl;}
    MT::Array< LitL > contexts;
    FOR1D_(fixed_partial_rules, k) {
      if (fixed_partial_rules.elem(k)->action == modeledActions(i))
        contexts.append(fixed_partial_rules.elem(k)->context);
    }
    if (DEBUG>0) {cout<<contexts.N<<" context(s):"<<endl;  FOR1D(contexts, k) {cout<<contexts(k)<<endl;}}
    // Set up rule-learner for each abstract action
    double alpha_PEN = complexity_penalty_coeff;
    double p_min = p_lower_bound__noise_outcome;
    double p_min_noisyDefaultRule = p_lower_bound__noise_outcome_in_default_rule;
    learners.append(new RuleLearner_FixedContexts(modeledActions(i), contexts, alpha_PEN, p_min, p_min_noisyDefaultRule));
  }
  if (DEBUG>0) {PRINT(learners);  cout<<"modeledActions:  "<<modeledActions<<endl;}
  
  experiences_per_modeledAction.resize(modeledActions.N);
  learners_uptodate.resize(modeledActions.N);
  learners_uptodate.setUni(true);
  
  updateLogicEngineConstants();
  updateRules(true);
  
  if (DEBUG>0) {cout<<"AbstractRuleExplorer_FixedContexts [END]"<<endl;}
}




bool AbstractRuleExplorer_FixedContexts::actionIsKnown(const TL::State& state, TL::Atom* action) {
  uint DEBUG = 0;
  if (DEBUG>0) {cout<<"AbstractRuleExplorer_FixedContexts::actionIsKnown [START]"<<endl;}
  if (DEBUG>0) {cout<<"State:"<<endl<<state<<"Action:  "<<*action<<endl;}
  int id = possibleGroundActions.findValue(action);
  CHECK(id >= 0  &&  id < (int) possibleGroundActions.N, "");
  if (DEBUG>0) {PRINT(actions__confidences(id));}
  // Action unknown: if covered by default rule AND state of target rule is fulfilled
  if (actions__confidences(id) < RULE_CONFIDENCE_THRESHOLD) {
    uint i;
    FOR1D_(fixed_partial_rules, i) {
      TL::SubstitutionSet subs;
      if (TL::ruleReasoning::cover_rule_groundedAction(state, action, fixed_partial_rules.elem(i), subs)) {
        if (DEBUG>0) {cout<<"Covering rule:"<<endl<<*fixed_partial_rules.elem(i)<<endl;}
        if (DEBUG>0) {cout<<"Return false"<<endl<<"AbstractRuleExplorer_FixedContexts::actionIsKnown [END]"<<endl;}
        return false;
      }
    }
  }
  if (DEBUG>0) {cout<<"Return true"<<endl<<"AbstractRuleExplorer_FixedContexts::actionIsKnown [END]"<<endl;}
  return true;
}



// ------------------------------------
//  Reward for Unknown Contexts
// ------------------------------------

double evaluate_prada_reward__specific_contexts(const MT::Array< LitL >& ground_contexts, const NID_DBN& net, uint t, uint depth, uint verbosity = 0) {
  uint DEBUG = 0;
  if (DEBUG<verbosity)
    DEBUG = verbosity;
  if (t>depth)
    return 0.0;
  uint i, k;
  double reward = 0.;
  FOR1D(ground_contexts, i) {
    if (DEBUG>2) {cout<<"i="<<i<<"  "<<ground_contexts(i)<<endl;}
    double prob = 1.;
    FOR1D(ground_contexts(i), k) {
      if (DEBUG>2) {cout<<"k="<<k<<"  "<<*ground_contexts(i)(k)<<endl;}
      CHECK(ground_contexts(i)(k)->atom->pred->type != TL::Predicate::predicate_comparison, "Here, no comparisons allowed in ground_contexts.");
      PredicateRV* v = net.rvm->l2v(ground_contexts(i)(k));
      if (v != NULL) {
        if (ground_contexts(i)(k)->positive)
          prob *= v->P(t,1);
        else
          prob *= v->P(t,0);
      }
      else {
        // ignoriere unbekannte
//         prob *= 0.;
      }
    }
    reward += prob;
//     if (DEBUG>0) {PRINT(reward);}
  }
  if (DEBUG>0) {cout<<"t="<<t<<":  ";}
  if (DEBUG>0) {cout<<endl;  PRINT(reward);}

  return reward;
}



class PRADA_Reward__Specific_Contexts : public PRADA_Reward {
  
  protected:
    uint depth;
    MT::Array< LitL > ground_contexts;
    
  public :
    
    PRADA_Reward__Specific_Contexts(MT::Array< LitL > _ground_contexts, uint _depth) {
      ground_contexts = _ground_contexts;
      depth = _depth;
    }
    
    // Random variables --> Reals
    double evaluate_prada_reward(const NID_DBN& net, uint t) {
      return evaluate_prada_reward__specific_contexts(ground_contexts, net, t, depth, 0);
    }
    
    double evaluate_prada_reward(const NID_DBN& net, uint t, uint verbosity) {
      return evaluate_prada_reward__specific_contexts(ground_contexts, net, t, depth, verbosity);
    }
};


// ------------------------------------
//  Reward for specific R-max instance
// ------------------------------------

class PRADA_Reward__Rmax__Specific_Contexts : public PRADA_Reward__Specific_Contexts {
  
  PRADA_Reward* exploit_PRADA_reward;
  
  double evaluate_prada_reward__rmax(const NID_DBN& net, uint t, uint verbosity = 0) {
    uint DEBUG = 0;
    double reward_value_exploit = exploit_PRADA_reward->evaluate_prada_reward(net, t);
    double reward_value_explore = evaluate_prada_reward__specific_contexts(ground_contexts, net, t, depth, verbosity);
    double reward_value = reward_value_exploit + reward_value_explore;
    if (DEBUG>0) {PRINT(reward_value_exploit);  PRINT(reward_value_explore);  PRINT(reward_value);}
    return reward_value;
  }
  
  public :
    
    PRADA_Reward__Rmax__Specific_Contexts(TL::Reward* exploit_reward, MT::Array< LitL > _ground_contexts, uint _depth)
            : PRADA_Reward__Specific_Contexts(_ground_contexts, _depth) {
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





TL::Atom* AbstractRuleExplorer_FixedContexts::decideAction(const TL::State& state, NID_Planner* planner, uint behavior_type, bool use_known_state_partial) {
  uint DEBUG = 2;
  #ifdef ENFORCED_SILENCE
  DEBUG = 0;
  #endif
  if (DEBUG>0) {cout<<"AbstractRuleExplorer_FixedContexts::decideAction [START]"<<endl;}
  uint i, k;
  double t_start, t_finish;
  
  current_state = state;
  if (confident_ground_rules.num() == 0)  // This is required if world has changed. Then ground rules need to be recalculated.
    updateRules();
  
  message.clr();
//   message << "[" << visited_actions.N << "]  ";
  
  if (DEBUG>0) {PRINT(fixed_partial_rules.num());  PRINT(rulesC.rules.num());}
  
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
  
  
  // (4) Fixed-context specialty:  calculate actions with effect for this state
  AtomL actions_with_effect;
  uintA actions_with_effect__ids;
  FOR1D(possibleGroundActions, i) {
    FOR1D_(fixed_partial_rules, k) {
      TL::SubstitutionSet subs;
      if (TL::ruleReasoning::cover_rule_groundedAction(state, possibleGroundActions(i), fixed_partial_rules.elem(k), subs)) {
        actions_with_effect.append(possibleGroundActions(i));
        actions_with_effect__ids.append(i);
        break;
      }
    }
  }
  if (DEBUG>0) {PRINT(actions_with_effect);  PRINT(actions_with_effect__ids);}
  
  bool fixedContext__current_state_is_known = true;
  FOR1D(action__is_known, i) {
    if (!action__is_known(i)) {
      fixedContext__current_state_is_known = false;
      break;
    }
  }
  if (DEBUG>0) {PRINT(fixedContext__current_state_is_known);}
  
  AtomL unknown_actions;
  FOR1D(possibleGroundActions, i) {
    if (!action__is_known(i))
      unknown_actions.append(possibleGroundActions(i));
  }
  if (DEBUG>0) {PRINT(unknown_actions);}
  
  
  
   // Collect unknown contexts
  TL::RuleSet non_confident_rules;
  TL::RuleSet confident_rules;
  CHECK(rules__confidences. N == rulesC.rules.num(), "");
  FOR1D_(fixed_partial_rules, i) {
    if (TL::ruleReasoning::isDefaultRule(fixed_partial_rules.elem(i)))
      continue;
    bool confident = false;
    FOR1D_(rulesC.rules, k) {
      if (rulesC.rules.elem(k)->context == fixed_partial_rules.elem(i)->context
          &&   rules__confidences(k) >= RULE_CONFIDENCE_THRESHOLD) {
        confident = true;
        break;
      }
    }
    if (confident) confident_rules.append(rulesC.rules.elem(k));
    else non_confident_rules.append(fixed_partial_rules.elem(i));
  }
  if (DEBUG>0) {
    cout<<"CONFIDENT RULES:"<<endl;
    confident_rules.write();
    cout<<"NON-CONFIDENT RULES:"<<endl;
    non_confident_rules.write();
  }
  
  
  // Save the literals in the unknown contexts for the reward
  TL::RuleSet non_confident_ground_rules;
  TL::ruleReasoning::ground(non_confident_ground_rules, non_confident_rules, logicObjectManager::constants);
  LitL grounded_unknown_context_literals;
  MT::Array< LitL > grounded_unknown_contexts;
  FOR1D_(non_confident_ground_rules, i) {
    grounded_unknown_contexts.append(non_confident_ground_rules.elem(i)->context);
    grounded_unknown_context_literals.setAppend(non_confident_ground_rules.elem(i)->context);
  }
  if (DEBUG>0) {PRINT(grounded_unknown_context_literals);}
  
  if (DEBUG>0) {
    uintA changingIds_preds, changingIds_funcs;
    ruleReasoning::changingConcepts(changingIds_preds, changingIds_funcs, confident_rules);
    cout<<"Abstract unknown contexts:"<<endl;
    FOR1D_(non_confident_rules, i) {
      cout<<"i="<<i<<":  "<<non_confident_rules.elem(i)->context<<endl;
      uint l;
      FOR1D(non_confident_rules.elem(i)->context, l) {
        TL::Literal* lit = non_confident_rules.elem(i)->context(l);
        if (changingIds_preds.findValue(lit->atom->pred->id) < 0) {
          cout<<"In rule #i="<<i<<": literal #l="<<l<<" "<<*lit<<" cannot be achieved."<<endl;
        }
      }
      if (non_confident_rules.elem(i)->context.N > l)
        cout << "We cannot plan for context of rule #i="<<i<<"!"<<endl;
    }
    cout<<"grounded_unknown_contexts:"<<endl;
    FOR1D(grounded_unknown_contexts, i) {
      cout<<"i="<<i<<":  "<<grounded_unknown_contexts(i);
      if (TL::logicReasoning::holds(state, grounded_unknown_contexts(i))) {
        cout<<"  --> 1"<<endl;  // can easily happen!  rule with effect, but uncertain outcomes!
      }
      else
        cout<<"  --> 0"<<endl;
    }
  }

  
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
        action = actions_with_effect(rnd.num(actions_with_effect.N));
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
    if (DEBUG>0) {cout<<"E^3"<<endl;}
    TL::Atom* bad_action = NULL;
    //   (E^3)  (1) STATE KNOWN --> PLAN
    if (fixedContext__current_state_is_known) {
      if (DEBUG>0) {cout<<"*** STATE IS KNOWN --> PLAN. ***"<<endl;  cerr<<"*** STATE IS KNOWN --> PLAN. ***"<<endl;}
      //   (E^3)  (1.1)  TRY TO EXPLOIT
      if (DEBUG>0) {cout<<"*** EXPLOIT TRY. ***"<<endl;   cerr<<"*** EXPLOIT TRY. ***"<<endl;}
      // (1) TRY EXPLOIT
      AtomL exploit_plan;
      if (USING_IPPC_DOMAIN()) {
        // Special heuristic for IPPC domains [START]
        MT_MSG("special heuristic for EXPLOITING");
        // check whether we have tried to exploit in this state already while the state has not changed since
        uint q;
        bool have_exploited_already = false;
        FOR1D_DOWN(visited_pre_states, q) {
          if (visited_pre_states.N != moves.N) HALT("visited_pre_states.N="<<visited_pre_states.N<<"  vs.  moves.N="<<moves.N);
          if (*visited_pre_states(q) == current_state) {
            if (moves(q) == MOVE_TYPE__EXPLOIT) {
              cout<<"we have exploited in visited_pre_states #q="<<q<<endl;
              have_exploited_already = true;
              break;
            }
          }
          else
            break;
        }
        cout<<"State was the same onwards from state q+1 = "<<(q+1)<<endl;
        PRINT(have_exploited_already);
        if (have_exploited_already  &&  !is_major_experience.last()) {
          MT_MSG("HOPELESS EXPLOITING: (i) current state equals previous state and (ii) no major insight from last action");
          cout<<"HOPELESS EXPLOITING: (i) current state equals previous state and (ii) no major insight from last action"<<endl;
          MT_MSG("in relational case:  makes no sense to exploit if not for all actions a manipulating rule");
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
      else {
        if (DEBUG>0) {cout<<"*** PLANNED EXPLORE TRY. ***"<<endl;   cerr<<"*** PLANNED EXPLORE TRY. ***"<<endl;}
        if (DEBUG>0) {cout<<"(Plan towards specific unknown contexts.)"<<endl;}
        AtomL explore_plan;
                
        delete ((TL::PRADA*) planner)->net;
        ((TL::PRADA*) planner)->net = NULL;
        
        MaximizeFunctionReward fake_reward;
        fake_reward.important_literals = grounded_unknown_context_literals;
        PRADA_Reward__Specific_Contexts* planned_explore_reward = new PRADA_Reward__Specific_Contexts(grounded_unknown_contexts, PLANNED_EXPLORE__HORIZON);
        ((TL::PRADA*) planner)->setReward(&fake_reward, planned_explore_reward);
        ((TL::PRADA*) planner)->setHorizon(PLANNED_EXPLORE__HORIZON+1);
        ((TL::PRADA*) planner)->setNumberOfSamples(PLANNED_EXPLORE__NUMBER_PRADA_SAMPLES);
        ((TL::PRADA*) planner)->setThresholdReward(0.05);
        
        double value;
        ((TL::PRADA*) planner)->generatePlan(explore_plan, value, state, 1);
        
        action = NULL;
        if (explore_plan.N > 0  && value > PLANNED_EXPLORE__THRESHOLD) {
          action = explore_plan(0);
          explore_plan.memMove = true;
          explore_plan.remove(explore_plan.N-1);
        }
        
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
        if (!fixedContext__current_state_is_known) {cout<<"*** STATE IS UNKNOWN *** "<<endl;  cerr<<"*** STATE IS UNKNOWN *** "<<endl;}
        cout << "*** DIRECT EXPLORE ACTION. ***"<<endl;   cerr<<"*** DIRECT EXPLORE ACTION. ***"<<endl;
      }
      if (unknown_actions.N == 0) {MT_MSG("++++++ NO UNKNOWN ACTIONS!!!!! +++++++");  cout<<"++++++ NO UNKNOWN ACTIONS!!!!! +++++++"<<endl;}
      double min_conf = 1000.;
      uint min_conf_id = 100000;
      FOR1D(actions_with_effect__ids, i) {
        if (actions__confidences(actions_with_effect__ids(i)) < min_conf) {
          min_conf = actions__confidences(actions_with_effect__ids(i));
          min_conf_id = i;
        }
      }
      // action with least confidence
      uintA min_conf_actions;
      FOR1D(actions_with_effect__ids, i) {
        if (TL::areEqual(actions__confidences(actions_with_effect__ids(i)), min_conf))
          min_conf_actions.append(actions_with_effect__ids(i));
      }
      if (DEBUG>0) PRINT(min_conf_actions);
      action = possibleGroundActions(min_conf_actions(rnd.num(min_conf_actions.N)));
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
    MaximizeFunctionReward fake_reward;
    fake_reward.important_literals = grounded_unknown_context_literals;
    PRADA_Reward__Specific_Contexts* planned_explore_reward = new PRADA_Reward__Rmax__Specific_Contexts(exploit_reward, grounded_unknown_contexts, PLANNED_EXPLORE__HORIZON);
    ((TL::PRADA*) planner)->setReward(&fake_reward, planned_explore_reward);
    
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
      cout<<"No action found in R-max planning; do simple direct explore step  (#rules="<<getRules().num()<<")."<<endl;
      
      double min_conf = 1000.;
      uint min_conf_id = 100000;
      FOR1D(actions_with_effect__ids, i) {
        if (actions__confidences(actions_with_effect__ids(i)) < min_conf) {
          min_conf = actions__confidences(actions_with_effect__ids(i));
          min_conf_id = i;
        }
      }
      // action with least confidence
      uintA min_conf_actions;
      FOR1D(actions_with_effect__ids, i) {
        if (TL::areEqual(actions__confidences(actions_with_effect__ids(i)), min_conf))
          min_conf_actions.append(actions_with_effect__ids(i));
      }
      if (DEBUG>0) PRINT(min_conf_actions);
      action = possibleGroundActions(min_conf_actions(rnd.num(min_conf_actions.N)));
      moves_explore_direct.append(visited_pre_states.N);
      moves.append(MOVE_TYPE__EXPLORE_DIRECT);
    }
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
  if (DEBUG>0) {cout<<"AbstractRuleExplorer_FixedContexts::decideAction [END]"<<endl;}
  
  if (possibleGroundActions.findValue(action) < 0) {
    cerr<<"Unknown action decided for:  "<<*action<<endl;
  }
  
  return action;
}



}
