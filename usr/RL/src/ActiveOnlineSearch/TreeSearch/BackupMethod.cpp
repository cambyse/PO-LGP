#include "BackupMethod.h"
#include "PriorModels.h"

#include <util/QtUtil.h>

#include <float.h>
#include <limits>
#include <algorithm>
#include <vector>
#include <unordered_map>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using lemon::INVALID;
using std::vector;

namespace backup_method {

    void BackupMethod::init(double disc,
                            std::shared_ptr<AbstractEnvironment> env,
                            const graph_t & g,
                            const node_info_map_t & ni_map,
                            const mcts_arc_info_map_t & mcts_ai_map) {
        discount = disc;
        environment = env;
        graph = &g;
        node_info_map = &ni_map;
        mcts_arc_info_map = &mcts_ai_map;
    }

    Bellman::Bellman(std::shared_ptr<const tree_policy::TreePolicy> tree_policy,
                     double prior_counts):
        tree_policy(tree_policy),
        prior_counts(prior_counts)
    {}

    void Bellman::init(double disc,
                       std::shared_ptr<AbstractEnvironment> env,
                       const graph_t & g,
                       const node_info_map_t & ni_map,
                       const mcts_arc_info_map_t & mcts_ai_map) {
        BackupMethod::init(disc,env,g,ni_map,mcts_ai_map);
        if(prior_counts<0) {
            if(environment->has_min_reward() && environment->has_max_reward()) {
                prior_counts = 1;
            } else {
                prior_counts = 0;
            }
        } else if(prior_counts>0) {
            if(!environment->has_min_reward() || !environment->has_max_reward()) {
                DEBUG_ERROR("Cannot use prior counts without min/max reward");
            }
        }
    }

    void Bellman::backup(const node_t & observation_node,
                         const node_t & action_node,
                         mcts_node_info_map_t & mcts_node_info_map) const {
        DEBUG_EXPECT(0,environment!=nullptr);

        // compute action value
        {
            DEBUG_OUT(1,"Backup action node " << graph->id(action_node));
            // compute imediate reward (mean and variance)
            arc_t to_action_arc = in_arc_it_t(*graph,action_node);
            int action_transition_counts = (*mcts_arc_info_map)[to_action_arc].transition_counts;
            reward_t min_reward = 0;
            reward_t max_reward = 0;
            if(prior_counts>0) {
                min_reward = environment->min_reward();
                max_reward = environment->max_reward();
            }
            prior_models::PriorCounts mean_and_variance((*mcts_arc_info_map)[to_action_arc].reward_sum,
                                                        (*mcts_arc_info_map)[to_action_arc].squared_reward_sum,
                                                        action_transition_counts,
                                                        min_reward,
                                                        max_reward,
                                                        prior_counts);
            reward_t mean_reward = mean_and_variance.mean;
            reward_t mean_reward_variance = mean_and_variance.variance;
            DEBUG_OUT(2,"    ^r=" << mean_reward << ", ~r=" << mean_reward_variance);
            // compute value (mean and variance)
            reward_t action_value = mean_reward;
            reward_t action_value_variance = mean_reward_variance;
            reward_t min_action_value = 0;
            reward_t max_action_value = 0;
            for(out_arc_it_t to_state_arc_1(*graph, action_node); to_state_arc_1!=INVALID; ++to_state_arc_1) {

                node_t target_state_node_1 = graph->target(to_state_arc_1);
                double prob_1 = (double)(*mcts_arc_info_map)[to_state_arc_1].transition_counts/action_transition_counts;
                double prob_variance_1 = prob_1*(1-prob_1)/(action_transition_counts+1);
                reward_t state_value_1 = mcts_node_info_map[target_state_node_1].value;
                reward_t state_value_variance_1 = mcts_node_info_map[target_state_node_1].value_variance;
                DEBUG_OUT(2,"    transition to state node " << graph->id(target_state_node_1));
                DEBUG_OUT(2,"        ^p=" << prob_1 << ", ~p=" << prob_variance_1);
                DEBUG_OUT(2,"        ^V=" << state_value_1 << ", ~V=" << state_value_variance_1);

                // update action value
                action_value += discount*prob_1*state_value_1;

                // update action value variance (first sum)
                action_value_variance += discount*discount*
                    (prob_1*prob_1 + prob_variance_1)*
                    state_value_variance_1;

                // update action value bounds
                min_action_value += discount*prob_1*mcts_node_info_map[target_state_node_1].min_value;
                max_action_value += discount*prob_1*mcts_node_info_map[target_state_node_1].max_value;

                // update action value variance (second sum)
                for(out_arc_it_t to_state_arc_2(*graph, action_node); to_state_arc_2!=INVALID; ++to_state_arc_2) {

                    node_t target_state_node_2 = graph->target(to_state_arc_2);
                    double prob_2 = (double)(*mcts_arc_info_map)[to_state_arc_2].transition_counts/action_transition_counts;
                    reward_t state_value_2 = mcts_node_info_map[target_state_node_2].value;

                    double prob_covariance = target_state_node_1==target_state_node_2?
                        prob_2*(1-prob_2)/(action_transition_counts+1):
                        -prob_1*prob_2/(action_transition_counts+1);
                    DEBUG_OUT(2,"        transition-pair to state nodes " << graph->id(target_state_node_1) << "/" << graph->id(target_state_node_2));
                    DEBUG_OUT(2,"        ^p=" << prob_1 << "/" << prob_2 << ", ~p=" << prob_covariance);
                    DEBUG_OUT(2,"        ^V=" << state_value_1 << "/" << state_value_2);

                    action_value_variance += discount*discount*
                        prob_covariance*
                        state_value_1*state_value_2;
                }
            }
            mcts_node_info_map[action_node].set_value(action_value, action_value_variance,min_action_value,max_action_value);
            DEBUG_OUT(1,"Assign ^Q=" << action_value << ", ~Q=" << action_value_variance);
        }

        // compute state value
        DEBUG_OUT(1,"Backup observation node " << graph->id(observation_node));
        if(tree_policy!=nullptr) {
            // get an action from the tree policy
            action_handle_t action = tree_policy->get_action(observation_node);
            // the tree-policy may choose an action that was not sampled yet, so
            // we try to find the corresponding action node and take average
            // over all available actions otheriwse (random policy)
            vector<node_t> actions_to_use;
            for(out_arc_it_t to_action_arc(*graph, observation_node); to_action_arc!=INVALID; ++to_action_arc) {
                node_t action_node = graph->target(to_action_arc);
                if((*node_info_map)[action_node].action==action) {
                    actions_to_use.assign(1,action_node);
                    break;
                } else {
                    actions_to_use.push_back(action_node);
                }
            }
            reward_t value = 0;
            reward_t value_variance = 0;
            reward_t min_value = 0;
            reward_t max_value = 0;
            for(node_t action_node : actions_to_use) {
                value += mcts_node_info_map[action_node].value;
                value_variance += mcts_node_info_map[action_node].value_variance;
                min_value += mcts_node_info_map[action_node].min_value;
                max_value += mcts_node_info_map[action_node].max_value;
            }
            value /= actions_to_use.size();
            value_variance /= actions_to_use.size();
            min_value /= actions_to_use.size();
            max_value /= actions_to_use.size();
            // assign
            mcts_node_info_map[observation_node].set_value(value, value_variance,min_value,max_value);
        } else {
            // find maximum value and set of max-value actions
            reward_t max = std::numeric_limits<reward_t>::lowest();
            vector<node_t> max_value_actions;
            for(out_arc_it_t to_action_arc(*graph, observation_node); to_action_arc!=INVALID; ++to_action_arc) {
                node_t action_node = graph->target(to_action_arc);
                reward_t value = mcts_node_info_map[action_node].value;
                if(value>max) {
                    max_value_actions.clear();
                }
                if(value>=max) {
                    max = value;
                    max_value_actions.push_back(action_node);
                }
            }

            DEBUG_EXPECT(1,max_value_actions.size()>0);

            // compute variance
            reward_t value_variance = 0;
            reward_t min_value = 0;
            reward_t max_value = 0;
            for(node_t action_node : max_value_actions) {
                value_variance += mcts_node_info_map[action_node].value_variance;
                min_value += mcts_node_info_map[action_node].min_value;
                max_value += mcts_node_info_map[action_node].max_value;
            }
            value_variance /= max_value_actions.size();
            min_value /= max_value_actions.size();
            max_value /= max_value_actions.size();

            // assign
            mcts_node_info_map[observation_node].set_value(max, value_variance,min_value,max_value);
        }
    }

    MonteCarlo::MonteCarlo(double prior_counts):
        prior_counts(prior_counts)
    {}

    void MonteCarlo::init(double disc,
                          std::shared_ptr<AbstractEnvironment> env,
                          const graph_t & g,
                          const node_info_map_t & ni_map,
                          const mcts_arc_info_map_t & mcts_ai_map) {
        BackupMethod::init(disc,env,g,ni_map,mcts_ai_map);
        if(prior_counts<0) {
            if(discount<1 && environment->has_min_reward() && environment->has_max_reward()) {
                prior_counts = 1;
            } else {
                prior_counts = 0;
            }
        } else if(prior_counts>0) {
            if(!environment->has_min_reward() || !environment->has_max_reward()) {
                DEBUG_ERROR("Cannot use prior counts without min/max reward");
            }
            if(!(discount<1)) {
                DEBUG_ERROR("Discount must be smaller than one to use prior counts");
            }
        }
    }

    void MonteCarlo::backup(const node_t & observation_node,
                            const node_t & action_node,
                            mcts_node_info_map_t & mcts_node_info_map) const {
        // compute action and state value and variance
        for(auto * info : {&mcts_node_info_map[action_node], &mcts_node_info_map[observation_node]}) {
            reward_t min_return = 0;
            reward_t max_return = 0;
            if(prior_counts>0) {
                min_return = environment->min_reward()/(1-discount);
                max_return = environment->max_reward()/(1-discount);
            }
            prior_models::PriorCounts mean_and_variance(info->return_sum,
                                                        info->squared_return_sum,
                                                        info->rollout_counts,
                                                        min_return,
                                                        max_return,
                                                        prior_counts);
            // assign
            info->set_value(mean_and_variance.mean,
                            mean_and_variance.variance,
                            info->min_return,
                            info->max_return);
        }
        DEBUG_OUT(1,QString("    backup observ.-node(%1):	^V=%2	~V=%3	V+/-=%4/%5").
                  arg(graph->id(observation_node)).
                  arg(mcts_node_info_map[observation_node].value).
                  arg(mcts_node_info_map[observation_node].value_variance).
                  arg(mcts_node_info_map[observation_node].min_value).
                  arg(mcts_node_info_map[observation_node].max_value));
        DEBUG_OUT(1,QString("    backup action-node(%1):	^V=%2	~V=%3	V+/-=%4/%5").
                  arg(graph->id(action_node)).
                  arg(mcts_node_info_map[action_node].value).
                  arg(mcts_node_info_map[action_node].value_variance).
                  arg(mcts_node_info_map[action_node].min_value).
                  arg(mcts_node_info_map[action_node].max_value));
    }

    HybridMCDP::HybridMCDP(double mc_weight,
                           double reward_prior_counts,
                           double return_prior_counts):
        mc_weight(mc_weight),
        monte_carlo(return_prior_counts),
        bellman(nullptr,reward_prior_counts) {
        if(mc_weight>1) {
            DEBUG_ERROR("Weight must be in [0,1] but is " << mc_weight);
            mc_weight = 1;
        }
        if(mc_weight<0) {
            DEBUG_ERROR("Weight must be in [0,1] but is " << mc_weight);
            mc_weight = 0;
        }
    }

    void HybridMCDP::init(double discount,
                          std::shared_ptr<AbstractEnvironment> environment,
                          const graph_t & graph,
                          const node_info_map_t & node_info_map,
                          const mcts_arc_info_map_t & mcts_arc_info_map) {
        BackupMethod::init(discount,environment,graph,node_info_map,mcts_arc_info_map);
        monte_carlo.init(discount,environment,graph,node_info_map,mcts_arc_info_map);
        bellman.init(discount,environment,graph,node_info_map,mcts_arc_info_map);
    }

    void HybridMCDP::backup(const node_t & observation_node,
                            const node_t & action_node,
                            mcts_node_info_map_t & mcts_node_info_map) const {
        bellman.backup(observation_node,action_node,mcts_node_info_map);
        auto bellman_observation_value = mcts_node_info_map[observation_node].value;
        auto bellman_observation_value_variance = mcts_node_info_map[observation_node].value_variance;
        auto bellman_min_observation_value = mcts_node_info_map[observation_node].min_value;
        auto bellman_max_observation_value = mcts_node_info_map[observation_node].max_value;
        auto bellman_action_value = mcts_node_info_map[action_node].value;
        auto bellman_action_value_variance = mcts_node_info_map[action_node].value_variance;
        auto bellman_min_action_value = mcts_node_info_map[action_node].min_value;
        auto bellman_max_action_value = mcts_node_info_map[action_node].max_value;
        monte_carlo.backup(observation_node,action_node,mcts_node_info_map);
        auto monte_carlo_observation_value = mcts_node_info_map[observation_node].value;
        auto monte_carlo_observation_value_variance = mcts_node_info_map[observation_node].value_variance;
        auto monte_carlo_min_observation_value = mcts_node_info_map[observation_node].min_value;
        auto monte_carlo_max_observation_value = mcts_node_info_map[observation_node].max_value;
        auto monte_carlo_action_value = mcts_node_info_map[action_node].value;
        auto monte_carlo_action_value_variance = mcts_node_info_map[action_node].value_variance;
        auto monte_carlo_min_action_value = mcts_node_info_map[action_node].min_value;
        auto monte_carlo_max_action_value = mcts_node_info_map[action_node].max_value;

        mcts_node_info_map[observation_node].value          = mc_weight*monte_carlo_observation_value          + (1-mc_weight)*bellman_observation_value;
        mcts_node_info_map[observation_node].value_variance = mc_weight*monte_carlo_observation_value_variance + (1-mc_weight)*bellman_observation_value_variance;
        mcts_node_info_map[observation_node].min_value      = mc_weight*monte_carlo_min_observation_value      + (1-mc_weight)*bellman_min_observation_value;
        mcts_node_info_map[observation_node].max_value      = mc_weight*monte_carlo_max_observation_value      + (1-mc_weight)*bellman_max_observation_value;
        mcts_node_info_map[action_node     ].value          = mc_weight*monte_carlo_action_value               + (1-mc_weight)*bellman_action_value;
        mcts_node_info_map[action_node     ].value_variance = mc_weight*monte_carlo_action_value_variance      + (1-mc_weight)*bellman_action_value_variance;
        mcts_node_info_map[action_node     ].min_value      = mc_weight*monte_carlo_min_action_value           + (1-mc_weight)*bellman_min_action_value;
        mcts_node_info_map[action_node     ].max_value      = mc_weight*monte_carlo_max_action_value           + (1-mc_weight)*bellman_max_action_value;
    }

} // namespace backup_method {
