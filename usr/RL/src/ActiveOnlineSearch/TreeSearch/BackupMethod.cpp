#include "BackupMethod.h"

#include <util/QtUtil.h>

#include <float.h>
#include <limits>
#include <algorithm>
#include <vector>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using lemon::INVALID;
using std::vector;

namespace backup_method {

    Bellman::Bellman(std::shared_ptr<const tree_policy::TreePolicy> tree_policy):
        tree_policy(tree_policy)
    {}

    void Bellman::operator()(const node_t & state_node,
                             const node_t & action_node,
                             double discount,
                             std::shared_ptr<AbstractEnvironment> environment,
                             const graph_t & graph,
                             const node_info_map_t & node_info_map,
                             mcts_node_info_map_t & mcts_node_info_map,
                             const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action value
        {
            DEBUG_OUT(1,"Backup action node " << graph.id(action_node));
            arc_t to_action_arc = in_arc_it_t(graph,action_node);
            int action_counts = mcts_arc_info_map[to_action_arc].get_counts();
            reward_t mean_reward = mcts_arc_info_map[to_action_arc].get_reward_sum()/action_counts;
            reward_t mean_squared_reward = mcts_arc_info_map[to_action_arc].get_squared_reward_sum()/action_counts;
            reward_t mean_reward_variance;
            if(action_counts<2) {
                mean_reward_variance = std::numeric_limits<double>::infinity();
            } else {
                mean_reward_variance = (mean_squared_reward - mean_reward*mean_reward)/(action_counts-1);
            }
            DEBUG_OUT(2,"    ^r=" << mean_reward << ", ~r=" << mean_reward_variance);
            reward_t action_value = mean_reward;
            reward_t action_value_variance = mean_reward_variance;
            for(out_arc_it_t to_state_arc_1(graph, action_node); to_state_arc_1!=INVALID; ++to_state_arc_1) {

                node_t target_state_node_1 = graph.target(to_state_arc_1);
                double prob_1 = mcts_arc_info_map[to_state_arc_1].get_counts()/action_counts;
                double prob_variance_1 = prob_1*(1-prob_1)/(action_counts+1);
                reward_t state_value_1 = mcts_node_info_map[target_state_node_1].get_value();
                reward_t state_value_variance_1 = mcts_node_info_map[target_state_node_1].get_value_variance();
                DEBUG_OUT(2,"    transition to state node " << graph.id(target_state_node_1));
                DEBUG_OUT(2,"        ^p=" << prob_1 << ", ~p=" << prob_variance_1);
                DEBUG_OUT(2,"        ^V=" << state_value_1 << ", ~V=" << state_value_variance_1);

                // update action value
                action_value += discount*prob_1*state_value_1;

                // update action value variance (first sum)
                action_value_variance += discount*discount*
                    (prob_1*prob_1 + prob_variance_1)*
                    state_value_variance_1;

                // update action value variance (second sum)
                for(out_arc_it_t to_state_arc_2(graph, action_node); to_state_arc_2!=INVALID; ++to_state_arc_2) {

                    node_t target_state_node_2 = graph.target(to_state_arc_2);
                    double prob_2 = mcts_arc_info_map[to_state_arc_2].get_counts()/action_counts;
                    reward_t state_value_2 = mcts_node_info_map[target_state_node_2].get_value();

                    double prob_covariance = target_state_node_1==target_state_node_2?
                        prob_2*(1-prob_2)/(action_counts+1):
                        -prob_1*prob_2/(action_counts+1);
                    DEBUG_OUT(2,"        transition-pair to state nodes " << graph.id(target_state_node_1) << "/" << graph.id(target_state_node_2));
                    DEBUG_OUT(2,"        ^p=" << prob_1 << "/" << prob_2 << ", ~p=" << prob_covariance);
                    DEBUG_OUT(2,"        ^V=" << state_value_1 << "/" << state_value_2);

                    action_value_variance += discount*discount*
                        prob_covariance*
                        state_value_1*state_value_2;
                }
            }
            mcts_node_info_map[action_node].set_value(action_value, action_value_variance);
            DEBUG_OUT(1,"Assign ^Q=" << action_value << ", ~Q=" << action_value_variance);
        }

        // compute state value
        if(tree_policy!=nullptr) {
            // get an action from the tree policy
            action_handle_t action = (*tree_policy)(state_node,
                                                    environment,
                                                    graph,
                                                    node_info_map,
                                                    mcts_node_info_map,
                                                    mcts_arc_info_map);
            // find corresponding action node or take averave over available
            // actions
            vector<node_t> actions_to_use;
            for(out_arc_it_t to_action_arc(graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
                node_t action_node = graph.target(to_action_arc);
                if(node_info_map[action_node].action==action) {
                    actions_to_use.assign(1,action_node);
                    break;
                } else {
                    actions_to_use.push_back(action_node);
                }
            }
            reward_t value = 0;
            reward_t value_variance = 0;
            for(node_t action_node : actions_to_use) {
                value += mcts_node_info_map[action_node].get_value();
                value_variance += mcts_node_info_map[action_node].get_value_variance();
            }
            value /= actions_to_use.size();
            value_variance /= actions_to_use.size();
            // assign
            mcts_node_info_map[state_node].set_value(value, value_variance);
        } else {
            // find maximum value and set of max-value actions
            reward_t max_value = -DBL_MAX;
            vector<node_t> max_value_actions;
            for(out_arc_it_t to_action_arc(graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
                node_t action_node = graph.target(to_action_arc);
                reward_t value = mcts_node_info_map[action_node].get_value();
                if(value>max_value) {
                    max_value_actions.clear();
                }
                if(value>=max_value) {
                    max_value = value;
                    max_value_actions.push_back(action_node);
                }
            }

            DEBUG_EXPECT(1,max_value_actions.size()>0);

            // compute variance
            reward_t value_variance = 0;
            for(node_t action_node : max_value_actions) {
                value_variance += mcts_node_info_map[action_node].get_value_variance();
            }
            value_variance /= max_value_actions.size();

            // assign
            mcts_node_info_map[state_node].set_value(max_value, value_variance);
        }
    }

    void MonteCarlo::operator()(const node_t & state_node,
                                const node_t & action_node,
                                double discount,
                                std::shared_ptr<AbstractEnvironment> environment,
                                const graph_t & graph,
                                const node_info_map_t & node_info_map,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action and state value and variance
        for(auto * info : {&mcts_node_info_map[action_node], &mcts_node_info_map[state_node]}) {
            int counts = info->get_rollout_counts();
            reward_t expected_return = info->get_return_sum()/counts;
            reward_t return_variance;
            if(counts<2) {
                return_variance = std::numeric_limits<double>::infinity();
            } else {
                return_variance = (info->get_squared_return_sum()/counts - expected_return*expected_return)*counts/(counts-1);
            }
            // assign
            info->set_value(expected_return, return_variance/counts);
        }

        DEBUG_OUT(1,QString("    backup state-node(%1):	value=%2").
                  arg(graph.id(state_node)).
                  arg(mcts_node_info_map[state_node].get_value()));
        DEBUG_OUT(1,QString("    backup action-node(%1):	value=%2").
                  arg(graph.id(action_node)).
                  arg(mcts_node_info_map[action_node].get_value()));
    }

} // namespace backup_method {
