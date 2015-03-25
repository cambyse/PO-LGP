#include "BackupMethod.h"

#include <float.h>
#include <limits>
#include <algorithm>
#include <vector>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using lemon::INVALID;
using std::vector;

namespace backup_method {

    void Bellman::operator()(const node_t & state_node,
                             const node_t & action_node,
                             double discount,
                             std::shared_ptr<const Environment> environment,
                             const graph_t & graph,
                             mcts_node_info_map_t & mcts_node_info_map,
                             const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action value
        {
            arc_t to_action_arc = in_arc_it_t(graph,action_node);
            reward_t mean_reward = mcts_arc_info_map[to_action_arc].get_reward_sum()/mcts_arc_info_map[to_action_arc].get_counts();
            reward_t mean_squared_reward = mcts_arc_info_map[to_action_arc].get_squared_reward_sum()/mcts_arc_info_map[to_action_arc].get_counts();
            reward_t reward_variance = mean_squared_reward - mean_reward*mean_reward;
            reward_t action_value = mean_reward;
            reward_t action_value_variance = reward_variance;
            for(out_arc_it_t to_state_arc_1(graph, action_node); to_state_arc_1!=INVALID; ++to_state_arc_1) {

                node_t target_state_node_1 = graph.target(to_state_arc_1);
                double prob_1 = mcts_arc_info_map[to_state_arc_1].get_counts()/mcts_node_info_map[action_node].get_transition_counts();
                reward_t state_value_1 = mcts_node_info_map[target_state_node_1].get_value();

                // update action value
                action_value += discount*prob_1*state_value_1;

                // update action value variance (first sum)
                action_value_variance += discount*discount*
                    mean_squared_reward*
                    prob_1*
                    mcts_node_info_map[target_state_node_1].get_value_variance();

                // update action value variance (second sum)
                for(out_arc_it_t to_state_arc_2(graph, action_node); to_state_arc_2!=INVALID; ++to_state_arc_2) {

                    node_t target_state_node_2 = graph.target(to_state_arc_2);
                    double prob_2 = mcts_arc_info_map[to_state_arc_2].get_counts()/mcts_node_info_map[action_node].get_transition_counts();
                    reward_t state_value_2 = mcts_node_info_map[target_state_node_2].get_value();
                    double p_covariance = target_state_node_1==target_state_node_2?
                        prob_1*(1-prob_1):
                        -prob_1*prob_2;

                    action_value_variance += discount*discount*(
                        reward_variance*prob_1*prob_2 +
                        mean_squared_reward*p_covariance
                        )*state_value_1*state_value_2;
                }
            }
            mcts_node_info_map[action_node].set_value(action_value, action_value_variance);
        }

        // compute state value
        {
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
                                std::shared_ptr<const Environment> environment,
                                const graph_t & graph,
                                mcts_node_info_map_t & mcts_node_info_map,
                                const mcts_arc_info_map_t & mcts_arc_info_map) const {
        // compute action value and variance
        auto& action_info = mcts_node_info_map[action_node];
        int action_counts = action_info.get_rollout_counts();
        reward_t action_value = action_info.get_return_sum()/action_info.get_rollout_counts();
        reward_t action_value_variance = action_info.get_squared_return_sum()/action_counts
            - action_value*action_value;
        // correcting variance bias
        if(action_counts<2) {
            action_value_variance = std::numeric_limits<double>::infinity();
            //action_value_variance = 1e10;
        } else {
            action_value_variance *= action_counts/(action_counts-1);
        }
        // assign
        action_info.set_value(action_value, action_value_variance);

        // compute state value and variance
        auto& state_info = mcts_node_info_map[state_node];
        int state_counts = state_info.get_rollout_counts();
        reward_t state_value = state_info.get_return_sum()/state_info.get_rollout_counts();
        reward_t state_value_variance = state_info.get_squared_return_sum()/state_counts
            - state_value*state_value;
        // correcting variance bias
        if(state_counts<2) {
            state_value_variance = std::numeric_limits<double>::infinity();
            //state_value_variance = 1e10;
        } else {
            state_value_variance *= state_counts/(state_counts-1);
        }
        // assign
        state_info.set_value(state_value, state_value_variance);

        DEBUG_OUT(1,QString("    backup state-node(%1):	value=%2").
                  arg(graph.id(state_node)).
                  arg(mcts_node_info_map[state_node].get_value()));
        DEBUG_OUT(1,QString("    backup action-node(%1):	value=%2").
                  arg(graph.id(action_node)).
                  arg(mcts_node_info_map[action_node].get_value()));
    }

} // namespace backup_method {
