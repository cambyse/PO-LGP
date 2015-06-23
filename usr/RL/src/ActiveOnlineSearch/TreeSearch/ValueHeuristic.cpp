#include "ValueHeuristic.h"
#include "PriorModels.h"

#include <limits>

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple_macros.h>

namespace value_heuristic {

    void ValueHeuristic::init(double disc,
                              std::shared_ptr<AbstractEnvironment> env) {
        discount = disc;
        environment = env;
    }

    reward_t Zero::add_value_estimate(const node_t & state_node,
                                      mcts_node_info_map_t & mcts_node_info_map) {
        DEBUG_EXPECT(0,environment!=nullptr);
        mcts_node_info_map[state_node].add_rollout_return(0);
        mcts_node_info_map[state_node].add_rollout_to_set(std::make_shared<RolloutItem>());
        return 0;
    }

    Rollout::Rollout(int rollout_length, double prior_counts):
        rollout_length(rollout_length),
        prior_counts(prior_counts)
    {}

    void Rollout::init(double disc,
                       std::shared_ptr<AbstractEnvironment> env) {
        ValueHeuristic::init(disc,env);
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

    reward_t Rollout::add_value_estimate(const node_t & state_node,
                                         mcts_node_info_map_t & mcts_node_info_map) {
        DEBUG_EXPECT(0,environment!=nullptr);

        // clear last rollout
        last_rollout.clear();

        // no rollout if we're in a terminal state
        if(environment->is_terminal_state()) {
            DEBUG_OUT(1,"Set value of terminal node all to zero");
            mcts_node_info_map[state_node].set_value(0,0,0,0);
            mcts_node_info_map[state_node].min_return = 0;
            mcts_node_info_map[state_node].max_return = 0;
            last_rollout.push_back(std::make_shared<RolloutItem>());
            return 0;
        }

        // Do rollout of specified length (rollout_length).
        int length = rollout_length;

        // If rollout_length is not specified (smaller than zero) and
        // environment does not have a terminal state do one-step
        // rollout. Otherwise do infinite rollout (util terminal state).
        if(length<0 && !environment->has_terminal_state()) {
            length = 1;
        }
        observation_handle_t observation;
        action_handle_t action;
        reward_t reward;
        reward_t discounted_return = 0;
        double discount_factor = 1;
        int k=0;
        while((length<0 || k<length) && !environment->is_terminal_state()) {
            // select action uniformly from available actions
            action = util::random_select(environment->get_actions());
            // sample transition
            t(observation,reward) = environment->transition(action);
            // update return
            discounted_return += discount_factor*reward;
            // update counter and discount factor
            ++k;
            discount_factor*=discount;
            // update last rollout (update discounted return and next item
            // later)
            last_rollout.push_back(std::make_shared<RolloutItem>(action,observation,reward,0,1,nullptr));
        }
        // "empty" rollout item for last node
        last_rollout.push_back(std::make_shared<RolloutItem>());
        // now update discounted return
        {
            reward_t reverse_discounted_return = 0;
            std::shared_ptr<RolloutItem> next = nullptr;
            for(auto rollout_item_it=last_rollout.rbegin(); rollout_item_it!=last_rollout.rend(); ++rollout_item_it) {
                (*rollout_item_it)->discounted_return = reverse_discounted_return;
                reverse_discounted_return = (*rollout_item_it)->reward + discount * reverse_discounted_return;
                (*rollout_item_it)->next = next;
                next = (*rollout_item_it);
            }
            DEBUG_EXPECT(0,fabs(reverse_discounted_return-discounted_return)<1e-10);
        }
        // add rollout
        mcts_node_info_map[state_node].add_rollout_return(discounted_return);
        mcts_node_info_map[state_node].add_rollout_to_set(last_rollout.front());
        // set value (mean and variance)
        auto rollout_return_sum = mcts_node_info_map[state_node].return_sum;
        auto squared_rollout_return_sum = mcts_node_info_map[state_node].squared_return_sum;
        auto rollout_counts = mcts_node_info_map[state_node].rollout_counts;
        DEBUG_EXPECT(0,rollout_counts>=1);
        reward_t min_return = 0;
        reward_t max_return = 0;
        if(prior_counts>0) {
            min_return = environment->min_reward()/(1-discount);
            max_return = environment->max_reward()/(1-discount);
        }
        DEBUG_OUT(2,"rollout_return_sum: " << rollout_return_sum);
        DEBUG_OUT(2,"squared_rollout_return_sum: " << squared_rollout_return_sum);
        DEBUG_OUT(2,"rollout_counts: " << rollout_counts);
        DEBUG_OUT(2,"min_return: " << min_return);
        DEBUG_OUT(2,"max_return: " << max_return);
        DEBUG_OUT(2,"prior_counts: " << prior_counts);
        prior_models::PriorCounts mean_and_variance(rollout_return_sum,
                                                    squared_rollout_return_sum,
                                                    rollout_counts,
                                                    min_return,
                                                    max_return,
                                                    prior_counts);
        mcts_node_info_map[state_node].set_value(mean_and_variance.mean,
                                                 mean_and_variance.variance,
                                                 std::min(mcts_node_info_map[state_node].min_value,discounted_return),
                                                 std::max(mcts_node_info_map[state_node].max_value,discounted_return));
        DEBUG_OUT(1,"value mean/variance: " << mean_and_variance.mean << "/" << mean_and_variance.variance);
        if(discounted_return!=0) DEBUG_OUT(1,"rollout return: " << discounted_return);

        return discounted_return;
    }

} // end namespace value_heuristic
