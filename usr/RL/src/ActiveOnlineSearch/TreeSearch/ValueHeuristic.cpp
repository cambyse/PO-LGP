#include "ValueHeuristic.h"
#include "PriorModels.h"

#include <limits>

#include <util/util.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple_macros.h>

using std::vector;
using std::shared_ptr;
using std::make_shared;

namespace value_heuristic {

    void ValueHeuristic::init(double disc,
                              shared_ptr<AbstractEnvironment> env) {
        discount = disc;
        environment = env;
    }

    void Zero::add_value_estimate(const node_t & state_node,
                                  mcts_node_info_map_t & mcts_node_info_map) {
        DEBUG_EXPECT(0,environment!=nullptr);
        mcts_node_info_map[state_node].set_value(0,0,0,0);
    }

    RolloutStatistics::RolloutStatistics(double prior_counts):
        prior_counts(prior_counts)
    {}

    void RolloutStatistics::init(double disc,
                       shared_ptr<AbstractEnvironment> env) {
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

    void RolloutStatistics::add_value_estimate(const node_t & state_node,
                                               mcts_node_info_map_t & mcts_node_info_map) {
        DEBUG_EXPECT(0,environment!=nullptr);

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
        // get min max value from return
        reward_t min_value = std::numeric_limits<reward_t>::max();
        reward_t max_value = std::numeric_limits<reward_t>::lowest();
        for(auto rollout_item : mcts_node_info_map[state_node].rollout_set) {
            min_value = std::min(min_value,rollout_item->discounted_return);
            max_value = std::max(max_value,rollout_item->discounted_return);
        }
        // compute and set value
        prior_models::PriorCounts mean_and_variance(rollout_return_sum,
                                                    squared_rollout_return_sum,
                                                    rollout_counts,
                                                    min_return,
                                                    max_return,
                                                    prior_counts);
        mcts_node_info_map[state_node].set_value(mean_and_variance.mean,
                                                 mean_and_variance.variance,
                                                 min_value,
                                                 max_value);
        DEBUG_OUT(1,"value mean/variance: " << mean_and_variance.mean << "/" << mean_and_variance.variance);
    }

} // end namespace value_heuristic
