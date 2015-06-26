#include "BackupMethod.h"
#include "PriorModels.h"

#include <util/QtUtil.h>
#include <util/return_tuple.h>

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

    void BackupMethod::init(double discount_,
                            std::shared_ptr<AbstractEnvironment> environment_,
                            const graph_t & graph_,
                            const node_info_map_t & node_info_map_,
                            mcts_node_info_map_t & mcts_node_info_map_,
                            const mcts_arc_info_map_t & mcts_arc_info_map_) {
        discount = discount_;
        environment = environment_;
        graph = &graph_;
        node_info_map = &node_info_map_;
        mcts_node_info_map = &mcts_node_info_map_;
        mcts_arc_info_map = &mcts_arc_info_map_;
    }

    Bellman::Bellman(std::shared_ptr<tree_policy::TreePolicy> tree_policy_,
                     double prior_counts):
        tree_policy(tree_policy_),
        prior_counts(prior_counts)
    {}

    void Bellman::init(double discount_,
                       std::shared_ptr<AbstractEnvironment> environment_,
                       const graph_t & graph_,
                       const node_info_map_t & node_info_map_,
                       mcts_node_info_map_t & mcts_node_info_map_,
                       const mcts_arc_info_map_t & mcts_arc_info_map_) {
        BackupMethod::init(discount_,
                           environment_,
                           graph_,
                           node_info_map_,
                           mcts_node_info_map_,
                           mcts_arc_info_map_);
        // init prior counts
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
        // init policy
        if(tree_policy==nullptr) tree_policy.reset(new tree_policy::Optimal());
        tree_policy->init(environment,
                          *graph,
                          *node_info_map,
                          *mcts_node_info_map,
                          *mcts_arc_info_map);
        tree_policy->restrict_to_existing = true;
    }

    void Bellman::backup_action_node(const node_t & action_node) const {
        DEBUG_EXPECT(0,environment!=nullptr);
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
        reward_t mean_reward_variance = mean_and_variance.variance_of_mean;
        DEBUG_OUT(3,"    ^r=" << mean_reward << ", ~r=" << mean_reward_variance);
        // transition probabilities
        vector<double> counts;
        for(out_arc_it_t arc(*graph, action_node); arc!=INVALID; ++arc) {
            counts.push_back((*mcts_arc_info_map)[arc].transition_counts);
        }
        prior_models::Dirichlet prob(counts);
        // compute value (mean and variance)
        reward_t action_value = mean_reward;
        reward_t action_value_variance = mean_reward_variance;
        reward_t min_action_value = 0;
        reward_t max_action_value = 0;
        int idx_1 = 0;
        for(out_arc_it_t to_state_arc_1(*graph, action_node);
            to_state_arc_1!=INVALID;
            ++to_state_arc_1, ++idx_1) {

            node_t state_node_1 = graph->target(to_state_arc_1);
            DEBUG_OUT(3,"    transition to state node " << graph->id(state_node_1));
            DEBUG_OUT(3,"        ^p=" << prob.mean[idx_1] << ", ~p=" << prob.covariance[idx_1][idx_1]);
            DEBUG_OUT(3,"        ^V=" << (*mcts_node_info_map)[state_node_1].value << ", ~V=" << (*mcts_node_info_map)[state_node_1].value_variance);

            // update action value
            action_value += discount*prob.mean[idx_1]*(*mcts_node_info_map)[state_node_1].value;

            // update action value variance (first sum)
            action_value_variance += discount*discount*
                (prob.mean[idx_1]*prob.mean[idx_1] + prob.covariance[idx_1][idx_1])*
                (*mcts_node_info_map)[state_node_1].value_variance;

            // update action value bounds
            min_action_value += discount*prob.mean[idx_1]*(*mcts_node_info_map)[state_node_1].min_value;
            max_action_value += discount*prob.mean[idx_1]*(*mcts_node_info_map)[state_node_1].max_value;

            // update action value variance (second sum)
            int idx_2 = 0;
            for(out_arc_it_t to_state_arc_2(*graph, action_node);
                to_state_arc_2!=INVALID;
                ++to_state_arc_2, ++idx_2) {

                node_t state_node_2 = graph->target(to_state_arc_2);
                DEBUG_OUT(3,"        transition-pair to state nodes " << graph->id(state_node_1) << "/" << graph->id(state_node_2));
                DEBUG_OUT(3,"        ^p=" << prob.mean[idx_1] << "/" << prob.mean[idx_2] << ", ~p=" << prob.covariance[idx_1][idx_2]);
                DEBUG_OUT(3,"        ^V=" << (*mcts_node_info_map)[state_node_1].value << "/" << (*mcts_node_info_map)[state_node_2].value);

                action_value_variance += discount*discount*
                    prob.covariance[idx_1][idx_2]*
                    (*mcts_node_info_map)[state_node_1].value*(*mcts_node_info_map)[state_node_2].value;
            }
        }
        (*mcts_node_info_map)[action_node].set_value(action_value,
                                                  action_value_variance,
                                                  min_action_value,
                                                  max_action_value);
        DEBUG_OUT(2,QString("Assign	^Q=%1	~Q=%2	↹Q=%3/%4").
                  arg(action_value).
                  arg(action_value_variance).
                  arg(min_action_value).
                  arg(max_action_value));
    }

    void Bellman::backup_observation_node(const node_t & observation_node) const {
        DEBUG_EXPECT(0,environment!=nullptr);
        DEBUG_EXPECT(0,tree_policy!=nullptr);
        DEBUG_EXPECT(0,tree_policy->restrict_to_existing);
        DEBUG_OUT(1,"Backup observation node " << graph->id(observation_node));
        // get action probabilities
        RETURN_TUPLE(action_container_t, actions,
                     vector<double>, probs) = tree_policy->get_action_probabilities(observation_node);
        std::unordered_map<action_handle_t,node_t,
                           AbstractEnvironment::ActionHash,
                           AbstractEnvironment::ActionEq> action_nodes;
        for(out_arc_it_t to_action_arc(*graph, observation_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t action_node = graph->target(to_action_arc);
            action_nodes[(*node_info_map)[action_node].action] = action_node;
        }
        double mean_value = 0;
        double mean_value_variance = 0;
        double min_value = 0;
        double max_value = 0;
        for(int idx=0; idx<(int)actions.size(); ++idx) {
            node_t action_node = action_nodes[actions[idx]];
            mean_value += probs[idx]*(*mcts_node_info_map)[action_node].value;
#warning Does the variance REALLY go quadratic with the probability??!
            mean_value_variance += pow(probs[idx],2)*(*mcts_node_info_map)[action_node].value_variance;
            min_value += probs[idx]*(*mcts_node_info_map)[action_node].min_value;
            max_value += probs[idx]*(*mcts_node_info_map)[action_node].max_value;
        }
        // old
        auto action = tree_policy->get_action(observation_node);
        node_t action_node = INVALID;
        for(out_arc_it_t to_action_arc(*graph, observation_node); to_action_arc!=INVALID; ++to_action_arc) {
            action_node = graph->target(to_action_arc);
            if((*node_info_map)[action_node].action==action) {
                break;
            }
        }
        DEBUG_EXPECT(0,action_node!=INVALID);
        // assign
        (*mcts_node_info_map)[observation_node].set_value((*mcts_node_info_map)[action_node].value,
                                                       (*mcts_node_info_map)[action_node].value_variance,
                                                       (*mcts_node_info_map)[action_node].min_value,
                                                       (*mcts_node_info_map)[action_node].max_value);
        DEBUG_OUT(2,QString("Assign	^Q=%1	~Q=%2	↹Q=%3/%4").
                  arg((*mcts_node_info_map)[action_node].value).
                  arg((*mcts_node_info_map)[action_node].value_variance).
                  arg((*mcts_node_info_map)[action_node].min_value).
                  arg((*mcts_node_info_map)[action_node].max_value));
        // DEBUG_EXPECT(0,fabs((*mcts_node_info_map)[action_node].value-mean_value)<1e-10);
        // DEBUG_EXPECT(0,fabs((*mcts_node_info_map)[action_node].value_variance-mean_value_variance)<1e-10);
        // DEBUG_EXPECT(0,fabs((*mcts_node_info_map)[action_node].min_value-min_value)<1e-10);
        // DEBUG_EXPECT(0,fabs((*mcts_node_info_map)[action_node].max_value-max_value)<1e-10);
        // if(fabs((*mcts_node_info_map)[action_node].value-mean_value)>=1e-10 ||
        //    fabs((*mcts_node_info_map)[action_node].value_variance-mean_value_variance)>=1e-10 ||
        //    fabs((*mcts_node_info_map)[action_node].min_value-min_value)>=1e-10 ||
        //    fabs((*mcts_node_info_map)[action_node].max_value-max_value)>=1e-10) {
        //     DEBUG_OUT(0,(*mcts_node_info_map)[action_node].value << " / " << mean_value);
        //     DEBUG_OUT(0,(*mcts_node_info_map)[action_node].value_variance << " / " << mean_value_variance);
        //     DEBUG_OUT(0,(*mcts_node_info_map)[action_node].min_value << " / " << min_value);
        //     DEBUG_OUT(0,(*mcts_node_info_map)[action_node].max_value << " / " << max_value);
        //     DEBUG_OUT(0,"    " << probs);
        // }
    }

    MonteCarlo::MonteCarlo(double prior_counts):
        prior_counts(prior_counts)
    {}

    void MonteCarlo::init(double discount_,
                          std::shared_ptr<AbstractEnvironment> environment_,
                          const graph_t & graph_,
                          const node_info_map_t & node_info_map_,
                          mcts_node_info_map_t & mcts_node_info_map_,
                          const mcts_arc_info_map_t & mcts_arc_info_map_) {
        BackupMethod::init(discount_,
                           environment_,
                           graph_,
                           node_info_map_,
                           mcts_node_info_map_,
                           mcts_arc_info_map_);
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

    void MonteCarlo::backup_action_node(const node_t & action_node) const {
        backup_node(action_node);
        DEBUG_OUT(1,QString("    backup action-node(%1):	^V=%2	~V=%3	V+/-=%4/%5").
                  arg(graph->id(action_node)).
                  arg((*mcts_node_info_map)[action_node].value).
                  arg((*mcts_node_info_map)[action_node].value_variance).
                  arg((*mcts_node_info_map)[action_node].min_value).
                  arg((*mcts_node_info_map)[action_node].max_value));
    }

    void MonteCarlo::backup_observation_node(const node_t & observation_node) const {
        backup_node(observation_node);
        DEBUG_OUT(1,QString("    backup observ.-node(%1):	^V=%2	~V=%3	V+/-=%4/%5").
                  arg(graph->id(observation_node)).
                  arg((*mcts_node_info_map)[observation_node].value).
                  arg((*mcts_node_info_map)[observation_node].value_variance).
                  arg((*mcts_node_info_map)[observation_node].min_value).
                  arg((*mcts_node_info_map)[observation_node].max_value));
    }

    void MonteCarlo::backup_node(const node_t & node) const {
        auto * info = &(*mcts_node_info_map)[node];
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
                        mean_and_variance.variance_of_mean,
                        info->min_return,
                        info->max_return);
        // debug info
        DEBUG_OUT(3,"    info->return_sum	" << info->return_sum);
        DEBUG_OUT(3,"    info->squared_return_sum	" << info->squared_return_sum);
        DEBUG_OUT(3,"    info->rollout_counts	" << info->rollout_counts);
        DEBUG_OUT(3,"    min_return	" << min_return);
        DEBUG_OUT(3,"    max_return	" << max_return);
        DEBUG_OUT(3,"    prior_counts	" << prior_counts);
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

    void HybridMCDP::init(double discount_,
                          std::shared_ptr<AbstractEnvironment> environment_,
                          const graph_t & graph_,
                          const node_info_map_t & node_info_map_,
                          mcts_node_info_map_t & mcts_node_info_map_,
                          const mcts_arc_info_map_t & mcts_arc_info_map_) {
        BackupMethod::init(discount_,
                           environment_,
                           graph_,
                           node_info_map_,
                           mcts_node_info_map_,
                           mcts_arc_info_map_);
        monte_carlo.init(discount_,
                         environment_,
                         graph_,
                         node_info_map_,
                         mcts_node_info_map_,
                         mcts_arc_info_map_);
        bellman.init(discount_,
                     environment_,
                     graph_,
                     node_info_map_,
                     mcts_node_info_map_,
                     mcts_arc_info_map_);
    }

    void HybridMCDP::backup_action_node(const node_t & action_node) const {
        // make bellman backup
        bellman.backup_action_node(action_node);
        auto bellman_action_value = (*mcts_node_info_map)[action_node].value;
        auto bellman_action_value_variance = (*mcts_node_info_map)[action_node].value_variance;
        auto bellman_min_action_value = (*mcts_node_info_map)[action_node].min_value;
        auto bellman_max_action_value = (*mcts_node_info_map)[action_node].max_value;
        // make monte-carlo backup
        monte_carlo.backup_action_node(action_node);
        auto monte_carlo_action_value = (*mcts_node_info_map)[action_node].value;
        auto monte_carlo_action_value_variance = (*mcts_node_info_map)[action_node].value_variance;
        auto monte_carlo_min_action_value = (*mcts_node_info_map)[action_node].min_value;
        auto monte_carlo_max_action_value = (*mcts_node_info_map)[action_node].max_value;
        // average both
        (*mcts_node_info_map)[action_node].value          = mc_weight*monte_carlo_action_value          + (1-mc_weight)*bellman_action_value;
        (*mcts_node_info_map)[action_node].value_variance = mc_weight*monte_carlo_action_value_variance + (1-mc_weight)*bellman_action_value_variance;
        (*mcts_node_info_map)[action_node].min_value      = mc_weight*monte_carlo_min_action_value      + (1-mc_weight)*bellman_min_action_value;
        (*mcts_node_info_map)[action_node].max_value      = mc_weight*monte_carlo_max_action_value      + (1-mc_weight)*bellman_max_action_value;
    }

    void HybridMCDP::backup_observation_node(const node_t & observation_node) const {
        // make bellman backup
        bellman.backup_observation_node(observation_node);
        auto bellman_observation_value = (*mcts_node_info_map)[observation_node].value;
        auto bellman_observation_value_variance = (*mcts_node_info_map)[observation_node].value_variance;
        auto bellman_min_observation_value = (*mcts_node_info_map)[observation_node].min_value;
        auto bellman_max_observation_value = (*mcts_node_info_map)[observation_node].max_value;
        // make monte-carlo backup
        monte_carlo.backup_observation_node(observation_node);
        auto monte_carlo_observation_value = (*mcts_node_info_map)[observation_node].value;
        auto monte_carlo_observation_value_variance = (*mcts_node_info_map)[observation_node].value_variance;
        auto monte_carlo_min_observation_value = (*mcts_node_info_map)[observation_node].min_value;
        auto monte_carlo_max_observation_value = (*mcts_node_info_map)[observation_node].max_value;
        // average both
        (*mcts_node_info_map)[observation_node].value          = mc_weight*monte_carlo_observation_value          + (1-mc_weight)*bellman_observation_value;
        (*mcts_node_info_map)[observation_node].value_variance = mc_weight*monte_carlo_observation_value_variance + (1-mc_weight)*bellman_observation_value_variance;
        (*mcts_node_info_map)[observation_node].min_value      = mc_weight*monte_carlo_min_observation_value      + (1-mc_weight)*bellman_min_observation_value;
        (*mcts_node_info_map)[observation_node].max_value      = mc_weight*monte_carlo_max_observation_value      + (1-mc_weight)*bellman_max_observation_value;
    }

} // namespace backup_method {
