#include "TreePolicy.h"

#include <unordered_set>
#include <vector>
#include <utility>

#include <util/util.h>
#include <util/softmax.h>
#include <util/return_tuple.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <set>
#include <string>
#include <sstream>

using util::random_select;
using std::unordered_set;
using std::vector;
using std::pair;
using std::make_pair;
using lemon::INVALID;
using std::cout;
using std::endl;

namespace tree_policy {

    typedef MonteCarloTreeSearch::graph_t       graph_t;
    typedef MonteCarloTreeSearch::node_t        node_t;
    typedef MonteCarloTreeSearch::arc_t         arc_t;
    typedef MonteCarloTreeSearch::node_it_t     node_it_t;
    typedef MonteCarloTreeSearch::arc_it_t      arc_it_t;
    typedef MonteCarloTreeSearch::in_arc_it_t   in_arc_it_t;
    typedef MonteCarloTreeSearch::out_arc_it_t  out_arc_it_t;


    void TreePolicy::init(std::shared_ptr<AbstractEnvironment> env,
                          const graph_t & g,
                          const node_info_map_t & ni_map,
                          const mcts_node_info_map_t & mcts_ni_map,
                          const mcts_arc_info_map_t & mcts_ai_map) {
        environment = env;
        graph = &g;
        node_info_map = &ni_map;
        mcts_node_info_map = &mcts_ni_map;
        mcts_arc_info_map = &mcts_ai_map;
    }

    action_handle_t TreePolicy::get_action(const node_t & state_node) const {
        RETURN_TUPLE(action_container_t, actions,
                     vector<double>, probs) = get_action_probabilities(state_node);
        DEBUG_EXPECT(actions.size()>0);
        IF_DEBUG(1) {
            DEBUG_OUT(1,"Action probabilities (node " << graph->id(state_node) << "):");
            for(int idx=0; idx<(int)actions.size(); ++idx) {
                DEBUG_OUT(1,"    " << *(actions[idx]) << "	" << probs[idx] );
            }
            if(actions.size()==0) {
                DEBUG_OUT(1,"    node has outgoing arc? " << (out_arc_it_t(*graph,state_node)==INVALID));
            }
        }
        int idx = util::random_select_idx(probs);
        return actions[idx];
    }

    Uniform::action_probability_t Uniform::get_action_probabilities(const node_t & state_node) const {
        if(restrict_to_existing) {
            int action_n = 0;
            action_container_t actions;
            for(out_arc_it_t arc(*graph,state_node); arc!=INVALID; ++arc) {
                actions.push_back((*node_info_map)[graph->target(arc)].action);
                ++action_n;
            }
            vector<double> probs(action_n,1./action_n);
            return action_probability_t(actions, probs);
        } else {
            action_container_t actions = environment->get_actions();
            int action_n = actions.size();
            vector<double> probs(action_n,1./action_n);
            return action_probability_t(actions, probs);
        }
    }

    MaxPolicy::~MaxPolicy() {
        delete available_actions;
    }

    void MaxPolicy::init(std::shared_ptr<AbstractEnvironment> environment,
                         const graph_t & graph,
                         const node_info_map_t & node_info_map,
                         const mcts_node_info_map_t & mcts_node_info_map,
                         const mcts_arc_info_map_t & mcts_arc_info_map) {
        TreePolicy::init(environment,graph,node_info_map,mcts_node_info_map,mcts_arc_info_map);
        available_actions = new graph_t::NodeMap<action_container_t>(graph);
    }

    MaxPolicy::action_probability_t MaxPolicy::get_action_probabilities(const node_t & state_node) const {

        // get set of actions
        unordered_set<action_handle_t,
                      AbstractEnvironment::ActionHash,
                      AbstractEnvironment::ActionEq> action_set;
        {
            // remember action sets in case computation is costly
            action_container_t actions = (*available_actions)[state_node];
            if(actions.size()>0) {
#ifdef UNIT_TESTS
                std::vector<std::string> old_actions;
                for(auto a : actions) {
                    std::stringstream s;
                    s << *a;
                    old_actions.push_back(s.str());
                }
                std::vector<std::string> new_actions;
                for(auto a : environment->get_actions()) {
                    std::stringstream s;
                    s << *a;
                    new_actions.push_back(s.str());
                }
                DEBUG_EXPECT(old_actions==new_actions);
#endif
            } else {
                actions = environment->get_actions();
                (*available_actions)[state_node] = actions;
            }
            action_set.insert(actions.begin(), actions.end());
        }

        // compute scores for sampled actions
        vector<double> scores;
        vector<action_handle_t> scored_actions;
        for(out_arc_it_t to_action_arc(*graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t action_node = graph->target(to_action_arc);
            action_handle_t action = (*node_info_map)[action_node].action;
            double score_value = score(state_node, to_action_arc, action_node);
            scores.push_back(score_value);
            scored_actions.push_back(action);
            // erase this action from set
            action_set.erase(action);
        }
        // set scores to infinity for unsampled actions (if not restricted to
        // existing actions)
        if(!restrict_to_existing) {
            for(auto action : action_set) {
                scores.push_back(std::numeric_limits<double>::infinity());
                scored_actions.push_back(action);
            }
        }
        DEBUG_EXPECT(!scores.empty());
        DEBUG_EXPECT(scored_actions.size()==scores.size());

        // compute soft-max probabilities and return
        auto probabilities = util::soft_max(scores,soft_max_temperature);
        IF_DEBUG(0) {
            double sum = 0;
            for(auto prob : probabilities) {
                sum += prob;
                DEBUG_EXPECT(prob==prob);
            }
            DEBUG_EXPECT_APPROX(sum,1);
        }
        return action_probability_t(scored_actions, probabilities);
    }

    double Optimal::score(const node_t & state_node,
                            const arc_t & to_action_arc,
                            const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].value;
    }

    UCB1::UCB1(double Cp): Cp(Cp) {}

    double UCB1::score(const node_t & state_node,
                         const arc_t & to_action_arc,
                         const node_t & action_node) const {
        /* Sample actions according to UCB1 policy. The upper bound is computed
         * as $$Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + 2 C_p \sqrt{2\log n / n_j}$$
         *  */
        return (*mcts_node_info_map)[action_node].value +
            2*Cp*sqrt(
                2*log((*mcts_node_info_map)[state_node].action_counts)/
                (*mcts_arc_info_map)[to_action_arc].transition_counts
                );
    }

    UCB_Variance::UCB_Variance(double zeta, double c): zeta(zeta), c(c) {}

    /* $$Q_{(s,a)}^+ = \widehat{Q}_{(s,a)} + \sqrt{\frac{2V\zeta\log n}{n_j}} + c \frac{3b\zeta\log n}{n_j}$$
     *
     * where V is the variance of the return (not of the value!), b is the upper
     * bound on the reward (assuming zero as lower bound), and zeta and c > 0
     * control the behavior. */
    double UCB_Variance::score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const {
        double b = reward_bound();
        double n = (*mcts_node_info_map)[state_node].action_counts;
        double nj = (*mcts_arc_info_map)[to_action_arc].transition_counts;
        double score = (*mcts_node_info_map)[action_node].value_variance;
        if(score!=std::numeric_limits<double>::infinity()) {
            // we drop nj in the first term because we use the variance of the
            // value instead of the return
            score = (*mcts_node_info_map)[action_node].value +
                sqrt( 2 * (*mcts_node_info_map)[action_node].value_variance * zeta * log(n) ) +
                c * (3 * b * log(n) ) / nj;
        }
        return score;
    }

    double UCB_Variance::reward_bound() const {
        double b = 1;
        if(environment->has_min_reward() && environment->has_max_reward()) {
            b = environment->max_reward() - environment->min_reward();
        } else {
            DEBUG_WARNING("Environment does not have bounded reward. Using b = 1.");
        }
        return b;
    }

    double HardUpper::score(const node_t & state_node,
                              const arc_t & to_action_arc,
                              const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].max_value;
    }

    Quantile::Quantile(double Cp_,
                       double quantile_,
                       double min_return_,
                       double max_return_,
                       double prior_counts_):
        Cp(Cp_),
        quantile(quantile_),
        min_return(min_return_),
        max_return(max_return_),
        prior_counts(prior_counts_)
    {}

    double Quantile::score(const node_t & state_node,
                           const arc_t & to_action_arc,
                           const node_t & action_node) const {
        auto & rollouts = (*mcts_node_info_map)[action_node].rollout_set;
        // get counts to rescale weights for incorporating prior counts (weights
        // sum to 1)
        double counts = rollouts.size();
        DEBUG_EXPECT(counts>0);
        //-----------------------------------//
        // get sorted (by return value) list //
        //-----------------------------------//
        // also compute value
        double value = 0;
        vector<pair<double,double>> rollout_list;
        double weight_sum = 0;
        bool all_weights_negative = true;
        for(auto rollout_item : rollouts) {
            rollout_list.push_back(make_pair(rollout_item->discounted_return,
                                             rollout_item->weight*counts)); // rescale weights
            value += rollout_item->discounted_return * rollout_item->weight * counts;
            weight_sum += rollout_item->weight;
            if(rollout_item->weight>=0) {
                all_weights_negative = false;
            }
        }
        // if weights were not updated use uniform weighting
        if(all_weights_negative) {
            value = 0;
            for(auto & return_weight_pair : rollout_list) {
                return_weight_pair.second = 1;
                value += return_weight_pair.first;
            }
        } else {
            DEBUG_EXPECT_APPROX(weight_sum,1);
        }
        // add prior counts
        if(prior_counts>0) {
            rollout_list.push_back(make_pair(min_return, prior_counts/2));
            rollout_list.push_back(make_pair(max_return, prior_counts/2));
            value += min_return * prior_counts / 2;
            value += max_return * prior_counts / 2;
            counts += prior_counts;
        }
        // renormalize value
        value /= counts;
        // sort list
        std::sort(rollout_list.begin(),rollout_list.end());
        // find quantile
        bool found_lower = false;
        bool found_upper = false;
        double lower_quantile = std::min(1-quantile,quantile) * counts;
        double upper_quantile = std::max(1-quantile,quantile) * counts;
        auto lower_quantile_pair_below = rollout_list.front();
        auto lower_quantile_pair_above = rollout_list.front();
        auto upper_quantile_pair_below = rollout_list.front();
        auto upper_quantile_pair_above = rollout_list.front();
        for(auto & return_weight_pair : rollout_list) {
            if(!found_lower) {
                lower_quantile -= return_weight_pair.second;
                lower_quantile_pair_above = return_weight_pair;
                if(lower_quantile<0) found_lower = true;
                else lower_quantile_pair_below = return_weight_pair;
            }
            if(!found_upper) {
                upper_quantile -= return_weight_pair.second;
                upper_quantile_pair_above = return_weight_pair;
                if(upper_quantile<0) found_upper = true;
                else upper_quantile_pair_below = return_weight_pair;
            }
            if(found_lower && found_upper) break;
        }
        DEBUG_EXPECT(found_lower && found_upper);
        // interpolate linearly between the two values
        double t_lower = (lower_quantile_pair_above.second+lower_quantile)/lower_quantile_pair_above.second;
        double lower_quantile_value = (1-t_lower)*lower_quantile_pair_below.first + t_lower*lower_quantile_pair_above.first;
        double t_upper = (upper_quantile_pair_above.second+upper_quantile)/upper_quantile_pair_above.second;
        double upper_quantile_value = (1-t_upper)*upper_quantile_pair_below.first + t_upper*upper_quantile_pair_above.first;
        DEBUG_EXPECT(t_lower>=0 && t_lower<=1);
        DEBUG_EXPECT(t_upper>=0 && t_upper<=1);
        DEBUG_EXPECT(lower_quantile_value==lower_quantile_value);
        DEBUG_EXPECT(upper_quantile_value==upper_quantile_value);
        DEBUG_EXPECT(value==value);
        double exploration_term = log((*mcts_node_info_map)[state_node].action_counts) /
            (*mcts_arc_info_map)[to_action_arc].transition_counts;
        return value + Cp * upper_quantile_value + Cp*sqrt(exploration_term);
        // compute score
        double b = 1;
        if(environment->has_min_reward() && environment->has_max_reward()) {
            b = environment->max_reward() - environment->min_reward();
        }
        double n = (*mcts_node_info_map)[state_node].action_counts;
        double nj = (*mcts_arc_info_map)[to_action_arc].transition_counts;
        double score = (*mcts_node_info_map)[action_node].value +
            sqrt( (2 * (upper_quantile_value - lower_quantile_value) * log(n) ) / nj ) +
            Cp * (3 * b * log(n) ) / nj;
        return score;
    }

} // end namespace tree_policy
