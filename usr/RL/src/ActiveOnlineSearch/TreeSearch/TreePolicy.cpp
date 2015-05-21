#include "TreePolicy.h"

#include <unordered_set>
#include <vector>
#include <utility>

#include "../Environment/Environment.h"

#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using util::random_select;
using std::unordered_set;
using std::vector;
using std::pair;
using std::make_pair;
using lemon::INVALID;

namespace tree_policy {

    typedef MonteCarloTreeSearch::graph_t       graph_t;
    typedef MonteCarloTreeSearch::node_t        node_t;
    typedef MonteCarloTreeSearch::arc_t         arc_t;
    typedef MonteCarloTreeSearch::node_it_t     node_it_t;
    typedef MonteCarloTreeSearch::arc_it_t      arc_it_t;
    typedef MonteCarloTreeSearch::in_arc_it_t   in_arc_it_t;
    typedef MonteCarloTreeSearch::out_arc_it_t  out_arc_it_t;

    action_handle_t Uniform::operator()(const node_t & state_node,
                                 std::shared_ptr<AbstractEnvironment> environment,
                                 const graph_t & graph,
                                 const node_info_map_t & node_info_map,
                                 const mcts_node_info_map_t & mcts_node_info_map,
                                 const mcts_arc_info_map_t & mcts_arc_info_map) const {
        action_handle_t action = random_select(environment->get_actions());
        DEBUG_OUT(1,"Select action: " << *action);
        return action;
    }

    action_handle_t MaxPolicy::operator()(const node_t & state_node,
                                   std::shared_ptr<AbstractEnvironment> environment,
                                   const graph_t & graph,
                                   const node_info_map_t & node_info_map,
                                   const mcts_node_info_map_t & mcts_node_info_map,
                                   const mcts_arc_info_map_t & mcts_arc_info_map) const {

        // get set of actions
        auto actions = environment->get_actions();
        unordered_set<action_handle_t,
                      AbstractEnvironment::ActionHash,
                      AbstractEnvironment::ActionEq> action_set(actions.begin(), actions.end());

        // prepare vector for computing upper bounds
        vector<pair<reward_t,action_handle_t>> scores;

        // compute upper bounds
        DEBUG_OUT(3,"Computing upper bound for state node " << graph.id(state_node));
        for(out_arc_it_t to_action_arc(graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t action_node = graph.target(to_action_arc);
            action_handle_t action = node_info_map[action_node].action;
            reward_t upper = score(state_node,
                                   to_action_arc,
                                   action_node,
                                   environment,
                                   graph,
                                   node_info_map,
                                   mcts_node_info_map,
                                   mcts_arc_info_map);
            scores.push_back(make_pair(upper,action));
            // erase this action from set
            action_set.erase(action);
        }

        // select unsampled action if there are any left
        if(action_set.size()>0) {
            action_handle_t action = random_select(action_set);
            DEBUG_OUT(2,"Selecting unsampled action: " << *action);
            return action;
        } else {
            DEBUG_OUT(2,"No unsampled actions.");
        }

        // select max upper bound action otherwise
        IF_DEBUG(3) {
            DEBUG_OUT(3,"Use upper bound to choose between:");
            for(auto bound_action : scores) {
                DEBUG_OUT(3,"    '" << *(bound_action.second) << "' with bound " << bound_action.first);
            }
        }
        DEBUG_EXPECT(1,scores.size()>0);
        reward_t max_score = -DBL_MAX;
        vector<action_handle_t> max_score_actions;
        for(auto bound_action : scores) {
            if(bound_action.first>max_score) {
                max_score_actions.clear();
            }
            if(bound_action.first>=max_score) {
                max_score = bound_action.first;
                max_score_actions.push_back(bound_action.second);
            }
        }

        // random tie breaking between action with equal upper bound
        action_handle_t action = random_select(max_score_actions);
        DEBUG_OUT(2,"Choosing action " << *action << " with upper bound " << max_score );
        return action;
    }

    reward_t Optimal::score(const node_t & state_node,
                            const arc_t & to_action_arc,
                            const node_t & action_node,
                            std::shared_ptr<AbstractEnvironment> environment,
                            const graph_t & graph,
                            const node_info_map_t & node_info_map,
                            const mcts_node_info_map_t & mcts_node_info_map,
                            const mcts_arc_info_map_t & mcts_arc_info_map) const {
        return mcts_node_info_map[action_node].get_value();
    }

    UCB1::UCB1(double Cp): Cp(Cp) {}

    reward_t UCB1::score(const node_t & state_node,
                         const arc_t & to_action_arc,
                         const node_t & action_node,
                         std::shared_ptr<AbstractEnvironment> environment,
                         const graph_t & graph,
                         const node_info_map_t & node_info_map,
                         const mcts_node_info_map_t & mcts_node_info_map,
                         const mcts_arc_info_map_t & mcts_arc_info_map) const {
        return mcts_node_info_map[action_node].get_value() +
            2*Cp*sqrt(
                2*log(mcts_node_info_map[state_node].get_transition_counts())/
                mcts_arc_info_map[to_action_arc].get_transition_counts()
                );
    }

    UCB_Plus::UCB_Plus(double Cp): Cp(Cp) {}

    reward_t UCB_Plus::score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node,
                             std::shared_ptr<AbstractEnvironment> environment,
                             const graph_t & graph,
                             const node_info_map_t & node_info_map,
                             const mcts_node_info_map_t & mcts_node_info_map,
                             const mcts_arc_info_map_t & mcts_arc_info_map) const {

        // upper bound = value + Cp sqrt( value_variance / n) where n is the
        // number of times this action was taken.
        return mcts_node_info_map[action_node].get_value() +
            Cp*sqrt(mcts_node_info_map[action_node].get_value_variance());
    }

} // end namespace tree_policy
