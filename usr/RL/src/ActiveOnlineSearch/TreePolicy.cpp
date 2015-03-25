#include "TreePolicy.h"

#include <set>
#include <vector>
#include <utility>

#include "Environment.h"
#include "AbstractMonteCarloTreeSearch.h"

#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using util::random_select;
using std::set;
using std::vector;
using std::pair;
using std::make_pair;
using lemon::INVALID;

namespace tree_policy {

    typedef AbstractMonteCarloTreeSearch::graph_t       graph_t;
    typedef AbstractMonteCarloTreeSearch::node_t        node_t;
    typedef AbstractMonteCarloTreeSearch::arc_t         arc_t;
    typedef AbstractMonteCarloTreeSearch::node_it_t     node_it_t;
    typedef AbstractMonteCarloTreeSearch::arc_it_t      arc_it_t;
    typedef AbstractMonteCarloTreeSearch::in_arc_it_t   in_arc_it_t;
    typedef AbstractMonteCarloTreeSearch::out_arc_it_t  out_arc_it_t;
    typedef Environment::action_t     action_t;
    typedef Environment::state_t      state_t;
    typedef Environment::reward_t     reward_t;

    action_t Uniform::operator()(const node_t & state_node,
                                 std::shared_ptr<const Environment> environment,
                                 const graph_t & graph,
                                 const node_info_map_t & node_info_map,
                                 const mcts_node_info_map_t & mcts_node_info_map,
                                 const mcts_arc_info_map_t & mcts_arc_info_map) const {
        action_t action = random_select(environment->get_actions());
        DEBUG_OUT(1,"Select action: " << environment->action_name(action));
        return action;
    }


    action_t UpperBoundPolicy::operator()(const node_t & state_node,
                                          std::shared_ptr<const Environment> environment,
                                          const graph_t & graph,
                                          const node_info_map_t & node_info_map,
                                          const mcts_node_info_map_t & mcts_node_info_map,
                                          const mcts_arc_info_map_t & mcts_arc_info_map) const {

        // get set of actions
        set<action_t> action_set(environment->get_actions().begin(), environment->get_actions().end());

        // prepare vector for computing upper bounds
        vector<pair<reward_t,action_t>> upper_bounds;

        // comput upper bounds
        DEBUG_OUT(3,"Computing upper bound for state node " << graph.id(state_node));
        for(out_arc_it_t to_action_arc(graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t action_node = graph.target(to_action_arc);
            action_t action = node_info_map[action_node].action;
            reward_t upper = upper_bound(state_node,
                                         to_action_arc,
                                         action_node,
                                         environment,
                                         graph,
                                         node_info_map,
                                         mcts_node_info_map,
                                         mcts_arc_info_map);
            upper_bounds.push_back(make_pair(upper,action));
            // erase this action from set
            action_set.erase(action);
        }

        // select unsampled action if there are any left
        if(action_set.size()>0) {
            action_t action = random_select(action_set);
            DEBUG_OUT(2,"Selecting unsampled action: " << environment->action_name(action));
            return action;
        } else {
            DEBUG_OUT(2,"No unsampled actions.");
        }

        // select max upper bound action otherwise
        IF_DEBUG(3) {
            DEBUG_OUT(3,"Use upper bound to choose between:");
            for(auto bound_action : upper_bounds) {
                DEBUG_OUT(3,"    '" << environment->action_name(bound_action.second) <<
                          "' with bound " << bound_action.first);
            }
        }
        DEBUG_EXPECT(1,upper_bounds.size()>0);
        reward_t max_upper_bound = -DBL_MAX;
        vector<action_t> max_upper_bound_actions;
        for(auto bound_action : upper_bounds) {
            if(bound_action.first>max_upper_bound) {
                max_upper_bound_actions.clear();
            }
            if(bound_action.first>=max_upper_bound) {
                max_upper_bound = bound_action.first;
                max_upper_bound_actions.push_back(bound_action.second);
            }
        }

        // random tie breaking between action with equal upper bound
        action_t action = random_select(max_upper_bound_actions);
        DEBUG_OUT(2,"Choosing action " << environment->action_name(action) << " with upper bound " << max_upper_bound );
        return action;
    }

    UCB1::UCB1(double Cp): Cp(Cp) {}

    reward_t UCB1::upper_bound(const node_t & state_node,
                               const arc_t & to_action_arc,
                               const node_t & action_node,
                               std::shared_ptr<const Environment> environment,
                               const graph_t & graph,
                               const node_info_map_t & node_info_map,
                               const mcts_node_info_map_t & mcts_node_info_map,
                               const mcts_arc_info_map_t & mcts_arc_info_map) const {

        // upper bound = value + 2 Cp sqrt( 2 log n / nj) where n and nj are the
        // counts of the state node and the arc to the action node,
        // respectively.
        return mcts_node_info_map[action_node].get_value() +
            2*Cp*sqrt(
                2*log(mcts_node_info_map[state_node].get_transition_counts())/
                mcts_arc_info_map[to_action_arc].get_counts()
                );
    }

    UCB_Plus::UCB_Plus(double Cp): Cp(Cp) {}

    reward_t UCB_Plus::upper_bound(const node_t & state_node,
                                   const arc_t & to_action_arc,
                                   const node_t & action_node,
                                   std::shared_ptr<const Environment> environment,
                                   const graph_t & graph,
                                   const node_info_map_t & node_info_map,
                                   const mcts_node_info_map_t & mcts_node_info_map,
                                   const mcts_arc_info_map_t & mcts_arc_info_map) const {

        // upper bound = value + Cp sqrt( value_variance / n) where n is the
        // number of times this action was taken.
        return mcts_node_info_map[action_node].get_value() +
            Cp*sqrt(
                mcts_node_info_map[action_node].get_value_variance()/
                mcts_arc_info_map[to_action_arc].get_counts()
                );
    }

} // end namespace tree_policy
