#include "TreePolicy.h"

#include <unordered_set>
#include <vector>
#include <utility>

#include <util/util.h>

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

    action_handle_t Uniform::get_action(const node_t & state_node) const {
        if(restrict_to_existing || out_arc_it_t arc(*graph,state_node)==INVALID) {
            vector<action_handle_t> existing_actions;
            for(out_arc_it_t arc(*graph,state_node); arc!=INVALID; ++arc) {
                existing_actions.push_back((*node_info_map)[graph->target(arc)].action);
            }
            return random_select(existing_actions);
        } else {
            action_handle_t action = random_select(environment->get_actions());
            DEBUG_OUT(1,"Select action: " << *action);
            return action;
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

    action_handle_t MaxPolicy::get_action(const node_t & state_node) const {

        // get set of actions
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
                DEBUG_EXPECT(0,old_actions==new_actions);
#endif
        } else {
            actions = environment->get_actions();
            (*available_actions)[state_node] = actions;
        }

        unordered_set<action_handle_t,
                      AbstractEnvironment::ActionHash,
                      AbstractEnvironment::ActionEq> action_set(actions.begin(), actions.end());

        // prepare vector for computing upper bounds
        vector<pair<reward_t,action_handle_t>> scores;

        // compute upper bounds
        DEBUG_OUT(3,"Computing upper bound for state node " << graph->id(state_node));
        for(out_arc_it_t to_action_arc(*graph, state_node); to_action_arc!=INVALID; ++to_action_arc) {
            node_t action_node = graph->target(to_action_arc);
            action_handle_t action = (*node_info_map)[action_node].action;
            reward_t upper = score(state_node, to_action_arc, action_node);
            scores.push_back(make_pair(upper,action));
            // erase this action from set
            action_set.erase(action);
        }

        // select unsampled action if there are any left (and choice is not
        // restricted to existing actions)
        if(action_set.size()>0 && !restrict_to_existing) {
#ifdef UNIT_TESTS
            action_handle_t action = *(action_set.begin());
#else
            action_handle_t action = random_select(action_set);
#endif
            DEBUG_OUT(2,"Selecting unsampled action: " << *action);
            if(print_choice) {
                cout << "Select " << *action << " (unsampled)" << endl;
            }
            return action;
        } else {
            DEBUG_OUT(2,"No unsampled actions (or restricted to existing).");
        }

        // select max upper bound action otherwise
        IF_DEBUG(3) {
            DEBUG_OUT(3,"Use upper bound to choose between:");
            for(auto bound_action : scores) {
                DEBUG_OUT(3,"    '" << *(bound_action.second) << "' with bound " << bound_action.first);
            }
        }
        DEBUG_EXPECT(1,scores.size()>0);
        reward_t max_score = std::numeric_limits<reward_t>::lowest();
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
#ifdef UNIT_TESTS
        // not random tie breaking for unit tests
        action_handle_t action = max_score_actions.back();
#else
        action_handle_t action = random_select(max_score_actions);
#endif
        DEBUG_OUT(2,"Choosing action " << *action << " with upper bound " << max_score );
        if(print_choice) {
            cout << "Select " << *action << " (score=" << max_score << ")" << endl;
        }
        return action;
    }

    reward_t Optimal::score(const node_t & state_node,
                            const arc_t & to_action_arc,
                            const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].value;
    }

    UCB1::UCB1(double Cp): Cp(Cp) {}

    reward_t UCB1::score(const node_t & state_node,
                         const arc_t & to_action_arc,
                         const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].value +
            2*Cp*sqrt(
                2*log((*mcts_node_info_map)[state_node].action_counts)/
                (*mcts_arc_info_map)[to_action_arc].transition_counts
                );
    }

    UCB_Plus::UCB_Plus(double Cp): Cp(Cp) {}

    reward_t UCB_Plus::score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const {

        // upper bound = value + Cp sqrt( value_variance / n) where n is the
        // number of times this action was taken.
        return (*mcts_node_info_map)[action_node].value +
            Cp*sqrt((*mcts_node_info_map)[action_node].value_variance);
    }

    reward_t HardUpper::score(const node_t & state_node,
                              const arc_t & to_action_arc,
                              const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].max_value;
    }

} // end namespace tree_policy
