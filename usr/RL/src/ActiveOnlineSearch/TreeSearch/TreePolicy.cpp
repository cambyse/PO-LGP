#include "TreePolicy.h"

#include <unordered_set>
#include <vector>
#include <utility>

#include <util/util.h>
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

    Uniform::action_probability_t Uniform::get_action_probabilities(const node_t & state_node) const {
        auto actions = environment->get_actions();
        int action_n = actions.size();
        vector<double> probs(action_n,1./action_n);
        return action_probability_t(actions, probs);
    }

    action_handle_t Uniform::get_action(const node_t & state_node) const {
        action_handle_t action = random_select(environment->get_actions());
        DEBUG_OUT(1,"Select action: " << *action);
        return action;
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
#define FORCE_DEBUG_LEVEL 2
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
                DEBUG_EXPECT(0,old_actions==new_actions);
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
        // set scores to infinity for unsampled actions
        for(auto action : action_set) {
            scores.push_back(std::numeric_limits<double>::infinity());
            scored_actions.push_back(action);
        }

        // compute soft-max probabilities and return
        return action_probability_t(scored_actions, util::soft_max(scores,soft_max_temperature));
    }

    action_handle_t MaxPolicy::get_action(const node_t & state_node) const {
        RETURN_TUPLE(action_container_t, actions,
                     vector<double>, probs) = get_action_probabilities(state_node);
        IF_DEBUG(1) {
            DEBUG_OUT(1,"Action probabilities (node " << graph->id(state_node) << "):");
            for(int idx=0; idx<actions.size(); ++idx) {
                DEBUG_OUT(1,"    " << *(actions[idx]) << "	" << probs[idx] );
            }
        }
        int idx = util::random_select_idx(probs);
        return actions[idx];
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
        return (*mcts_node_info_map)[action_node].value +
            2*Cp*sqrt(
                2*log((*mcts_node_info_map)[state_node].action_counts)/
                (*mcts_arc_info_map)[to_action_arc].transition_counts
                );
    }

    UCB_Plus::UCB_Plus(double Cp): Cp(Cp) {}

    double UCB_Plus::score(const node_t & state_node,
                             const arc_t & to_action_arc,
                             const node_t & action_node) const {

        // upper bound = value + Cp sqrt( value_variance / n) where n is the
        // number of times this action was taken.
        return (*mcts_node_info_map)[action_node].value +
            Cp*sqrt((*mcts_node_info_map)[action_node].value_variance);
    }

    double HardUpper::score(const node_t & state_node,
                              const arc_t & to_action_arc,
                              const node_t & action_node) const {
        return (*mcts_node_info_map)[action_node].max_value;
    }

} // end namespace tree_policy
