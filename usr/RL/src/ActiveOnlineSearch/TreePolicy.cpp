#include "TreePolicy.h"

#include "Environment.h"
#include "AbstractMonteCarloTreeSearch.h"

#include <util/util.h>

#define DEBUG_LEVEL 1
#include <util/debug.h>

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

    TreePolicy::TreePolicy(std::shared_ptr<const Environment> environment,
                           const graph_t & graph,
                           const node_info_map_t & node_info_map,
                           const mcts_node_info_map_t & mcts_node_info_map):
        environment(environment),
        graph(graph),
        node_info_map(node_info_map),
        mcts_node_info_map(mcts_node_info_map)
    {}


    Uniform::Uniform(std::shared_ptr<const Environment> environment,
                     const graph_t & graph,
                     const node_info_map_t & node_info_map,
                     const mcts_node_info_map_t & mcts_node_info_map):
        TreePolicy(environment, graph, node_info_map, mcts_node_info_map)
    {}

    action_t Uniform::next(const node_t & node) {
        action_t action = util::random_select(environment->actions);
        DEBUG_OUT(1,"Select action: " << environment->action_name(action));
        return action;

        // using return_tuple::t;

        // // select action node
        // std::vector<node_t> nodes;
        // for(out_arc_it_t arc(graph, node); arc!=lemon::INVALID; ++arc) {
        //     nodes.push_back(graph.target(arc));
        // }
        // node_t action_node = util::random_select(nodes);
        // action_t action = node_info_map[action_node].action;

        // // sample state
        // state_t state_from = node_info_map[node].state;
        // state_t state_to;
        // reward_t reward;
        // t(state_to,reward) = environment->sample(state_from, action);

        // // find state node
        // for(out_arc_it_t arc(graph, action_node); arc!=lemon::INVALID; ++arc) {
        //     node_t state_node = graph.target(arc);
        //     if(node_info_map[state_node].state==state_to) {
        //         return state_node;
        //     }
        // }
        // DEBUG_DEAD_LINE;
        // return lemon::INVALID;
    }

} // end namespace tree_policy
