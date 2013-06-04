#ifndef LOOKAHEADSEARCH_H_
#define LOOKAHEADSEARCH_H_

#include <lemon/list_graph.h>
#include <lemon/dim2.h>

#include "Data.h"
#include "Representation/Representation.h"

#define DEBUG_LEVEL 1
#include "debug.h"

class LookAheadSearch {

public:

    USE_DATA_TYPEDEFS;
    USE_REPRESENTATION_TYPEDEFS;
    typedef reward_t::value_t value_t;

    enum NODE_TYPE { NONE, STATE, ACTION };
    enum EXPANSION_TYPE { NOT_DEFINED, NOT_EXPANDED, FULLY_EXPANDED };

    struct NodeInfo {
        NodeInfo();
        NodeInfo(
                const NODE_TYPE&,
                const EXPANSION_TYPE&,
                const instance_t *,
                const action_t&,
                const value_t&,
                const value_t&,
                const bool& del = false
        );
        NodeInfo(const NodeInfo&);
        ~NodeInfo();
        NodeInfo& operator=(const NodeInfo&);
        NODE_TYPE type;
        EXPANSION_TYPE expansion;
        const instance_t * instance;
        action_t action;
        value_t upper_value_bound, lower_value_bound;
        bool delete_instance;
    };

    struct ArcInfo {
        ArcInfo(const reward_t& r = 0, const probability_t& p = 0): expected_reward(r), prob(p) {}
        reward_t expected_reward;
        probability_t prob;
    };

    typedef lemon::ListDigraph graph_t;
    typedef graph_t::Node node_t;
    typedef graph_t::Arc arc_t;
    typedef graph_t::NodeMap<NodeInfo> node_info_map_t;
    typedef graph_t::ArcMap<ArcInfo> arc_info_map_t;

    typedef std::vector<node_t> node_vector_t;

    LookAheadSearch(const double& d);
    virtual ~LookAheadSearch();

    /*! \brief Clears the search tree. */
    void clear_tree();

    /*! \brief Build a search tree from the root state. */
    template < class Model >
    void build_tree(
            const instance_t * root,
            const Model& model,
            probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const,
            const size_t& max_node_counter = 0
    );

    /*! \brief Expand current tree by expanding one leaf
     * node and back-propagating changes. Return whether
     * tree needs further expansion. */
    template < class Model >
    bool expand_tree(
            const Model& model,
            probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
    );

    /*! \brief Returns the best action for the root state. */
    action_t get_optimal_action() const;

    /*! \brief Set the discount rate used for computing state and action values. */
    void set_discount(const double& d) { discount = d; }

    /*! \brief Print the tree to console and/or as eps file. */
    void print_tree(const bool& text, const bool& eps_export) const;

    /*! \brief Print the tree statistics to console. */
    void print_tree_statistics() const;

protected:

    typedef lemon::dim2::Point<double> Point;

    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    arc_info_map_t arc_info_map;
    double discount;
    size_t number_of_nodes;

    /*! \brief Sets the way the upper and lower bounds
     * are used for node selection and back-propagation. */
    enum BOUND_USAGE_TYPE {

        /*! Use maximum upper bound. */
        MAX_UPPER_BOUND,

        /*! Use maximum lower bound. */
        MAX_LOWER_BOUND,

        /*! Use maximum weighted uncertainty. */
        MAX_WEIGHTED_UNCERTAINTY,

        /*! Use expected lower and upper bounds. */
        EXPECTED_BOUNDS,

        /*! Use maximum upper bound for the upper bound and maximum lower bound for lower bound. */
        MAX_UPPER_FOR_UPPER_MAX_LOWER_FOR_LOWER,

        /*! Use maximum upper bound for the upper bound and the corresponding lower bound for lower bound. */
        MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER,

        /*! Use weighted bounds for calculating value:
         * \f$ value = \alpha \cdot bound_{lower} + (1-\alpha) \cdot bound_{upper} \f$
         * with \f$\alpha = \f$ LookAheadSearch::lower_bound_weight.\n\n
         * For binary rewards \f$ r_{min} \f$ and \f$ r_{max} \f$ with
         * probabilities \f$ p_{min} \f$ and \f$ p_{max} \f$,
         * \f$ p_{min} \cdot r_{min} + p_{max} \cdot r_{max} \f$
         * corresponds to the the expected reward in every time step.
         * Given upper and lower bounds
         * \f$value = p_{min} \cdot bound_{lower} + (1-p_{min}) \cdot bound_{upper}\f$
         * equals the reinforcement value i.e. the expected reward discounted over time.*/
        MAX_WEIGHTED_BOUNDS,

        /*! Use the same strategy as is used for optimal action selection (defined by LookAheadSearch::optimal_action_selection_type). */
        SAME_AS_OPTIMAL_ACTION_SELECTION
    };

    /*! \brief Weight for lower bound in strategy
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS.
     *
     * For binary rewards this corresponds to the
     * prior probability of obtaining the lower reward. */
    static const double lower_bound_weight;

    /*! \brief Defines how the optimal action is determined.
     *
     * Currently LookAheadSearch::MAX_LOWER_BOUND and MAX_WEIGHTED_BOUNDS are supported. */
    static const BOUND_USAGE_TYPE optimal_action_selection_type = MAX_WEIGHTED_BOUNDS;

    /*! \brief Defines how action nodes are selected for further examination.
     *
     * Currently LookAheadSearch::MAX_UPPER_BOUND and LookAheadSearch::MAX_LOWER_BOUND are supported. */
    static const BOUND_USAGE_TYPE tree_action_selection_type    = MAX_UPPER_BOUND;

    /*! \brief Defines how state nodes are selected for further examination.
     *
     * Currently only LookAheadSearch::MAX_WEIGHTED_UNCERTAINTY is supported. */
    static const BOUND_USAGE_TYPE tree_state_selection_type     = MAX_WEIGHTED_UNCERTAINTY;

    /*! \brief Defines how state bounds are used for calculating
     * action bounds in back-propagation.
     *
     * Currently only LookAheadSearch::EXPECTED_BOUNDS is supported. */
    static const BOUND_USAGE_TYPE action_back_propagation_type  = EXPECTED_BOUNDS;

    /*! \brief Defines how action bounds are used for calculating
     * state bounds in back-propagation.
     *
     * Currently LookAheadSearch::MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER,
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS, and SAME_AS_OPTIMAL_SCTION_SELECTION
     * are supported (LookAheadSearch::optimal_action_selection_type
     * must match on of the implemented strategies). */
    static const BOUND_USAGE_TYPE state_back_propagation_type   = SAME_AS_OPTIMAL_ACTION_SELECTION;

    /*! \brief Select the next action node for finding the leaf that is to be expanded. */
    node_t select_next_action_node(node_t state_node);

    /*! \brief Select the next state node (possibly a leaf) for finding the leaf that is to be expanded. */
    node_t select_next_state_node(node_t action_node);

    /*! \brief Expand the leaf node given a predictive model. */
    template < class Model >
    void expand_leaf_node(
            node_t state_node,
            const Model& model,
            probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
    );

    /*! \brief Expand the action node given a predictive model. */
    template < class Model >
    void expand_action_node(
            node_t action_node,
            const Model& model,
            probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
    );

    /*! \brief Update the given action node when back-propagating after leaf expansion.
     * Return parent state node. */
    node_t update_action_node(node_t action_node);

    /*! \brief Update the given state node when back-propagating after leaf expansion.
     * Return parent action node. */
    node_t update_state_node(node_t state_node);

    /*! \brief Returns whether the root_node needs further expansion.
     *
     * If there is an action with lower value bound greater than the
     * upper value bounds of all other actions this action cannot
     * become sub-optimal by further expansion of the tree.
     */
    bool tree_needs_further_expansion();

    /*! \brief Upper bound for state value without prior knowledge. */
    value_t get_upper_value_bound() { return reward_t::max_reward/(1-discount); }

    /*! \brief Lower bound for state value without prior knowledge. */
    value_t get_lower_value_bound() { return reward_t::min_reward/(1-discount); }

    /*! \brief Print information on node. */
    void print_node(node_t node) const;

    /*! \brief Node energy function. */
    double node_energy(node_t node, const graph_t::NodeMap<Point>& coords) const;
};

//=================================================================//
//                    Function Definitions                         //
//=================================================================//

template < class Model >
void LookAheadSearch::build_tree(
        const instance_t * root_instance,
        const Model& model,
        probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const,
        const size_t& max_node_counter
) {

    DEBUG_OUT(2,"Building new search tree");

    // clear tree
    clear_tree();

    // add root node
    root_node = graph.addNode();
    ++number_of_nodes;
    node_info_map[root_node] = NodeInfo(
            STATE,
            NOT_EXPANDED,
            instance_t::create(root_instance->action, root_instance->state, root_instance->reward, root_instance->const_it()-1),
            action_t::NULL_ACTION,
            get_upper_value_bound(),
            get_lower_value_bound()
    );

    expand_leaf_node(root_node,model,prediction);
    update_state_node(root_node);

    // fully expand tree
    idx_t last_progress = -1;
    if(tree_needs_further_expansion()) {
        while(expand_tree(model,prediction)) {
            if(max_node_counter>0 && number_of_nodes>max_node_counter) {
                DEBUG_OUT(0,"Abort: Tree has more than " << max_node_counter << " nodes (" << number_of_nodes << ")");
                break;
            } else {
                if(DEBUG_LEVEL>=1) {
                    last_progress = util::print_progress(number_of_nodes, max_node_counter, 50, "Building Tree", last_progress);
                }
            }
        }
    }
    if(DEBUG_LEVEL>=1) {
        std::cout << std::endl;
    }

    if(DEBUG_LEVEL>=4) {
        print_tree(false,true);
    }
}

template < class Model >
bool LookAheadSearch::expand_tree(
        const Model& model,
        probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
) {

    node_t current_state_node = root_node;
    node_t current_action_node = lemon::INVALID;

    // find leaf
    while(node_info_map[current_state_node].expansion==FULLY_EXPANDED) {
        current_action_node = select_next_action_node(current_state_node);
        current_state_node = select_next_state_node(current_action_node);
    }

    // expand leaf
    expand_leaf_node(current_state_node, model, prediction);

    // back propagation
    while(current_state_node!=root_node) {
        current_action_node = update_state_node(current_state_node);
        current_state_node = update_action_node(current_action_node);
    }
    update_state_node(root_node);

    return tree_needs_further_expansion();
}

template < class Model >
void LookAheadSearch::expand_leaf_node(
        node_t state_node,
        const Model& model,
        probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
) {

    DEBUG_OUT(3,"Expanding leaf node");
    if(DEBUG_LEVEL>=3) {
        print_node(state_node);
    }

    if(node_info_map[state_node].type!=STATE) {
        DEBUG_OUT(0,"Error: trying to expand non-state node as state node");
    }
    if(node_info_map[state_node].expansion!=NOT_EXPANDED) {
        DEBUG_OUT(0,"Error: trying to expand state node with expansion other than NOT_EXPANDED");
    }

    const instance_t * instance_from = node_info_map[state_node].instance;

    // create action nodes
    for(actionIt_t action=actionIt_t::first(); action!=util::INVALID; ++action) {

        // create new action node
        node_t action_node = graph.addNode();
        ++number_of_nodes;
        node_info_map[action_node] = NodeInfo(
                ACTION,
                NOT_EXPANDED,
                instance_t::create(instance_from->action, instance_from->state, instance_from->reward, instance_from->const_it()-1),
                action,
                get_upper_value_bound(),
                get_lower_value_bound()
        );
        arc_t state_to_action_arc = graph.addArc(state_node,action_node);
        arc_info_map[state_to_action_arc] = ArcInfo(0,0); // never used

        DEBUG_OUT(4,"    Added action node:");
        if(DEBUG_LEVEL>=4) {
            print_node(action_node);
        }
    }

    // expand all actions
    for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=lemon::INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        expand_action_node(action_node, model, prediction);
        node_t test_state_node = update_action_node(action_node);
        if(test_state_node!=state_node) {
            DEBUG_OUT(0,"Error: Action node update does not return expected state node");
        }
    }

    // set to fully expanded
    node_info_map[state_node].expansion = FULLY_EXPANDED;
}

template < class Model >
void LookAheadSearch::expand_action_node(
        node_t action_node,
        const Model& model,
        probability_t(Model::*prediction)(const instance_t *, const action_t&, const state_t&, const reward_t&) const
) {

    DEBUG_OUT(3,"Expanding action node");
    if(DEBUG_LEVEL>=4) {
        print_node(action_node);
    }

    if(node_info_map[action_node].type!=ACTION) {
        DEBUG_OUT(0,"Error: trying to expand non-action node as action node");
    }
    if(node_info_map[action_node].expansion!=NOT_EXPANDED) {
        DEBUG_OUT(0,"Error: trying to expand action node with expansion other than NOT_EXPANDED");
    }

    const instance_t * instance_from = node_info_map[action_node].instance;
    action_t action = node_info_map[action_node].action;

    // add all target states (iteration in terms of MDP-state and reward)
    for(stateIt_t new_state=stateIt_t::first(); new_state!=util::INVALID; ++new_state) {

        node_t new_state_node = lemon::INVALID;

        for(rewardIt_t new_reward=rewardIt_t::first(); new_reward!=util::INVALID; ++new_reward) {

            probability_t prob = (model.*prediction)(instance_from, action, new_state, new_reward);
            if(prob>0) {
                new_state_node = graph.addNode();
                ++number_of_nodes;
                node_info_map[new_state_node] = NodeInfo(
                        STATE,
                        NOT_EXPANDED,
                        instance_t::create(action, new_state, new_reward, instance_from, nullptr),
                        action_t::NULL_ACTION, // not defined for state nodes
                        get_upper_value_bound(),
                        get_lower_value_bound()
                );
                arc_t action_to_state_arc = graph.addArc(action_node,new_state_node);
                arc_info_map[action_to_state_arc].prob = prob;
                arc_info_map[action_to_state_arc].expected_reward = new_reward;
            }
        }

        if(new_state_node!=lemon::INVALID){
            DEBUG_OUT(4,"    Added state node");
            if(DEBUG_LEVEL>=4) {
                print_node(new_state_node);
            }
        }
    }

    if(graph_t::OutArcIt(graph,action_node)==lemon::INVALID) {
        DEBUG_OUT(0,"Error: No possible state transition.")
    }

    node_info_map[action_node].expansion = FULLY_EXPANDED;
}

#include "debug_exclude.h"

#endif /* LOOKAHEADSEARCH_H_ */
