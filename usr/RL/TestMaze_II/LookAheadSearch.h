#ifndef LOOKAHEADSEARCH_H_
#define LOOKAHEADSEARCH_H_

#include <lemon/list_graph.h>
#include <lemon/graph_to_eps.h>
#include <lemon/dim2.h>
#include <lemon/connectivity.h>
#include <lemon/adaptors.h>

#include "Config.h"
#include "util/ProgressBar.h"
#include "Maze.h"

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include "debug.h"

class LookAheadSearch {

public:

    USE_CONFIG_TYPEDEFS;
    typedef reward_t::value_t value_t;

    enum NODE_TYPE { NONE, OBSERVATION, ACTION };
    enum EXPANSION_TYPE { NOT_DEFINED, NOT_EXPANDED, FULLY_EXPANDED };

    struct NodeInfo {
        NodeInfo();
        NodeInfo(
                const NODE_TYPE&,
                const EXPANSION_TYPE&,
                instance_t *,
                const action_t&,
                const value_t&,
                const value_t&
        );
        NodeInfo(const NodeInfo&);
        ~NodeInfo();
        NodeInfo& operator=(const NodeInfo&);
        NODE_TYPE type;
        EXPANSION_TYPE expansion;
        instance_t * instance; // !!!Needs to be set and deleted manually!!!
        action_t action;
        value_t upper_value_bound, lower_value_bound;
    };

    struct ArcInfo {
        ArcInfo(const reward_t& r = 0, const probability_t& p = 0): transition_reward(r), prob(p) {}
        reward_t transition_reward;
        probability_t prob;
    };

    typedef lemon::ListDigraph                graph_t;
    typedef graph_t::Node                     node_t;
    typedef graph_t::Arc                      arc_t;
    typedef graph_t::NodeMap<NodeInfo>        node_info_map_t;
    typedef graph_t::ArcMap<ArcInfo>          arc_info_map_t;
    typedef graph_t::NodeMap<lemon::Color>    node_color_map_t;

    typedef std::vector<node_t> node_vector_t;

    LookAheadSearch(const double& d);
    virtual ~LookAheadSearch();

    /*! \brief Clears the search tree. */
    void clear_tree();

    /*! \brief Build a search tree from the root observation. */
    template < class Model >
    void build_tree(
            const instance_t * root,
            const Model& model,
            const size_t& max_node_counter = 0
    );

    /*! \brief Expand current tree by expanding one leaf
     * node and back-propagating changes. Return whether
     * tree needs further expansion. */
    template < class Model >
    bool expand_tree(const Model&);

    /*! \brief Repeatedly expand tree until either the optimal action is
     *  unambiguous or the maximum tree size is reached. */
    template < class Model >
    void fully_expand_tree(
            const Model& model,
            const size_t& max_node_counter = 0
    );

    /*! \brief Returns the best action for the root observation. */
    action_t get_optimal_action() const;

    /*! \brief Returns the predicted transition probability for given action to
     *  given observation and reward. */
    template < class Model >
    probability_t get_predicted_transition_probability(const action_t&,
                                                       const observation_t&,
                                                       const reward_t& ,
                                                       const Model& model
        ) const;

    /*! \brief Prune obsolete branches after performing action a into observation s
     *  and reset root node. */
    template < class Model >
    void prune_tree(const action_t& a, const instance_t * new_root_instance, const Model& model);

    /*! \brief Set the discount rate used for computing observation and action values. */
    void set_discount(const double& d) { discount = d; }

    /*! \brief Print the tree to console and/or as eps file. */
    void print_tree(const bool& text,
                    const bool& eps_export,
                    const char* file_name = "look_ahead_tree.eps",
                    const node_color_map_t * color_map = nullptr
        ) const;

    /*! \brief Print the tree statistics to console. */
    void print_tree_statistics();

    size_t get_number_of_nodes() const { return number_of_nodes; }

protected:

    typedef lemon::dim2::Point<double> Point;

    graph_t graph;
    node_t root_node;
    node_info_map_t node_info_map;
    arc_info_map_t arc_info_map;
    double discount;
    size_t number_of_nodes;
    static const bool random_tie_break;

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

        /*! Use maximum upper bound for the upper bound and maximum lower bound
         *  for lower bound. This is the 'optimistic' or best-case assumption
         *  for estimating the bounds. */
        MAX_UPPER_FOR_UPPER_MAX_LOWER_FOR_LOWER,

        /*! Use maximum upper bound for the upper bound and minimum lower bound
         *  for lower bound. This is the worst-case assumption in terms of
         *  certainty, the bounds are as wide as possible. */
        MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER,

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

    /*! \brief Defines how observation nodes are selected for further examination.
     *
     * Currently only LookAheadSearch::MAX_WEIGHTED_UNCERTAINTY is
     * supported. LookAheadSearch::MAX_WEIGHTED_UNCERTAINTY means that the
     * uncertainty of each observation is multiplied with the probability of reaching
     * this observation by executing the action. This product -- the weighted
     * uncertainty -- is evaluated to determine the observation with a maximum
     * weighted uncertainty, which is further explored. */
    static const BOUND_USAGE_TYPE tree_observation_selection_type     = MAX_WEIGHTED_UNCERTAINTY;

    /*! \brief Defines how observation bounds are used for calculating
     * action bounds in back-propagation.
     *
     * Currently only LookAheadSearch::EXPECTED_BOUNDS and
     * LookAheadSearch::MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER are
     * supported. */
    static const BOUND_USAGE_TYPE action_back_propagation_type  = EXPECTED_BOUNDS;

    /*! \brief Defines how action bounds are used for calculating
     * observation bounds in back-propagation.
     *
     * Currently
     * LookAheadSearch::MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER,
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS, and
     * LookAheadSearch::SAME_AS_OPTIMAL_ACTION_SELECTION are supported
     * (LookAheadSearch::optimal_action_selection_type must match on of the
     * implemented strategies). In the case of
     * LookAheadSearch::MAX_WEIGHTED_BOUNDS the action node with highes weighted
     * bounds is chosen and the bounds of this action node are used for the
     * observation node. */
    static const BOUND_USAGE_TYPE observation_back_propagation_type   = SAME_AS_OPTIMAL_ACTION_SELECTION;

    /*! \brief Select the next action node for finding the leaf that is to be expanded. */
    node_t select_next_action_node(node_t observation_node);

    /*! \brief Select the next observation node (possibly a leaf) for finding the leaf that is to be expanded. */
    node_t select_next_observation_node(node_t action_node);

    /*! \brief Expand the leaf node given a predictive model. */
    template < class Model >
    void expand_leaf_node(
            node_t observation_node,
            const Model& model
    );

    /*! \brief Expand the action node given a predictive model. */
    template < class Model >
    void expand_action_node(
            node_t action_node,
            const Model& model
    );

    /** \brief Select the optimal action from a given observation node. */
    node_vector_t optimal_action_nodes(const node_t& observation_node) const;

    /*! \brief Update the given action node when back-propagating after leaf expansion.
     * Return parent observation node. */
    node_t update_action_node(node_t action_node);

    /*! \brief Update the given observation node when back-propagating after leaf expansion.
     * Return parent action node. */
    node_t update_observation_node(node_t observation_node);

    /*! \brief Returns whether the root_node needs further expansion.
     *
     * If there is an action with lower value bound greater than the
     * upper value bounds of all other actions this action cannot
     * become sub-optimal by further expansion of the tree.
     */
    bool tree_needs_further_expansion();

    /*! \brief Upper bound for observation value without prior knowledge. */
    value_t get_upper_value_bound() { return reward_t::max_reward/(1-discount); }

    /*! \brief Lower bound for observation value without prior knowledge. */
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
        const size_t& max_node_counter
) {

    DEBUG_OUT(2,"Building new search tree");

    // clear tree
    clear_tree();

    // add root node
    root_node = graph.addNode();
    ++number_of_nodes;
    node_info_map[root_node] = NodeInfo(
            OBSERVATION,
            NOT_EXPANDED,
            instance_t::create(root_instance->action, root_instance->observation, root_instance->reward, root_instance->const_it()-1),
            action_t::NULL_ACTION,
            get_upper_value_bound(),
            get_lower_value_bound()
    );

    expand_leaf_node(root_node,model);
    update_observation_node(root_node);

    // fully expand tree
    fully_expand_tree(model, max_node_counter);
}

template < class Model >
bool LookAheadSearch::expand_tree(const Model& model) {

    node_t current_observation_node = root_node;
    node_t current_action_node = lemon::INVALID;

    // find leaf
    while(node_info_map[current_observation_node].expansion==FULLY_EXPANDED) {
        current_action_node = select_next_action_node(current_observation_node);
        current_observation_node = select_next_observation_node(current_action_node);
    }

    // expand leaf
    expand_leaf_node(current_observation_node, model);

    // back propagation
    while(current_observation_node!=root_node) {
        current_action_node = update_observation_node(current_observation_node);
        current_observation_node = update_action_node(current_action_node);
    }
    update_observation_node(root_node);

    return tree_needs_further_expansion();
}

template < class Model >
void LookAheadSearch::fully_expand_tree(
        const Model& model,
        const size_t& max_node_counter
) {

    // fully expand tree
    if(DEBUG_LEVEL>=1) {
        ProgressBar::init("Building Tree: ");
    }
    if(tree_needs_further_expansion()) {
        while(expand_tree(model)) {
            if(max_node_counter>0 && number_of_nodes>max_node_counter) {
                DEBUG_OUT(1,"Abort: Tree has more than " << max_node_counter << " nodes (" << number_of_nodes << ")");
                break;
            } else {
                if(DEBUG_LEVEL>=1) {
                    ProgressBar::print(number_of_nodes, max_node_counter);
                }
            }
        }
    }
    if(DEBUG_LEVEL>=1) {
        ProgressBar::terminate();
    }

    if(DEBUG_LEVEL>=4) {
        print_tree(false,true);
    }
}

template < class Model >
LookAheadSearch::probability_t LookAheadSearch::get_predicted_transition_probability(const action_t& action,
                                                                                     const observation_t& observation,
                                                                                     const reward_t& reward,
                                                                                     const Model& model
    ) const {

    DEBUG_OUT(2,"Get predicted transition probability for (" << action << "," << observation << "," << reward << ")" );

    // find action node
    node_t action_node = lemon::INVALID;
    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=lemon::INVALID; ++out_arc) {
        action_node = graph.target(out_arc);
        if(node_info_map[action_node].action==action) {
            break;
        }
    }
    if(action_node==lemon::INVALID) {
        DEBUG_OUT(0,"Error: Action " << action << " not available from root node");
        return 0;
    }

    // find observation node
    node_t observation_node = lemon::INVALID;
    arc_t observation_node_in_arc = lemon::INVALID;
    for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=lemon::INVALID; ++out_arc) {
        node_t tmp_observation_node = graph.target(out_arc);
        if(node_info_map[tmp_observation_node].instance->observation==observation &&
           node_info_map[tmp_observation_node].instance->reward==reward) {
            observation_node = tmp_observation_node;
            observation_node_in_arc = out_arc;
            if(DEBUG_LEVEL==0) {
                break;
            }
        }
        DEBUG_OUT(2,"        Found observation " << node_info_map[tmp_observation_node].instance->observation <<
                  ", reward " << node_info_map[tmp_observation_node].instance->reward <<
                  ", prob " << arc_info_map[out_arc].prob);
    }
    if(observation_node==lemon::INVALID) {
        probability_t prob = model.get_prediction(node_info_map[root_node].instance, action, observation, reward);
        DEBUG_OUT(0,"Error: Node with observation " << observation << ", reward " << reward << " could not be found (Maze probability: " << prob << ")" );
        return 0;
    }

    // return transition probability
    return arc_info_map[observation_node_in_arc].prob;
}

template < class Model >
void LookAheadSearch::prune_tree(const action_t& a, const instance_t * new_root_instance, const Model& model) {

    // get undirected graph for using standard algorithms
    typedef lemon::Undirector<graph_t> ugraph_t;
    ugraph_t ugraph(graph);

    // some variables
    std::vector<node_t> nodes_to_delete;
    node_t new_root_node;

    DEBUG_OUT(2,"Pruning tree...");

    // identify action node
    graph_t::OutArcIt arc_to_action;
    node_t action_node;
    for(arc_to_action = graph_t::OutArcIt(graph,root_node); arc_to_action!=lemon::INVALID; ++arc_to_action) {
        action_node = graph.target(arc_to_action);
        if(node_info_map[action_node].action==a) {
            DEBUG_OUT(2,"    Found chosen action node (" << graph.id(action_node) << ")");
            break;
        }
    }
    if(arc_to_action==lemon::INVALID) {
        DEBUG_OUT(0,"Error: Could not identify choosen action");
        clear_tree();
        root_node = graph.addNode();
        ++number_of_nodes;
        node_info_map[root_node] = NodeInfo(
            OBSERVATION,
            NOT_EXPANDED,
            instance_t::create(new_root_instance->action, new_root_instance->observation, new_root_instance->reward, new_root_instance->const_it()-1),
            action_t::NULL_ACTION,
            get_upper_value_bound(),
            get_lower_value_bound()
            );
        return;
    }

    // find new root node
    observation_t observation = new_root_instance->observation;
    reward_t reward = new_root_instance->reward;
    graph_t::OutArcIt arc_to_observation;
    for(arc_to_observation = graph_t::OutArcIt(graph,action_node); arc_to_observation!=lemon::INVALID; ++arc_to_observation) {
        node_t observation_node = graph.target(arc_to_observation);
        if(node_info_map[observation_node].instance->observation==observation &&
           node_info_map[observation_node].instance->reward==reward) {
            DEBUG_OUT(2,"    Found new root node (" << graph.id(observation_node) << ")");
            new_root_node=observation_node;
            break;
        }
    }
    if(arc_to_observation==lemon::INVALID) {
        DEBUG_OUT(0,"Error: Could not identify new root node");
        if(DEBUG_LEVEL>0) {
            DEBUG_OUT(0,"    Need observation " << observation << ", reward " << reward);
            for(arc_to_observation = graph_t::OutArcIt(graph,action_node); arc_to_observation!=lemon::INVALID; ++arc_to_observation) {
                node_t observation_node = graph.target(arc_to_observation);
                DEBUG_OUT(0,"        Found observation " << node_info_map[observation_node].instance->observation <<
                          ", reward " << node_info_map[observation_node].instance->reward );
            }
            DEBUG_OUT(0,"    Old root instance " );
            for( const_instanceIt_t old_instance(node_info_map[root_node].instance); old_instance!=util::INVALID; --old_instance) {
                DEBUG_OUT(0,"        " << *old_instance );
            }
            DEBUG_OUT(0,"    New root instance " );
            for( const_instanceIt_t new_instance(new_root_instance); new_instance!=util::INVALID; --new_instance) {
                DEBUG_OUT(0,"        " << *new_instance );
            }
            // print_tree(false,true,"pruning_tree_error.eps");
            // clear_tree();
            // root_node = graph.addNode();
            // ++number_of_nodes;
            // node_info_map[root_node] = NodeInfo(
            //     OBSERVATION,
            //     NOT_EXPANDED,
            //     instance_t::create(new_root_instance->action, new_root_instance->observation, new_root_instance->reward, new_root_instance->const_it()-1),
            //     action_t::NULL_ACTION,
            //     get_upper_value_bound(),
            //     get_lower_value_bound()
            //     );
        }
        return;
    }

    // remember successor observations and other data for debugging purposes
    std::vector<std::tuple<node_t,ArcInfo> > successor_observations;
    NodeInfo action_node_info;
    ArcInfo arc_to_action_info;
    for(graph_t::OutArcIt arc_to_observation(graph,action_node); arc_to_observation!=lemon::INVALID; ++arc_to_observation) {
        node_t successor = graph.target(arc_to_observation);
        successor_observations.push_back(std::make_tuple(successor,arc_info_map[arc_to_observation]));
    }
    action_node_info = node_info_map[action_node];
    arc_to_action_info = arc_info_map[graph_t::InArcIt(graph,action_node)];

    // remove selected action node from tree to split into two components (we
    // don't need to worry about arcs since they are erased along with the
    // corresponding nodes)
    graph.erase(action_node);

    // identify and remember nodes that are not in the same component as the new
    // root node
    node_color_map_t pruning_map(graph);
    ugraph_t::NodeMap<int> component_map(ugraph);
    int component_n = lemon::connectedComponents(ugraph,component_map);
    if(component_n<2) {
        DEBUG_OUT(0,"Error: Search tree was not split by removing chosen action node");
        return;
    } else {
        DEBUG_OUT(2,"    " << component_n << " connected components");
    }
    int main_component = component_map[new_root_node];
    DEBUG_OUT(2,"    main component: " << main_component);
    for(ugraph_t::NodeIt node(ugraph); node!=lemon::INVALID; ++node) {
        if(component_map[node]!=main_component) {
            nodes_to_delete.push_back(node); // remember
            DEBUG_OUT(3,"    node " << graph.id(node) << " NOT in main component");
            pruning_map[node] = lemon::Color(1,0.5,0.5);
        } else {
            DEBUG_OUT(3,"    node " << graph.id(node) << " IS in main component");
            pruning_map[node] = lemon::Color(0.5,1,0.5);
        }
    }

    // print the pruning tree to a file
    if(DEBUG_LEVEL>2) {
        node_t tmp_action_node = graph.addNode();
        arc_t tmp_arc = graph.addArc(root_node,tmp_action_node);
        node_info_map[tmp_action_node] = action_node_info;
        arc_info_map[tmp_arc] = arc_to_action_info;
        pruning_map[tmp_action_node] = lemon::Color(0.5,0.5,1);
        for( auto successor : successor_observations ) {
            arc_t arc = graph.addArc(tmp_action_node, std::get<0>(successor));
            arc_info_map[arc] = std::get<1>(successor);
        }
        print_tree(false,true,"pruning_tree.eps",&pruning_map);
        graph.erase(tmp_action_node);
    }

    // erase nodes that are not in the main component
    for(node_t node : nodes_to_delete) {
        if(node_info_map[node].type==OBSERVATION) {
            delete node_info_map[node].instance;
        }
        graph.erase(node);
    }

    // update root node
    if(DEBUG_LEVEL>=1) {
        // sanity check
        instance_t * ins = node_info_map[new_root_node].instance;
        if(new_root_instance->action!=ins->action ||
           new_root_instance->observation!=ins->observation ||
           new_root_instance->reward!=ins->reward) {
            DEBUG_OUT(0,"Error: Old and new instance of new root node do not match");
            DEBUG_OUT(0,"    old: " << *ins << ", new: " << *new_root_instance);
        }
    }
    root_node = new_root_node;
    instance_t * new_root_instance_copy = instance_t::create(new_root_instance->action, new_root_instance->observation, new_root_instance->reward, new_root_instance->const_it()-1);
    *(node_info_map[root_node].instance) = *new_root_instance_copy;
    delete new_root_instance_copy;

    // update number of nodes
    DEBUG_OUT(3,"    Updating number of nodes...");
    number_of_nodes = 0;
    for(graph_t::NodeIt node(graph); node!=lemon::INVALID; ++node) {
        ++number_of_nodes;
    }
    DEBUG_OUT(3,"        " << number_of_nodes << " nodes");

    // make sure root node is expanded
    switch(node_info_map[root_node].expansion) {
    case FULLY_EXPANDED:
        // everythin fine
        break;
    case NOT_EXPANDED:
        DEBUG_OUT(1,"Expanding root node");
        expand_leaf_node(root_node, model);
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    // check graph structure
    if(!lemon::tree(ugraph)) {
        DEBUG_OUT(0,"Error: Search graph is not a tree");
    }

    DEBUG_OUT(2,"DONE");
}

template < class Model >
void LookAheadSearch::expand_leaf_node(
        node_t observation_node,
        const Model& model
) {

    DEBUG_OUT(3,"Expanding leaf node");
    if(DEBUG_LEVEL>=3) {
        print_node(observation_node);
    }

    if(node_info_map[observation_node].type!=OBSERVATION) {
        DEBUG_OUT(0,"Error: trying to expand non-observation node as observation node");
    }
    if(node_info_map[observation_node].expansion!=NOT_EXPANDED) {
        DEBUG_OUT(0,"Error: trying to expand observation node with expansion other than NOT_EXPANDED");
    }

    instance_t * instance_from = node_info_map[observation_node].instance;

    // create action nodes
    for(actionIt_t action=actionIt_t::first(); action!=util::INVALID; ++action) {
        node_t action_node = graph.addNode();
        ++number_of_nodes;
        node_info_map[action_node] = NodeInfo(
                ACTION,
                NOT_EXPANDED,
                instance_from, // use instance from parent observation node
                action,
                get_upper_value_bound(),
                get_lower_value_bound()
        );
        arc_t observation_to_action_arc = graph.addArc(observation_node,action_node);
        arc_info_map[observation_to_action_arc] = ArcInfo(0,0); // never used

        DEBUG_OUT(4,"    Added action node:");
        if(DEBUG_LEVEL>=4) {
            print_node(action_node);
        }
    }

    // expand and update all newly added action nodes
    for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=lemon::INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        expand_action_node(action_node, model);
        update_action_node(action_node);
    }

    // set to fully expanded
    node_info_map[observation_node].expansion = FULLY_EXPANDED;
}

template < class Model >
void LookAheadSearch::expand_action_node(
        node_t action_node,
        const Model& model
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

    // add all target observations (MDP-observation-reward combinations)
    for(observation_t new_observation : observationIt_t::all) {

        node_t new_observation_node = lemon::INVALID;

        for(reward_t new_reward : rewardIt_t::all) {

            probability_t prob = model.get_prediction(instance_from, action, new_observation, new_reward);
            if(prob>0) {
                new_observation_node = graph.addNode();
                ++number_of_nodes;
                node_info_map[new_observation_node] = NodeInfo(
                        OBSERVATION,
                        NOT_EXPANDED,
                        instance_t::create(action, new_observation, new_reward, instance_from, nullptr),
                        action_t::NULL_ACTION, // not defined for observation nodes
                        get_upper_value_bound(),
                        get_lower_value_bound()
                );
                arc_t action_to_observation_arc = graph.addArc(action_node,new_observation_node);
                arc_info_map[action_to_observation_arc].prob = prob;
                arc_info_map[action_to_observation_arc].transition_reward = new_reward;
            }
        }

        if(new_observation_node!=lemon::INVALID){
            DEBUG_OUT(4,"    Added observation node");
            if(DEBUG_LEVEL>=4) {
                print_node(new_observation_node);
            }
        }
    }

    if(graph_t::OutArcIt(graph,action_node)==lemon::INVALID) {
        DEBUG_OUT(0,"Error: No possible observation transition.")
    }

    node_info_map[action_node].expansion = FULLY_EXPANDED;
}

#include "debug_exclude.h"

#endif /* LOOKAHEADSEARCH_H_ */
