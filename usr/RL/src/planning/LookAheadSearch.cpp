#include "LookAheadSearch.h"

#include <unistd.h> // for sleep()
#include <float.h>
#include <math.h>
#include <vector>
#include <tuple>

#include <util/util.h>
#include <Maze/Maze.h>
#include <environment/Environment.h>
#include <representation/DoublyLinkedInstance.h>

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#include <util/debug.h>

using lemon::INVALID;
using lemon::Color;
using util::approx_eq;
using std::vector;
using std::tuple;
using std::make_tuple;
using std::get;

#ifdef NO_RANDOM
const bool LookAheadSearch::random_tie_break = false;
#else
const bool LookAheadSearch::random_tie_break = true;
#endif

const double LookAheadSearch::lower_bound_weight = 0.5;

LookAheadSearch::NodeInfo::NodeInfo():
    type(NONE),
    expansion(NOT_DEFINED),
    instance(nullptr),
    action(action_ptr_t()),
    upper_value_bound(0),
    lower_value_bound(0)
{}

LookAheadSearch::NodeInfo::NodeInfo(
    const NODE_TYPE& t,
    const EXPANSION_TYPE& e,
    instance_ptr_t i,
    const action_ptr_t& a,
    const value_t& uv,
    const value_t& lv
    ):
    type(t),
    expansion(e),
    instance(i),
    action(a),
    upper_value_bound(uv),
    lower_value_bound(lv)
{
    if(lower_value_bound>upper_value_bound) {
        DEBUG_ERROR("Lower value bound above upper value bound");
    }
}

LookAheadSearch::NodeInfo::NodeInfo(const NodeInfo& other):
    type(other.type),
    expansion(other.expansion),
    instance(other.instance),
    action(other.action),
    upper_value_bound(other.upper_value_bound),
    lower_value_bound(other.lower_value_bound)
{}

LookAheadSearch::NodeInfo::~NodeInfo() {}

LookAheadSearch::NodeInfo& LookAheadSearch::NodeInfo::operator=(const NodeInfo& other) {
    type = other.type;
    expansion = other.expansion;
    instance = other.instance;
    action = other.action;
    upper_value_bound = other.upper_value_bound;
    lower_value_bound = other.lower_value_bound;
    return (*this);
}

LookAheadSearch::LookAheadSearch(const double& d):
    root_node(INVALID),
    node_info_map(graph),
    arc_info_map(graph),
    discount(d),
    number_of_nodes(0)
{}

LookAheadSearch::~LookAheadSearch() {}

void LookAheadSearch::clear_tree() {
    graph.clear();
    number_of_nodes = 0;
    root_node = INVALID;
}

void LookAheadSearch::build_tree(
        const_instance_ptr_t root_instance,
        const Predictor& environment,
        const large_size_t& max_node_counter
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
            DoublyLinkedInstance::create(root_instance->action,
                                         root_instance->observation,
                                         root_instance->reward,
                                         root_instance->const_prev(),
                                         util::INVALID),
            action_ptr_t(),
            get_upper_value_bound(),
            get_lower_value_bound()
    );

    expand_leaf_node(root_node,environment);
    update_observation_node(root_node);

    // fully expand tree
    fully_expand_tree(environment, max_node_counter);
}

bool LookAheadSearch::expand_tree(const Predictor& environment) {

    node_t current_observation_node = root_node;
    node_t current_action_node = lemon::INVALID;

    // find leaf
    while(node_info_map[current_observation_node].expansion==FULLY_EXPANDED) {
        current_action_node = select_next_action_node(current_observation_node);
        current_observation_node = select_next_observation_node(current_action_node);
    }

    // expand leaf
    expand_leaf_node(current_observation_node, environment);

    // back propagation
    while(current_observation_node!=root_node) {
        current_action_node = update_observation_node(current_observation_node);
        current_observation_node = update_action_node(current_action_node);
    }
    update_observation_node(root_node);

    return tree_needs_further_expansion();
}

void LookAheadSearch::fully_expand_tree(
        const Predictor& environment,
        const large_size_t& max_node_counter
) {

    // fully expand tree
    if(DEBUG_LEVEL>=1) {
        ProgressBar::init("Building Tree: ");
    }
    bool max_node_number_exceeded = false;
    if(tree_needs_further_expansion()) {
        while(expand_tree(environment)) {
            if(max_node_counter>0 && number_of_nodes>max_node_counter) {
                if(DEBUG_LEVEL>0) {
                    max_node_number_exceeded = true;
                    ProgressBar::terminate();
                    std::cout << "Abort: Tree has more than " << max_node_counter << " nodes (" << number_of_nodes << ")" << std::endl;
                }
                break;
            } else {
                if(DEBUG_LEVEL>=1) {
                    ProgressBar::print(number_of_nodes, max_node_counter);
                }
            }
        }
    }
    if(DEBUG_LEVEL>=1 && !max_node_number_exceeded) {
        ProgressBar::terminate();
    }

    if(DEBUG_LEVEL>=4) {
        print_tree(false,true);
    }
}

LookAheadSearch::action_ptr_t LookAheadSearch::get_optimal_action() const {
    DEBUG_OUT(2,"Determining optimal action");

    node_vector_t optimal_action_node_vector = optimal_action_nodes(root_node);

    // select one action randomly
    action_ptr_t selected_action = action_ptr_t();
    if(optimal_action_node_vector.size()==0) {
        DEBUG_ERROR("No action available from root node. Choosing " << selected_action << ".");
        return selected_action;
    } else {
        // random select action
        node_t selected_action_node;
        if(random_tie_break) {
            selected_action_node = util::random_select(optimal_action_node_vector);
        } else {
            selected_action_node = optimal_action_node_vector.front();
        }

        // check
        if(node_info_map[selected_action_node].type!=ACTION) {
            DEBUG_ERROR("Next action node is not of type ACTION");
        }

        // get action
        selected_action = node_info_map[selected_action_node].action;
        DEBUG_OUT(3, "    choosing " << selected_action << " from " << optimal_action_node_vector.size() << " actions");

        return selected_action;
    }
}

LookAheadSearch::probability_t LookAheadSearch::get_predicted_transition_probability(const action_ptr_t& action,
                                                                                     const observation_ptr_t& observation,
                                                                                     const reward_ptr_t& reward,
                                                                                     const Predictor& environment
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
        DEBUG_ERROR("Action " << action << " not available from root node");
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
        probability_t prob = environment.get_prediction(node_info_map[root_node].instance, action, observation, reward);
        DEBUG_ERROR("Node with observation " << observation << ", reward " << reward << " could not be found (Maze probability: " << prob << ")" );
        return 0;
    }

    // return transition probability
    return arc_info_map[observation_node_in_arc].prob;
}

void LookAheadSearch::prune_tree(const action_ptr_t& a, const_instance_ptr_t new_root_instance, const Predictor& environment) {

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
        DEBUG_ERROR("Could not identify choosen action");
        clear_tree();
        root_node = graph.addNode();
        ++number_of_nodes;
        node_info_map[root_node] = NodeInfo(
            OBSERVATION,
            NOT_EXPANDED,
            DoublyLinkedInstance::create(new_root_instance->action,
                                         new_root_instance->observation,
                                         new_root_instance->reward,
                                         new_root_instance->const_prev(),
                                         util::INVALID),
            action_ptr_t(),
            get_upper_value_bound(),
            get_lower_value_bound()
            );
        return;
    }

    // find new root node
    observation_ptr_t observation = new_root_instance->observation;
    reward_ptr_t reward = new_root_instance->reward;
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
        DEBUG_OUT(1, "Could not identify new root node");
        IF_DEBUG(2) {
            DEBUG_OUT(0,"    Need observation " << observation << ", reward " << reward);
            for(arc_to_observation = graph_t::OutArcIt(graph,action_node); arc_to_observation!=lemon::INVALID; ++arc_to_observation) {
                node_t observation_node = graph.target(arc_to_observation);
                DEBUG_OUT(0,"        Found observation " << node_info_map[observation_node].instance->observation <<
                          ", reward " << node_info_map[observation_node].instance->reward );
            }
            DEBUG_OUT(0,"    Old root instance " );
            for(const_instance_ptr_t old_instance = node_info_map[root_node].instance; old_instance!=util::INVALID; --old_instance) {
                DEBUG_OUT(0,"        " << old_instance );
            }
            DEBUG_OUT(0,"    New root instance " );
            for(const_instance_ptr_t new_instance = new_root_instance; new_instance!=util::INVALID; --new_instance) {
                DEBUG_OUT(0,"        " << new_instance );
            }
            // print_tree(false,true,"pruning_tree_error.eps");
            // clear_tree();
            // root_node = graph.addNode();
            // ++number_of_nodes;
            // node_info_map[root_node] = NodeInfo(
            //     OBSERVATION,
            //     NOT_EXPANDED,
            //     instance_t::create(new_root_instance->action, new_root_instance->observation, new_root_instance->reward, new_root_instance->const_it()-1),
            //     action_ptr_t::NULL_ACTION,
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
        DEBUG_ERROR("Search tree was not split by removing chosen action node");
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
        graph.erase(node);
    }

    // update root node
    if(DEBUG_LEVEL>=1) {
        // sanity check
        instance_ptr_t ins = node_info_map[new_root_node].instance;
        if(new_root_instance->action!=ins->action ||
           new_root_instance->observation!=ins->observation ||
           new_root_instance->reward!=ins->reward) {
            DEBUG_ERROR("Old and new instance of new root node do not match");
            DEBUG_OUT(0,"    old: " << *ins << ", new: " << *new_root_instance);
        }
    }
    root_node = new_root_node;
    node_info_map[root_node].instance = DoublyLinkedInstance::create(new_root_instance->action,
            new_root_instance->observation,
            new_root_instance->reward,
            new_root_instance->const_prev(),
            util::INVALID);

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
        expand_leaf_node(root_node, environment);
        break;
    default:
        DEBUG_DEAD_LINE;
    }

    // check graph structure
    if(!lemon::tree(ugraph)) {
        DEBUG_ERROR("Search graph is not a tree");
    }

    DEBUG_OUT(2,"DONE");
}

void LookAheadSearch::print_tree(const bool& text,
                                 const bool& eps_export,
                                 const char* file_name,
                                 const node_color_map_t * color_map
    ) const {

    if(root_node==INVALID) return;

    if(text) {
        DEBUG_OUT(0,"Print all nodes:");
        node_vector_t * current_nodes = new node_vector_t();
        node_vector_t * next_nodes = new node_vector_t();
        current_nodes->push_back(root_node);
        while(current_nodes->size()>0) {
            for(idx_t idx=0; idx<(idx_t)current_nodes->size(); ++idx) {
                node_t node = (*current_nodes)[idx];
                print_node(node);
                for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
                    next_nodes->push_back(graph.target(out_arc));
                }
            }
            DEBUG_OUT(0,"--------------------");
            node_vector_t * tmp = next_nodes;
            current_nodes->clear();
            next_nodes = current_nodes;
            current_nodes = tmp;
        }
        delete current_nodes;
        delete next_nodes;
        DEBUG_OUT(0,"\n");
    }

    if(eps_export) {

        DEBUG_OUT(0,"Printing tree to file '" << file_name << "'");

        // for graphical output
        double arc_width = 0.05;
        double node_size = 0.4;
        graph_t::NodeMap<Point>          coords(graph,Point(0,0));
        graph_t::NodeMap<double>         sizes(graph);
        graph_t::NodeMap<std::string>    lables(graph);
        graph_t::NodeMap<int>            shapes(graph);
        node_color_map_t                 node_colors(graph);
        graph_t::NodeMap<double>         node_color_weights(graph);
        graph_t::ArcMap<Color>           arc_colors(graph);
        graph_t::ArcMap<double>          widths(graph);

        // go through tree from root and assign coordinates and color weights
        typedef vector<tuple<node_t,double> > level_vector_t;
        enum { NODE, WEIGHT };
        graph_t::NodeMap<node_t> left_neighbor(graph);
        graph_t::NodeMap<node_t> right_neighbor(graph);
        level_vector_t * current_level = new level_vector_t();
        level_vector_t * next_level = new level_vector_t();
        NODE_TYPE current_level_type = NONE;
        large_size_t level_counter = 0;
        current_level->push_back(make_tuple(root_node,1));
        current_level_type = node_info_map[root_node].type;
        node_color_weights[root_node] = 1;
        while(current_level->size()>0) {
            current_level_type=node_info_map[get<NODE>(current_level->front())].type;
            idx_t current_level_size = current_level->size();
            for(idx_t idx=0; idx<current_level_size; ++idx) {
                node_t current_node = get<NODE>((*current_level)[idx]);
                double current_weight = get<WEIGHT>((*current_level)[idx]);
                left_neighbor[current_node] = idx==0 ? INVALID : get<NODE>((*current_level)[idx-1]);
                right_neighbor[current_node] = idx==current_level_size-1 ? INVALID : get<NODE>((*current_level)[idx+1]);
                coords[current_node] = Point(3*((double)idx-(double)current_level_size/2), level_counter);
                if(current_level_type!=node_info_map[current_node].type) {
                    DEBUG_ERROR("Level of mixed type");
                }
                node_vector_t optimal_action_node_vector;
                if(current_level_type==OBSERVATION) {
                    optimal_action_node_vector = optimal_action_nodes(current_node);
                }
                for(graph_t::OutArcIt out_arc(graph,current_node); out_arc!=INVALID; ++out_arc) {
                    node_t next_level_node = graph.target(out_arc);
                    double weight_factor = 0;
                    if(current_level_type==OBSERVATION) {
                        for( auto optimal_node : optimal_action_node_vector ) {
                            if(next_level_node==optimal_node) {
                                weight_factor = 1;
                                break;
                            }
                        }
                    } else {
                        weight_factor = arc_info_map[out_arc].prob;
                    }
                    double next_level_node_weight = current_weight*weight_factor;
                    if(next_level_node_weight>1 || next_level_node_weight<0) {
                        DEBUG_ERROR("Invalid node weight (" << next_level_node_weight << ")");
                    }
                    node_color_weights[next_level_node] = next_level_node_weight;
                    next_level->push_back(make_tuple(next_level_node, next_level_node_weight));
                }
            }
            current_level->clear();
            level_vector_t * tmp = current_level;
            current_level = next_level;
            next_level = tmp;
            ++level_counter;
        }
        delete current_level;
        delete next_level;

        // go through graph and assign sizes, shapes, lables, colors
        for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
            sizes[node] = node_size;
            lables[node] = QString::number(graph.id(node)).toStdString();
            lables[node] += ": ";
            if(node_info_map[node].type==OBSERVATION) {
                lables[node] += node_info_map[node].instance->observation->print();
                if(node_info_map[node].instance->reward->get_value()==reward_space->max_reward()) {
                    lables[node] += "*";
                }
                shapes[node] = 0;
            } else if(node_info_map[node].type==ACTION) {
                lables[node] += node_info_map[node].action->print();
                shapes[node] = 2;
            } else {
                lables[node] = "";
                shapes[node] = 1;
            }
            // assign color
            node_colors[node] = Color(1., 1-node_color_weights[node], 1-node_color_weights[node]);
            // flip so that root is at top
            coords[node] = Point(coords[node].x, level_counter - coords[node].y);
        }
        for(graph_t::ArcIt arc(graph); arc!=INVALID; ++arc) {
            node_t source_node = graph.source(arc);
            node_t target_node = graph.target(arc);
            if(node_info_map[source_node].type==ACTION) {
                widths[arc] = arc_width*arc_info_map[arc].prob;
            } else {
                widths[arc] = arc_width*0.5;
            }
            arc_colors[arc] = Color(node_color_weights[target_node], 0, 0);
        }

        // optimize node horizontal position
        double min_x, max_x, max_dx=1, max_dx_average=100, max_max_dx=-DBL_MAX, max_dx_threshold=0.0001, node_margin=3*node_size;
        ProgressBar::init("Optimizing positions: ");
        while(fabs(max_dx-max_dx_average)>max_dx_threshold) {
            min_x =  DBL_MAX;
            max_x = -DBL_MAX;
            max_dx = -DBL_MAX;
            for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {

                // nodes x position
                double node_x = coords[node].x;

                // min and max position determined by the neighbors
                bool  left_ok =  left_neighbor[node]!=INVALID;
                bool right_ok = right_neighbor[node]!=INVALID;
                double min_x_allowed, max_x_allowed;
                if( !left_ok && !right_ok) {
                    min_x_allowed = -DBL_MAX;
                    max_x_allowed =  DBL_MAX;
                } else if( !left_ok) {
                    min_x_allowed = -DBL_MAX;
                    max_x_allowed = coords[right_neighbor[node]].x - node_margin;
                } else if(!right_ok) {
                    min_x_allowed = coords[ left_neighbor[node]].x + node_margin;
                    max_x_allowed = DBL_MAX;
                } else {
                    min_x_allowed = coords[ left_neighbor[node]].x + node_margin;
                    max_x_allowed = coords[right_neighbor[node]].x - node_margin;
                }

                // optimal node position with respect to parents
                double parents_x_sum = 0;
                int parents_counter = 0;
                for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
                    parents_x_sum += coords[graph.target(out_arc)].x;
                    ++parents_counter;
                }

                // optimal node position with respect to children
                double children_x_sum = 0;
                int children_counter = 0;
                for(graph_t::InArcIt in_arc(graph,node); in_arc!=INVALID; ++in_arc) {
                    children_x_sum += coords[graph.source(in_arc)].x;
                    ++children_counter;
                }

                // calculate new position
                double x_target = 0;
                int summands = 0;
                if(parents_counter>0) {
                    x_target += parents_x_sum/parents_counter;
                    ++summands;
                }
                if(children_counter>0) {
                    x_target += children_x_sum/children_counter;
                    ++summands;
                }
                if(left_ok) {
                    x_target += min_x_allowed;
                    ++summands;
                }
                if(right_ok) {
                    x_target += max_x_allowed;
                    ++summands;
                }
                if(summands==0) { DEBUG_DEAD_LINE; }
                x_target /= summands;
                if(x_target<min_x_allowed && x_target>max_x_allowed) {
                    x_target = (min_x_allowed + max_x_allowed)/2;
                } else if(x_target<min_x_allowed) {
                    x_target = min_x_allowed;
                } else if(x_target>max_x_allowed) {
                    x_target = max_x_allowed;
                }
                x_target = 0.2*x_target + 0.8*node_x; // only go a small step towards target
                double dx = fabs(x_target - node_x);
                coords[node] = Point(x_target, coords[node].y);

                // updates
                if(dx>max_dx) {
                    max_dx = dx;
                    if(max_dx>max_max_dx) {
                        max_max_dx = max_dx;
                    }
                }
                if(x_target<min_x) {
                    min_x = x_target;
                }
                if(x_target>max_x) {
                    max_x = x_target;
                }
            }
            max_dx_average = 0.99*max_dx_average + 0.01*max_dx;
            double progress_value = 1-max_dx/max_max_dx; // goes from 0 to 1
            progress_value = 0.1*progress_value + 0.9*pow(progress_value,4);
            ProgressBar::print(progress_value);
        }
        ProgressBar::terminate();

        // make coords positive and rescale to square
        double y_scale_factor = 1;
        if(max_x-min_x>level_counter) {
            y_scale_factor = (max_x-min_x)/level_counter;
        }
        for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
            coords[node] = Point(coords[node].x - min_x + 2, y_scale_factor*coords[node].y);
            // DEBUG_OUT(0, "Node " << graph.id(node) << " (" << coords[node].x << "," << coords[node].y << ")" );
        }

        // actually print the graph
        const node_color_map_t * use_color_map = color_map;
        if(color_map==nullptr) {
            use_color_map = &node_colors;
        }
        lemon::graphToEps(graph, file_name)
            .coords(coords)
            .title("Look Ahead Tree")
            .absoluteNodeSizes().nodeScale(1).nodeSizes(sizes)
            .nodeShapes(shapes).nodeColors(*use_color_map)
            .nodeTexts(lables).nodeTextSize(0.1)
            .absoluteArcWidths(true).arcWidthScale(1).arcWidths(widths).arcColors(arc_colors)
            .drawArrows(true).arrowWidth(4*arc_width).arrowLength(4*arc_width)
            .run();
    }
}

void LookAheadSearch::print_tree_statistics() const {

    // check for invalid root node
    if(root_node==INVALID) {
        DEBUG_OUT(0,"Root node is INVALID");
        return;
    }

    // count nodes and arcs
    node_vector_t * current_level = new node_vector_t();
    node_vector_t * next_level = new node_vector_t();
    NODE_TYPE current_level_type = NONE;
    large_size_t total_arc_counter = 0, total_node_counter = 0, level_counter = 0;
    current_level->push_back(root_node);
    current_level_type = node_info_map[root_node].type;
    DEBUG_OUT(0,"Printing tree statistics");
    while(current_level->size()>0) {
        current_level_type=node_info_map[(*current_level)[0]].type;
        DEBUG_OUT(0,"    Level " << level_counter << ": " <<
                  current_level->size() << " nodes (" <<
                  (current_level_type==OBSERVATION ? "OBSERVATION" : (current_level_type==ACTION ? "ACTION" : "NONE")) << ")");
        for(idx_t idx=0; idx<(idx_t)current_level->size(); ++idx) {
            ++total_node_counter;
            if(current_level_type!=node_info_map[(*current_level)[idx]].type) {
                DEBUG_ERROR("Level of mixed type");
            }
            for(graph_t::OutArcIt out_arc(graph,(*current_level)[idx]); out_arc!=INVALID; ++out_arc) {
                ++total_arc_counter;
                next_level->push_back(graph.target(out_arc));
            }
        }
        current_level->clear();
        node_vector_t * tmp = current_level;
        current_level = next_level;
        next_level = tmp;
        ++level_counter;
    }
    delete current_level;
    delete next_level;

    // error check
    if(total_node_counter!=number_of_nodes) {
        DEBUG_ERROR("Total node counter (" << total_node_counter << ") is different from number of nodes " << number_of_nodes);
    }

    // print result
    DEBUG_OUT(0,"    Tree has a total of " << total_node_counter << " nodes and " << total_arc_counter << " arcs");

    // prepare for printing actions
    value_t min_lower_bound = DBL_MAX, max_upper_bound = -DBL_MAX;
    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        value_t lower_bound = node_info_map[action_node].lower_value_bound;
        value_t upper_bound = node_info_map[action_node].upper_value_bound;
        if(lower_bound<min_lower_bound) {
            min_lower_bound=lower_bound;
        }
        if(upper_bound>max_upper_bound) {
            max_upper_bound=upper_bound;
        }
    }

    // print actions
    int width = 60;
    DEBUG_OUT(0,"    Values at root node:");
    DEBUG_OUT(0,QString("    %1|%2|%3")
              .arg(min_lower_bound,11,'e',5,'0')
              .arg(QString(' ').repeated(width-1))
              .arg(max_upper_bound,11,'e',5,'0')
              .toStdString()
        );
    action_ptr_t optimal_action = get_optimal_action();
    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=INVALID; ++out_arc) {
        node_t action_node = graph.target(out_arc);
        value_t lower_bound = node_info_map[action_node].lower_value_bound;
        value_t upper_bound = node_info_map[action_node].upper_value_bound;
        value_t weighted_bound = lower_bound_weight*lower_bound + (1-lower_bound_weight)*upper_bound;
        value_t normalized_lower_bound    = (lower_bound    - min_lower_bound)/(max_upper_bound - min_lower_bound);
        value_t normalized_upper_bound    = (upper_bound    - min_lower_bound)/(max_upper_bound - min_lower_bound);
        value_t normalized_weighted_bound = (weighted_bound - min_lower_bound)/(max_upper_bound - min_lower_bound);
        int lower_count    = round(width*normalized_lower_bound);
        int upper_count    = round(width*normalized_upper_bound);
        int weighted_count = round(width*normalized_weighted_bound);
        weighted_count = weighted_count==lower_count ? weighted_count+1 : weighted_count;
        upper_count = upper_count==weighted_count ? upper_count+1 : upper_count;
        DEBUG_OUT(0, QString("    %1%2|%3%4%5|%6%7 %8%9")
                  .arg(lower_bound,11,'e',5,'0')
                  .arg(QString(' ').repeated(lower_count))
                  .arg(QString('-').repeated(weighted_count-lower_count-1))
                  .arg((optimal_action_selection_type==MAX_WEIGHTED_BOUNDS) ? 'x' : '-')
                  .arg(QString('-').repeated(upper_count-weighted_count-1))
                  .arg(QString(' ').repeated(width-upper_count))
                  .arg(upper_bound,11,'e',5,'0')
                  .arg(node_info_map[action_node].action->print().c_str())
                  .arg(node_info_map[action_node].action==optimal_action ? '*' : ' ')
                  .toStdString()
            );
    }
}

LookAheadSearch::node_t LookAheadSearch::select_next_action_node(node_t observation_node) {

    DEBUG_OUT(3,"Selecting next action");

    // selection action(s)
    node_vector_t next_action_nodes;
    switch(tree_action_selection_type) {
    case MAX_UPPER_BOUND:
    {
        DEBUG_OUT(4,"    using maximum upper bound as criterion");
        value_t max_upper_bound = -DBL_MAX, current_upper_bound;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            current_action_node = graph.target(out_arc);
            current_upper_bound = node_info_map[current_action_node].upper_value_bound;
            if(approx_eq(current_upper_bound,max_upper_bound)) {
                next_action_nodes.push_back(current_action_node);
            } else if(current_upper_bound>max_upper_bound) {
                next_action_nodes.clear();
                next_action_nodes.push_back(current_action_node);
                max_upper_bound = current_upper_bound;
            }
        }
        break;
    }
    case MAX_LOWER_BOUND:
    {
        DEBUG_OUT(4,"    using maximum lower bound as criterion");
        value_t max_lower_bound = -DBL_MAX, current_lower_bound;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            current_action_node = graph.target(out_arc);
            current_lower_bound = node_info_map[current_action_node].lower_value_bound;
            if(approx_eq(current_lower_bound,max_lower_bound)) {
                next_action_nodes.push_back(current_action_node);
            } else if(current_lower_bound>max_lower_bound) {
                next_action_nodes.clear();
                next_action_nodes.push_back(current_action_node);
                max_lower_bound = current_lower_bound;
            }
        }
        break;
    }
    default:
        DEBUG_ERROR("Action selection type not implemented.");
        break;
    }

    // select action
    node_t selected_action_node;
    if(random_tie_break) {
        selected_action_node = util::random_select(next_action_nodes);
    } else {
        selected_action_node = next_action_nodes.front();
    }

    // check
    if(node_info_map[selected_action_node].type!=ACTION) {
        DEBUG_ERROR("Next action node is not of type ACTION");
    }

    DEBUG_OUT(4,"    Next action:");
    if(DEBUG_LEVEL>=4) {
        print_node(selected_action_node);
    }
    return selected_action_node;
}

LookAheadSearch::node_t LookAheadSearch::select_next_observation_node(node_t action_node) {
    DEBUG_OUT(3,"Selecting next observation");
    node_vector_t next_observation_nodes;
    switch(tree_observation_selection_type) {
    case MAX_WEIGHTED_UNCERTAINTY:
    {
        DEBUG_OUT(4,"    using maximum uncertainty as criterion");
        value_t max_uncertainty = -DBL_MAX, current_uncertainty;
        node_t current_observation_node;
        for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=INVALID; ++out_arc) {
            current_observation_node = graph.target(out_arc);
            current_uncertainty = node_info_map[current_observation_node].upper_value_bound - node_info_map[current_observation_node].lower_value_bound;
            current_uncertainty *= arc_info_map[out_arc].prob;
            if(approx_eq(current_uncertainty,max_uncertainty)) {
                next_observation_nodes.push_back(current_observation_node);
            } else if(current_uncertainty>max_uncertainty) {
                next_observation_nodes.clear();
                next_observation_nodes.push_back(current_observation_node);
                max_uncertainty = current_uncertainty;
            }
        }
        break;
    }
    default:
        DEBUG_ERROR("Observation selection type not implemented.");
        break;
    }

    // select observation
    node_t selected_observation_node;
    if(random_tie_break) {
        selected_observation_node = util::random_select(next_observation_nodes);
    } else {
        selected_observation_node = next_observation_nodes.front();
    }

    // check
    if(node_info_map[selected_observation_node].type!=OBSERVATION) {
        DEBUG_ERROR("Next observation node is not of type OBSERVATION");
    }
    DEBUG_OUT(4,"    Next observation:");
    if(DEBUG_LEVEL>=4) {
        print_node(selected_observation_node);
    }
    return selected_observation_node;
}

void LookAheadSearch::expand_leaf_node(
        node_t observation_node,
        const Predictor& environment
) {

    DEBUG_OUT(3,"Expanding leaf node");
    if(DEBUG_LEVEL>=3) {
        print_node(observation_node);
    }

    if(node_info_map[observation_node].type!=OBSERVATION) {
        DEBUG_ERROR("trying to expand non-observation node as observation node");
    }
    if(node_info_map[observation_node].expansion!=NOT_EXPANDED) {
        DEBUG_ERROR("trying to expand observation node with expansion other than NOT_EXPANDED");
    }

    instance_ptr_t instance_from = node_info_map[observation_node].instance;

    // create action nodes
    for(action_ptr_t action : action_space) {
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
        expand_action_node(action_node, environment);
        update_action_node(action_node);
    }

    // set to fully expanded
    node_info_map[observation_node].expansion = FULLY_EXPANDED;
}

void LookAheadSearch::expand_action_node(
        node_t action_node,
        const Predictor& environment
) {

    DEBUG_OUT(3,"Expanding action node");
    if(DEBUG_LEVEL>=4) {
        print_node(action_node);
    }

    if(node_info_map[action_node].type!=ACTION) {
        DEBUG_ERROR("trying to expand non-action node as action node");
    }
    if(node_info_map[action_node].expansion!=NOT_EXPANDED) {
        DEBUG_ERROR("trying to expand action node with expansion other than NOT_EXPANDED");
    }

    const_instance_ptr_t instance_from = node_info_map[action_node].instance;
    action_ptr_t action = node_info_map[action_node].action;

    // add all target observations (MDP-observation-reward combinations)
    probability_map_t prob_map = environment.get_prediction_map(instance_from, action);
    for(observation_ptr_t new_observation : observation_space) {

        node_t new_observation_node = lemon::INVALID;

        for(reward_ptr_t new_reward : reward_space) {

            //probability_t prob = environment.get_prediction(instance_from, action, new_observation, new_reward);
            probability_t prob = prob_map[make_tuple(new_observation,new_reward)];
            if(prob>0) {
                new_observation_node = graph.addNode();
                ++number_of_nodes;
                node_info_map[new_observation_node] = NodeInfo(
                        OBSERVATION,
                        NOT_EXPANDED,
                        DoublyLinkedInstance::create(action, new_observation, new_reward, instance_from, util::INVALID),
                        action_ptr_t(), // not defined for observation nodes
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
        DEBUG_ERROR("No possible observation transition.")
    }

    node_info_map[action_node].expansion = FULLY_EXPANDED;
}

LookAheadSearch::node_vector_t LookAheadSearch::optimal_action_nodes(const node_t& observation_node) const {
    // check
    if(node_info_map[observation_node].type!=OBSERVATION) {
        DEBUG_ERROR("Given observation node is not of type OBSERVATION");
    }

    // determine optimal actions
    node_vector_t optimal_action_node_vector;
    switch(optimal_action_selection_type) {
    case MAX_LOWER_BOUND:
    {
        DEBUG_OUT(3,"    Using maximum lower bound as criterion");
        value_t max_lower_bound = -DBL_MAX, current_lower_bound;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            current_action_node = graph.target(out_arc);
            current_lower_bound = node_info_map[current_action_node].lower_value_bound;
            if(approx_eq(current_lower_bound,max_lower_bound)) {
                optimal_action_node_vector.push_back(current_action_node);
            } else if(current_lower_bound>max_lower_bound) {
                optimal_action_node_vector.clear();
                optimal_action_node_vector.push_back(current_action_node);
                max_lower_bound=current_lower_bound;
            }
        }
        break;
    }
    case MAX_WEIGHTED_BOUNDS:
    {
        DEBUG_OUT(3,"    Using maximum weighted bounds as criterion");
        value_t max_value = -DBL_MAX, current_value;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            current_action_node = graph.target(out_arc);
            current_value = 0;
            current_value +=    lower_bound_weight  * node_info_map[current_action_node].lower_value_bound;
            current_value += (1-lower_bound_weight) * node_info_map[current_action_node].upper_value_bound;
            if(approx_eq(current_value,max_value)) {
                optimal_action_node_vector.push_back(current_action_node);
            } else if(current_value>max_value) {
                optimal_action_node_vector.clear();
                optimal_action_node_vector.push_back(current_action_node);
                max_value=current_value;
            }
        }
        break;
    }
    default:
        DEBUG_ERROR("Optimal action selection type not implemented");
        break;
    }

    return optimal_action_node_vector;
}

LookAheadSearch::node_t LookAheadSearch::update_action_node(node_t action_node) {
    switch(action_back_propagation_type) {
    case EXPECTED_BOUNDS:
    {
        probability_t prob_sum = 0;
        node_info_map[action_node].upper_value_bound = 0;
        node_info_map[action_node].lower_value_bound = 0;
        for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=INVALID; ++out_arc) {
            node_t observation_node = graph.target(out_arc);
            probability_t observation_prob = arc_info_map[out_arc].prob;
            prob_sum += observation_prob;
            reward_ptr_t transition_reward = arc_info_map[out_arc].transition_reward;
            node_info_map[action_node].upper_value_bound += observation_prob * (transition_reward->get_value() + discount*node_info_map[observation_node].upper_value_bound);
            node_info_map[action_node].lower_value_bound += observation_prob * (transition_reward->get_value() + discount*node_info_map[observation_node].lower_value_bound);
        }
        if(fabs(prob_sum-1)>1e-10) {
            DEBUG_ERROR("Unnormalized observation transition probabilities (p_sum=" << prob_sum << ")");
        }
        break;
    }
    case MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER:
    {
        value_t max_upper = -DBL_MAX;
        value_t min_lower =  DBL_MAX;
        for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=INVALID; ++out_arc) {
            node_t observation_node = graph.target(out_arc);
            reward_ptr_t transition_reward = arc_info_map[out_arc].transition_reward;
            value_t upper = transition_reward->get_value() + discount*node_info_map[observation_node].upper_value_bound;
            value_t lower = transition_reward->get_value() + discount*node_info_map[observation_node].lower_value_bound;
            if(upper>max_upper) {
                max_upper = upper;
            }
            if(lower<min_lower) {
                min_lower = lower;
            }
        }
        if(max_upper==-DBL_MAX||min_lower==DBL_MAX) {
            DEBUG_ERROR("No outgoing observation node");
        }
        node_info_map[action_node].upper_value_bound = max_upper;
        node_info_map[action_node].lower_value_bound = min_lower;
        break;
    }
    default:
        DEBUG_ERROR("Action back-propagation type not implemented.");
        break;
    }

    DEBUG_OUT(3,"Updated action node");
    if(DEBUG_LEVEL>=3) {
        print_node(action_node);
    }

    // return parent observation node
    graph_t::InArcIt in_arc(graph,action_node);
    node_t parent_observation_node = graph.source(in_arc);
    if(in_arc==INVALID) {
        DEBUG_ERROR("No parent observation node for this action");
    }
    if(++in_arc!=INVALID) {
        DEBUG_ERROR("More than one parent observation node for this action");
    }
    if(node_info_map[parent_observation_node].type!=OBSERVATION) {
        DEBUG_ERROR("Parent observation node is not of type OBSERVATION");
    }
    DEBUG_OUT(4,"    Parent observation node:");
    if(DEBUG_LEVEL>=4) {
        print_node(parent_observation_node);
    }
    return parent_observation_node;
}

LookAheadSearch::node_t LookAheadSearch::update_observation_node(node_t observation_node) {
    BOUND_USAGE_TYPE use_type = observation_back_propagation_type;
    if(use_type==SAME_AS_OPTIMAL_ACTION_SELECTION) {
        use_type = optimal_action_selection_type;
    }
    switch(use_type) {
    case MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER:
    {
        value_t max_upper_bound= -DBL_MAX, current_upper_bound, current_lower_bound;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            node_t action_node = graph.target(out_arc);
            current_upper_bound = node_info_map[action_node].upper_value_bound;
            current_lower_bound = node_info_map[action_node].lower_value_bound;
            if(
                (!approx_eq(current_upper_bound,max_upper_bound) && // if upper bounds are NOT equal
                 current_upper_bound>max_upper_bound ) || // primary criterion: upper bound
                ( approx_eq(current_upper_bound,max_upper_bound) && // if upper bounds are equal
                  current_lower_bound>node_info_map[observation_node].lower_value_bound ) // secondary criterion: lower bounds
                ) {
                max_upper_bound=current_upper_bound;
                node_info_map[observation_node].upper_value_bound = current_upper_bound;
                node_info_map[observation_node].lower_value_bound = current_lower_bound;
            }
        }
        break;
    }
    case MAX_WEIGHTED_BOUNDS:
    {
        value_t max_value= -DBL_MAX;
        for(graph_t::OutArcIt out_arc(graph,observation_node); out_arc!=INVALID; ++out_arc) {
            node_t action_node = graph.target(out_arc);
            value_t current_upper_bound = node_info_map[action_node].upper_value_bound;
            value_t current_lower_bound = node_info_map[action_node].lower_value_bound;
            value_t current_value = lower_bound_weight*current_lower_bound + (1-lower_bound_weight)*current_upper_bound;
            if(current_value>max_value) { // no randomization needed for equal values since actual action is not considered
                max_value=current_value;
                node_info_map[observation_node].upper_value_bound = current_upper_bound;
                node_info_map[observation_node].lower_value_bound = current_lower_bound;
            }
        }
        break;
    }
    default:
        DEBUG_ERROR("Observation back-propagation type not implemented.");
        break;
    }

    DEBUG_OUT(3,"Updated observation node");
    if(DEBUG_LEVEL>=4) {
        print_node(observation_node);
    }

    // return parent action node
    if(observation_node!=root_node) {
        graph_t::InArcIt in_arc(graph,observation_node);
        node_t parent_action_node = graph.source(in_arc);
        if(in_arc==INVALID) {
            DEBUG_ERROR("No parent action node for this observation");
        }
        if(++in_arc!=INVALID) {
            DEBUG_ERROR("More than one parent action node for this observation");
        }
        if(node_info_map[parent_action_node].type!=ACTION) {
            DEBUG_ERROR("Parent action node is not of type ACTION");
        }
        DEBUG_OUT(4,"    Parent action node:");
        if(DEBUG_LEVEL>=4) {
            print_node(parent_action_node);
        }
        return parent_action_node;
    } else {
        return INVALID;
    }
}

bool LookAheadSearch::tree_needs_further_expansion() {
    DEBUG_OUT(3,"Determining whether tree needs further expansion");
    value_t max_upper_bound        = -DBL_MAX;
    value_t max_upper_lower_bound  = -DBL_MAX;
    value_t second_max_upper_bound = -DBL_MAX;
    value_t current_lower_bound, current_upper_bound;
    for(graph_t::OutArcIt out_arc(graph,root_node); out_arc!=INVALID; ++out_arc) {
        node_t current_action_node = graph.target(out_arc);
        current_lower_bound = node_info_map[current_action_node].lower_value_bound;
        current_upper_bound = node_info_map[current_action_node].upper_value_bound;
        if(current_upper_bound>max_upper_bound) {
            second_max_upper_bound=max_upper_bound;
            max_upper_bound=current_upper_bound;
            max_upper_lower_bound=current_lower_bound;
        } else if(current_upper_bound>second_max_upper_bound){
            second_max_upper_bound=current_upper_bound;
        }
        DEBUG_OUT(4,"    " << node_info_map[current_action_node].action <<
                  ": [" << current_lower_bound << " <--> " << current_upper_bound << "]");
    }
    DEBUG_OUT(4,"           max upper: " << max_upper_bound );
    DEBUG_OUT(4,"     max upper lower: " << max_upper_lower_bound );
    DEBUG_OUT(4,"    second max upper: " << second_max_upper_bound );
    if(max_upper_bound>second_max_upper_bound) {
        DEBUG_OUT(4,"    " << max_upper_bound << ">" << second_max_upper_bound );
    } else if(max_upper_bound==second_max_upper_bound) {
        DEBUG_OUT(4,"    " << max_upper_bound << "==" << second_max_upper_bound );
    } else {
        DEBUG_OUT(0,"    " << max_upper_bound << ">" << second_max_upper_bound << " !!!What has happened here???" );
    }

    if(max_upper_bound==-DBL_MAX) {
        DEBUG_OUT(1,"Error: No actions available from root node");
        return true;
    }
    if(second_max_upper_bound==-DBL_MAX) {
        DEBUG_OUT(4,"    Only one action available from root node");
    }
    if(max_upper_lower_bound>second_max_upper_bound) {
        DEBUG_OUT(3,"    No");
        return false;
    } else {
        DEBUG_OUT(3,"    Yes");
        return true;
    }
}

void LookAheadSearch::print_node(node_t node) const {
    if(node==INVALID) {
        DEBUG_OUT(0,"Node is INVALID");
        return;
    }

    DEBUG_OUT(0, "Node " << graph.id(node) << ":");
    switch(node_info_map[node].type) {
    case NONE:
        DEBUG_OUT(0, "    type:    " << "NONE" );
        break;
    case OBSERVATION:
        DEBUG_OUT(0, "    type:    " << "OBSERVATION" );
        DEBUG_OUT(0, "    instance:" << *(node_info_map[node].instance) );
        break;
    case ACTION:
        DEBUG_OUT(0, "    type:    " << "ACTION" );
        DEBUG_OUT(0, "    instance:" << *(node_info_map[node].instance) );
        DEBUG_OUT(0, "    action:  " << node_info_map[node].action );
        break;
    }
    switch(node_info_map[node].expansion) {
    case NOT_DEFINED:
        DEBUG_OUT(0, "    expansion: NOT_DEFINED");
        break;
    case NOT_EXPANDED:
        DEBUG_OUT(0, "    expansion: NOT_EXPANDED");
        break;
    case FULLY_EXPANDED:
        DEBUG_OUT(0, "    expansion: FULLY_EXPANDED");
        break;
    default:
        DEBUG_ERROR("Unknown expansion");
        break;
    }
    DEBUG_OUT(0, "    upper_bound: " << node_info_map[node].upper_value_bound );
    DEBUG_OUT(0, "    lower_bound: " << node_info_map[node].lower_value_bound );
    DEBUG_OUT(0, "    in arcs from:");
    for(graph_t::InArcIt in_arc(graph,node); in_arc!=INVALID; ++in_arc) {
        std::cout << graph.id(graph.source(in_arc)) << " ";
    }
    std::cout << std::endl;
    DEBUG_OUT(0, "    out arcs to:");
    for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
        std::cout << graph.id(graph.target(out_arc)) << " ";
    }
    std::cout << std::endl;
}

double LookAheadSearch::node_energy(node_t node, const graph_t::NodeMap<Point>& coords) const {
    double energy = 0;
    double node_x_coord = coords[node].x;
    double node_y_coord = coords[node].y;

    double repulsion_coefficient = 0.08;
    double attraction_coefficient = 1;

    // repulsive part for other nodes
    for(graph_t::NodeIt other_node(graph); other_node!=INVALID; ++other_node) {
        if(other_node==node) continue;
        double distance = sqrt( pow(coords[other_node].x-node_x_coord,2) + pow(coords[other_node].y-node_y_coord,2) );
        if(distance<1e-6) distance=1e-6; // hack to prevent overflow
        energy += repulsion_coefficient/distance;
    }

    // attractive part for adjacent arcs
    for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
        node_t other_node = graph.target(out_arc);
        double distance = sqrt( pow(coords[other_node].x-node_x_coord,2) + pow(coords[other_node].y-node_y_coord,2) );
        if(distance<1e-6) distance=1e-6; // hack to prevent overflow
        energy += attraction_coefficient*pow(distance,2);
    }
    for(graph_t::InArcIt in_arc(graph,node); in_arc!=INVALID; ++in_arc) {
        node_t other_node = graph.source(in_arc);
        double distance = sqrt( pow(coords[other_node].x-node_x_coord,2) + pow(coords[other_node].y-node_y_coord,2) );
        if(distance<1e-6) distance=1e-6; // hack to prevent overflow
        energy += attraction_coefficient*pow(distance,2);
    }

    return energy;
}
