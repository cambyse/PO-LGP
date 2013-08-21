#include "LookAheadSearch.h"

#include <unistd.h> // for sleep()
#include <float.h>
#include <math.h>
#include <vector>
#include <tuple>

#include "util.h"
#include "Maze.h"

#define DEBUG_LEVEL 1
#include "debug.h"

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
    action(action_t()),
    upper_value_bound(0),
    lower_value_bound(0)
{}

LookAheadSearch::NodeInfo::NodeInfo(
    const NODE_TYPE& t,
    const EXPANSION_TYPE& e,
    instance_t * i,
    const action_t& a,
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
        DEBUG_OUT(0,"Error: Lower value bound above upper value bound");
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
    DEBUG_OUT(2,"Clearing graph");
    for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
        // DEBUG_OUT(0,"Deleting instance of:");
        // print_node(node);
        if(node_info_map[node].type==STATE) {
            delete node_info_map[node].instance;
        }
    }
    graph.clear();
    number_of_nodes = 0;
    root_node = INVALID;
}

LookAheadSearch::action_t LookAheadSearch::get_optimal_action() const {
    DEBUG_OUT(2,"Determining optimal action");

    node_vector_t optimal_action_node_vector = optimal_action_nodes(root_node);

    // select one action randomly
    action_t selected_action = action_t::STAY;
    if(optimal_action_node_vector.size()==0) {
        DEBUG_OUT(0,"Error: No action available from root node. Choosing " << selected_action << ".");
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
            DEBUG_OUT(0,"Error: Next action node is not of type ACTION");
        }

        // get action
        selected_action = node_info_map[selected_action_node].action;
        DEBUG_OUT(3, "    choosing " << selected_action << " from " << optimal_action_node_vector.size() << " actions");

        return selected_action;
    }
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
        size_t level_counter = 0;
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
                    DEBUG_OUT(0,"Error: Level of mixed type");
                }
                node_vector_t optimal_action_node_vector;
                if(current_level_type==STATE) {
                    optimal_action_node_vector = optimal_action_nodes(current_node);
                }
                for(graph_t::OutArcIt out_arc(graph,current_node); out_arc!=INVALID; ++out_arc) {
                    node_t next_level_node = graph.target(out_arc);
                    double weight_factor = 0;
                    if(current_level_type==STATE) {
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
                        DEBUG_OUT(0,"Error: Invalid node weight (" << next_level_node_weight << ")");
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
            if(node_info_map[node].type==STATE) {
                lables[node] += Maze::MazeState(node_info_map[node].instance->state).print();
                if(node_info_map[node].instance->reward==reward_t::max_reward) {
                    lables[node] += "*";
                }
                shapes[node] = 0;
            } else if(node_info_map[node].type==ACTION) {
                lables[node] += node_info_map[node].action.action_string();
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

void LookAheadSearch::print_tree_statistics() {

    // check for invalid root node
    if(root_node==INVALID) {
        DEBUG_OUT(0,"Root node is INVALID");
        return;
    }

    // count nodes and arcs
    node_vector_t * current_level = new node_vector_t();
    node_vector_t * next_level = new node_vector_t();
    NODE_TYPE current_level_type = NONE;
    size_t total_arc_counter = 0, total_node_counter = 0, level_counter = 0;
    current_level->push_back(root_node);
    current_level_type = node_info_map[root_node].type;
    DEBUG_OUT(0,"Printing tree statistics");
    while(current_level->size()>0) {
        current_level_type=node_info_map[(*current_level)[0]].type;
        DEBUG_OUT(0,"    Level " << level_counter << ": " <<
                  current_level->size() << " nodes (" <<
                  (current_level_type==STATE ? "STATE" : (current_level_type==ACTION ? "ACTION" : "NONE")) << ")");
        for(idx_t idx=0; idx<(idx_t)current_level->size(); ++idx) {
            ++total_node_counter;
            if(current_level_type!=node_info_map[(*current_level)[idx]].type) {
                DEBUG_OUT(0,"Error: Level of mixed type");
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
        DEBUG_OUT(0,"Error: Total node counter (" << total_node_counter << ") is different from number of nodes " << number_of_nodes << " (correcting)");
        number_of_nodes = total_node_counter;
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
    action_t optimal_action = get_optimal_action();
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
                  .arg(node_info_map[action_node].action.action_string())
                  .arg(node_info_map[action_node].action==optimal_action ? '*' : ' ')
                  .toStdString()
            );
    }
}

LookAheadSearch::node_t LookAheadSearch::select_next_action_node(node_t state_node) {

    DEBUG_OUT(3,"Selecting next action");

    // selection action(s)
    node_vector_t next_action_nodes;
    switch(tree_action_selection_type) {
    case MAX_UPPER_BOUND:
    {
        DEBUG_OUT(4,"    using maximum upper bound as criterion");
        value_t max_upper_bound = -DBL_MAX, current_upper_bound;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
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
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
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
        DEBUG_OUT(0,"Error: Action selection type not implemented.");
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
        DEBUG_OUT(0,"Error: Next action node is not of type ACTION");
    }

    DEBUG_OUT(4,"    Next action:");
    if(DEBUG_LEVEL>=4) {
        print_node(selected_action_node);
    }
    return selected_action_node;
}

LookAheadSearch::node_t LookAheadSearch::select_next_state_node(node_t action_node) {
    DEBUG_OUT(3,"Selecting next state");
    node_vector_t next_state_nodes;
    switch(tree_state_selection_type) {
    case MAX_WEIGHTED_UNCERTAINTY:
    {
        DEBUG_OUT(4,"    using maximum uncertainty as criterion");
        value_t max_uncertainty = -DBL_MAX, current_uncertainty;
        node_t current_state_node;
        for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=INVALID; ++out_arc) {
            current_state_node = graph.target(out_arc);
            current_uncertainty = node_info_map[current_state_node].upper_value_bound - node_info_map[current_state_node].lower_value_bound;
            current_uncertainty *= arc_info_map[out_arc].prob;
            if(approx_eq(current_uncertainty,max_uncertainty)) {
                next_state_nodes.push_back(current_state_node);
            } else if(current_uncertainty>max_uncertainty) {
                next_state_nodes.clear();
                next_state_nodes.push_back(current_state_node);
                max_uncertainty = current_uncertainty;
            }
        }
        break;
    }
    default:
        DEBUG_OUT(0,"Error: State selection type not implemented.");
        break;
    }

    // select state
    node_t selected_state_node;
    if(random_tie_break) {
        selected_state_node = util::random_select(next_state_nodes);
    } else {
        selected_state_node = next_state_nodes.front();
    }

    // check
    if(node_info_map[selected_state_node].type!=STATE) {
        DEBUG_OUT(0,"Error: Next state node is not of type STATE");
    }
    DEBUG_OUT(4,"    Next state:");
    if(DEBUG_LEVEL>=4) {
        print_node(selected_state_node);
    }
    return selected_state_node;
}

LookAheadSearch::node_vector_t LookAheadSearch::optimal_action_nodes(const node_t& state_node) const {
    // check
    if(node_info_map[state_node].type!=STATE) {
        DEBUG_OUT(0,"Error: Given state node is not of type STATE");
    }

    // determine optimal actions
    node_vector_t optimal_action_node_vector;
    switch(optimal_action_selection_type) {
    case MAX_LOWER_BOUND:
    {
        DEBUG_OUT(3,"    Using maximum lower bound as criterion");
        value_t max_lower_bound = -DBL_MAX, current_lower_bound;
        node_t current_action_node;
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
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
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
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
        DEBUG_OUT(0,"Error: Optimal action selection type not implemented");
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
            node_t state_node = graph.target(out_arc);
            probability_t state_prob = arc_info_map[out_arc].prob;
            prob_sum += state_prob;
            reward_t transition_reward = arc_info_map[out_arc].transition_reward;
            node_info_map[action_node].upper_value_bound += state_prob * (transition_reward + discount*node_info_map[state_node].upper_value_bound);
            node_info_map[action_node].lower_value_bound += state_prob * (transition_reward + discount*node_info_map[state_node].lower_value_bound);
        }
        if(fabs(prob_sum-1)>1e-10) {
            DEBUG_OUT(0,"Error: Unnormalized state transition probabilities (p_sum=" << prob_sum << ")");
        }
        break;
    }
    case MAX_UPPER_FOR_UPPER_MIN_LOWER_FOR_LOWER:
    {
        value_t max_upper = -DBL_MAX;
        value_t min_lower =  DBL_MAX;
        for(graph_t::OutArcIt out_arc(graph,action_node); out_arc!=INVALID; ++out_arc) {
            node_t state_node = graph.target(out_arc);
            reward_t transition_reward = arc_info_map[out_arc].transition_reward;
            value_t upper = transition_reward + discount*node_info_map[state_node].upper_value_bound;
            value_t lower = transition_reward + discount*node_info_map[state_node].lower_value_bound;
            if(upper>max_upper) {
                max_upper = upper;
            }
            if(lower<min_lower) {
                min_lower = lower;
            }
        }
        if(max_upper==-DBL_MAX||min_lower==DBL_MAX) {
            DEBUG_OUT(0,"Error: No outgoing state node");
        }
        node_info_map[action_node].upper_value_bound = max_upper;
        node_info_map[action_node].lower_value_bound = min_lower;
        break;
    }
    default:
        DEBUG_OUT(0,"Error: Action back-propagation type not implemented.");
        break;
    }

    DEBUG_OUT(3,"Updated action node");
    if(DEBUG_LEVEL>=3) {
        print_node(action_node);
    }

    // return parent state node
    graph_t::InArcIt in_arc(graph,action_node);
    node_t parent_state_node = graph.source(in_arc);
    if(in_arc==INVALID) {
        DEBUG_OUT(0,"Error: No parent state node for this action");
    }
    if(++in_arc!=INVALID) {
        DEBUG_OUT(0,"Error: More than one parent state node for this action");
    }
    if(node_info_map[parent_state_node].type!=STATE) {
        DEBUG_OUT(0,"Error: Parent state node is not of type STATE");
    }
    DEBUG_OUT(4,"    Parent state node:");
    if(DEBUG_LEVEL>=4) {
        print_node(parent_state_node);
    }
    return parent_state_node;
}

LookAheadSearch::node_t LookAheadSearch::update_state_node(node_t state_node) {
    BOUND_USAGE_TYPE use_type = state_back_propagation_type;
    if(use_type==SAME_AS_OPTIMAL_ACTION_SELECTION) {
        use_type = optimal_action_selection_type;
    }
    switch(use_type) {
    case MAX_UPPER_FOR_UPPER_CORRESPONDING_LOWER_FOR_LOWER:
    {
        value_t max_upper_bound= -DBL_MAX, current_upper_bound, current_lower_bound;
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
            node_t action_node = graph.target(out_arc);
            current_upper_bound = node_info_map[action_node].upper_value_bound;
            current_lower_bound = node_info_map[action_node].lower_value_bound;
            if(
                (!approx_eq(current_upper_bound,max_upper_bound) && // if upper bounds are NOT equal
                 current_upper_bound>max_upper_bound ) || // primary criterion: upper bound
                ( approx_eq(current_upper_bound,max_upper_bound) && // if upper bounds are equal
                  current_lower_bound>node_info_map[state_node].lower_value_bound ) // secondary criterion: lower bounds
                ) {
                max_upper_bound=current_upper_bound;
                node_info_map[state_node].upper_value_bound = current_upper_bound;
                node_info_map[state_node].lower_value_bound = current_lower_bound;
            }
        }
        break;
    }
    case MAX_WEIGHTED_BOUNDS:
    {
        value_t max_value= -DBL_MAX;
        for(graph_t::OutArcIt out_arc(graph,state_node); out_arc!=INVALID; ++out_arc) {
            node_t action_node = graph.target(out_arc);
            value_t current_upper_bound = node_info_map[action_node].upper_value_bound;
            value_t current_lower_bound = node_info_map[action_node].lower_value_bound;
            value_t current_value = lower_bound_weight*current_lower_bound + (1-lower_bound_weight)*current_upper_bound;
            if(current_value>max_value) { // no randomization needed for equal values since actual action is not considered
                max_value=current_value;
                node_info_map[state_node].upper_value_bound = current_upper_bound;
                node_info_map[state_node].lower_value_bound = current_lower_bound;
            }
        }
        break;
    }
    default:
        DEBUG_OUT(0,"Error: State back-propagation type not implemented.");
        break;
    }

    DEBUG_OUT(3,"Updated state node");
    if(DEBUG_LEVEL>=4) {
        print_node(state_node);
    }

    // return parent action node
    if(state_node!=root_node) {
        graph_t::InArcIt in_arc(graph,state_node);
        node_t parent_action_node = graph.source(in_arc);
        if(in_arc==INVALID) {
            DEBUG_OUT(0,"Error: No parent action node for this state");
        }
        if(++in_arc!=INVALID) {
            DEBUG_OUT(0,"Error: More than one parent action node for this state");
        }
        if(node_info_map[parent_action_node].type!=ACTION) {
            DEBUG_OUT(0,"Error: Parent action node is not of type ACTION");
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
    case STATE:
        DEBUG_OUT(0, "    type:    " << "STATE" );
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
        DEBUG_OUT(0,"Error: Unknown expansion");
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

template <>
LookAheadSearch::probability_t LookAheadSearch::get_predict_debug(const Maze& model, const instance_t * i, const action_t&a, const state_t& s, const reward_t& r) const {
    return model.get_prediction(i, a, s, r, true);
}
