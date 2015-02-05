#include "SearchTree.h"

#include <algorithm> // for std::max
#include <set>

#include <lemon/adaptors.h>     // for Undirector adapter
#include <lemon/connectivity.h> // for connected components

#include <QFile>
#include <QTextStream>

#include <util/util.h>
#include <util/QtUtil.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

using std::vector;
using std::list;
using std::set;
using std::get;
using std::tuple;
using std::make_tuple;
using lemon::INVALID;
using util::random_select;

SearchTree::SearchTree(const state_t & s, std::shared_ptr<Environment> env, double d):
    node_info_map(graph),
    arc_info_map(graph),
    discount(d),
    environment(env)
{
    root_node = graph.addNode();
    node_info_map[root_node].state=s;
}

void SearchTree::perform_rollout() {
    //---------------------//
    // generate trajectory //
    //---------------------//
    trajectory_t trajectory;
    node_t state_node = root_node;
    // follow tree policy to terminal node
    bool is_terminal = false;
    while(!is_terminal) {
        action_t action = tree_policy(state_node, is_terminal);
        auto state_reward = environment->sample(node_info_map[state_node].state, action);
        state_t state = std::get<0>(state_reward);
        reward_t reward = std::get<1>(state_reward);
        auto item = add_sample(state_node, action, state, reward);
        state_node = std::get<4>(item);
        trajectory.push_front(item);
    }

    // backpropagate
    for(auto it : trajectory) {
        update_model(get<0>(it), get<1>(it), get<2>(it), get<3>(it));
    }
}

SearchTree::action_t SearchTree::recommend_action() const {
    vector<action_t> optimal_actions;
    double max_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        double val = node_info_map[action_node].value;
        if(val>max_value) {
            optimal_actions.clear();
            max_value = val;
        }
        if(val>=max_value) {
            optimal_actions.push_back(node_info_map[action_node].action);
        }
    }
    return random_select(optimal_actions);
}

void SearchTree::prune(const action_t & action, const state_t & state) {
    //--------------------//
    // find new root node //
    //--------------------//

    node_t new_root_node = INVALID;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        if(node_info_map[action_node].action==action) {
            for(out_arc_it_t arc(graph, action_node); arc!=INVALID && new_root_node==INVALID; ++arc) {
                node_t state_node = graph.target(arc);
                if(node_info_map[state_node].state==state) {
                    new_root_node = state_node;
                }
            }
        } else {
            DEBUG_EXPECT(0,new_root_node==INVALID);
        }
    }

    // check if new root node was found
    if(new_root_node==INVALID) {
        DEBUG_WARNING("No branch for (action --> state) = (" << action << " --> " << state << ")");
        graph.clear();
        root_node = graph.addNode();
        node_info_map[root_node].state=state;
        return;
    }

    //------------//
    // prune tree //
    //------------//

    // remove incoming arc from new root node to split graph in two connected
    // components
    {
        vector<arc_t> arcs_to_erase;
        for(in_arc_it_t arc(graph, new_root_node); arc!=INVALID; ++arc) {
            arcs_to_erase.push_back(arc);
        }
        for(auto arc : arcs_to_erase) {
            graph.erase(arc);
        }
    }

    // compute connected components (need undirected graph to use standard
    // algorithms) and erase all node that are not in the component of the new
    // root node
    {
        // compute components
        typedef lemon::Undirector<graph_t> ugraph_t;
        ugraph_t ugraph(graph);
        ugraph_t::NodeMap<int> component_map(ugraph);
        int component_n = lemon::connectedComponents(ugraph,component_map);
        DEBUG_EXPECT(0,component_n>1);

        // find and erase nodes
        vector<node_t> nodes_to_erase;
        int main_component = component_map[new_root_node];
        for(node_it_t node(graph); node!=INVALID; ++node) {
            if(component_map[node]!=main_component) {
                nodes_to_erase.push_back(node);
            }
        }
        for(node_t node : nodes_to_erase) {
            graph.erase(node);
        }

        // check graph structure TODO: We have a DAG so this does not work:
        // if(!lemon::tree(ugraph))...
    }

    // set root node
    root_node = new_root_node;
}

void SearchTree::toPdf(const char* file_name) const {

    //-------------------//
    // random file names //
    //-------------------//
    //QString dot_file_name = util::random_alpha_num(50)+".dot";
    QString dot_file_name = "tree.dot";
    QString graphics_file_name = file_name;

    //-----------------------------------------//
    // get min and max value/reward and counts //
    //-----------------------------------------//
    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    int max_counts = 0;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        min_val = std::min(min_val, node_info_map[node].value);
        max_val = std::max(max_val, node_info_map[node].value);
        max_counts = std::max(max_counts, node_info_map[node].counts);
    }
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        if(node_info_map[graph.source(arc)].type==ACTION_NODE) {
            min_val = std::min(min_val, arc_info_map[arc].mean_reward);
            max_val = std::max(max_val, arc_info_map[arc].mean_reward);
        }
    }
    double norm = std::max(fabs(min_val), fabs(max_val));
    norm = norm==0?1:norm;

    //------------------//
    // the file content //
    //------------------//
    // start graph
    QString dot("digraph G {\n");
    // add nodes
    for(node_it_t node(graph); node!=INVALID; ++node) {
        double norm_val = node_info_map[node].value/norm;
        dot += QString("\tn%1[fixedsize=shape shape=%2 label=<%3> style=filled fillcolor=\"%4 %5 1\" penwidth=%6 truecolor=true]\n").
            arg(graph.id(node)).
            arg(node_info_map[node].type==STATE_NODE?"square":"circle").
            arg(str_rich(node)).
            arg(norm_val>0?0.3:0).
            arg(color_rescale(fabs(norm_val))).
            arg(10.*node_info_map[node].counts/max_counts+1);
    }
    // add arcs
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        node_t source = graph.source(arc);
        node_t target = graph.target(arc);
        if(node_info_map[source].type==STATE_NODE) {
            dot += QString("\tn%1 -> n%2[style=dashed]\n").
                arg(graph.id(source)).
                arg(graph.id(target));
        } else {
            double norm_val = arc_info_map[arc].mean_reward/norm;
            dot += QString("\tn%1 -> n%2[penwidth=%3 color=\"%4 %5 %5\"]\n").
                arg(graph.id(source)).
                arg(graph.id(target)).
                arg(5*arc_info_map[arc].probability).
                arg(norm_val>0?0.3:0).
                arg(color_rescale(fabs(norm_val)));
        }
    }
    dot += "}";

    //-----------------------------//
    // create file and write to it //
    //-----------------------------//
    {
        QFile outfile(dot_file_name);
        if(!outfile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            DEBUG_ERROR("Could not open file");
        }
        QTextStream dot_file_stream(&outfile);
        dot_file_stream << dot;
    }

    //------------------------------//
    // call dot to generate graphic //
    //------------------------------//
    system(QString("dot -Tpdf -o %1 %2").arg(graphics_file_name).arg(dot_file_name).toLatin1());

    //-----------------//
    // remove dot file //
    //-----------------//
    //remove(dot_file_name.toLatin1());
}

SearchTree::trajectory_item_t  SearchTree::add_sample(const node_t & node,
                                                      const action_t & action,
                                                      const state_t & state,
                                                      const reward_t & reward) {
    // node must be a state node
    DEBUG_EXPECT(0,node_info_map[node].type==STATE_NODE);

    // the (potentially newly added) action and state node and the
    // corresponding arcs
    node_t action_node = INVALID;
    node_t state_node = INVALID;
    arc_t to_action_arc = INVALID;
    arc_t to_state_arc = INVALID;

    DEBUG_OUT(1, "node " << str(node) << " (" << action << ", " << state << ", " << reward << ")");

    //---------------------------------------------------------//
    // check if nodes already exist and add nodes if not found //
    //---------------------------------------------------------//
    // iterate through existing action nodes
    bool action_node_found = false;
    bool state_node_found = false;
    for(out_arc_it_t tmp_to_action_arc(graph, node);
        tmp_to_action_arc!=INVALID && (!action_node_found || !state_node_found);
        ++tmp_to_action_arc) {

        node_t tmp_action_node = graph.target(tmp_to_action_arc);

        // action node exists --> remember
        if(node_info_map[tmp_action_node].action==action) {
            action_node = tmp_action_node;
            to_action_arc = tmp_to_action_arc;
            action_node_found = true;
            DEBUG_OUT(1, "    Found action node!");
        }

        // iterate through existing state nodes for this action node
        for(out_arc_it_t tmp_to_state_arc(graph, tmp_action_node);
            tmp_to_state_arc!=INVALID;
            ++tmp_to_state_arc) {

            node_t tmp_state_node = graph.target(tmp_to_state_arc);

            // state node exists --> remember
            if(node_info_map[tmp_state_node].state==state) {
                state_node = tmp_state_node;
                if(tmp_action_node==action_node) {
                    to_state_arc = tmp_to_state_arc;
                }
                state_node_found = true;
                DEBUG_OUT(1, "    Found state node!");
                break;
            }
        }
    }
    // add action node if not found
    if(action_node==INVALID) {
        action_node = graph.addNode();
        node_info_map[action_node].type = ACTION_NODE;
        node_info_map[action_node].action = action;
        node_info_map[action_node].state = state;
        DEBUG_OUT(1, "    Add action node!");
    }
    IF_DEBUG(1) {
        DEBUG_OUT(1, "    Action node: " << str(action_node));
        for(out_arc_it_t tmp_to_state_arc(graph, action_node); tmp_to_state_arc!=INVALID; ++tmp_to_state_arc) {
            DEBUG_OUT(1, "        Arc to state node: " << str(graph.target(tmp_to_state_arc)));
        }
    }
    // add state node if not found
    if(state_node==INVALID) {
        state_node = graph.addNode();
        node_info_map[state_node].type = STATE_NODE;
        node_info_map[state_node].state = state;
        DEBUG_OUT(1, "    Add state node!");
    }
    IF_DEBUG(1) {
        DEBUG_OUT(1, "    State node: " << str(state_node));
        for(out_arc_it_t tmp_to_state_arc(graph, state_node); tmp_to_state_arc!=INVALID; ++tmp_to_state_arc) {
            DEBUG_OUT(1, "        Arc to action node: " << str(graph.target(tmp_to_state_arc)));
        }
    }

    // add state --> action arc if not found
    if(to_action_arc==INVALID) {
        to_action_arc = graph.addArc(node, action_node);
    }
    // add action --> state arc if not found
    if(to_state_arc==INVALID) {
        to_state_arc = graph.addArc(action_node, state_node);
    }

    // update nodes and arcs
    update_data(node, to_action_arc, action_node, to_state_arc, reward);

    return trajectory_item_t(node, to_action_arc, action_node, to_state_arc, state_node);
}

QString SearchTree::str(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    return QString("%1 (%2)").
        arg(is_state_node?"STATE":"ACTION").
        arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
}

QString SearchTree::str_rich(const node_t & n) const {
    bool is_state_node = node_info_map[n].type==STATE_NODE;
    // return QString("<i>%1</i>=%2").
    //     arg(is_state_node?"s":"a").
    //     arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
    return QString("%1").arg(is_state_node?node_info_map[n].state:node_info_map[n].action);
}

double SearchTree::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}

void SearchTree::update_data(const node_t & state_node,
                             const arc_t & state_action_arc,
                             const node_t & action_node,
                             const arc_t & action_state_arc,
                             const reward_t & reward) {
    DEBUG_EXPECT(0, node_info_map[action_node].type==ACTION_NODE);

    // increment counter and reward
    double & mean_reward = arc_info_map[action_state_arc].mean_reward;
    int & action_counts = node_info_map[action_node].counts;
    int & trans_counts = arc_info_map[action_state_arc].counts;
    mean_reward = (trans_counts*mean_reward + reward)/(trans_counts+1);
    action_counts += 1;
    trans_counts += 1;
    arc_info_map[state_action_arc].counts += 1;
    node_info_map[state_node].counts += 1;
}


void SearchTree::update_model(const node_t & state_node,
                              const arc_t & state_action_arc,
                              const node_t & action_node,
                              const arc_t & action_state_arc) {
    DEBUG_EXPECT(0, node_info_map[state_node].type==STATE_NODE);
    DEBUG_EXPECT(0, node_info_map[action_node].type==ACTION_NODE);

    // recompute transition probabilities and action value
    const int & action_counts = node_info_map[action_node].counts;
    double action_value = 0;
    for(out_arc_it_t arc(graph, action_node); arc!=INVALID; ++arc) {
        double p = (double)arc_info_map[arc].counts/action_counts;
        arc_info_map[arc].probability = p;
        action_value += p*(arc_info_map[arc].mean_reward
                           + discount*node_info_map[graph.target(arc)].value);
    }
    node_info_map[action_node].value = action_value;

    // recompute state value
    double state_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
        state_value = std::max(state_value, node_info_map[graph.target(arc)].value);
    }
    node_info_map[state_node].value = state_value;
}

SearchTree::action_t SearchTree::tree_policy(const node_t & state_node, bool & is_terminal) const {
    // get a vector of unsampled actions
    vector<action_t> a_vector;
    vector<tuple<double,action_t>> upper_bounds;
    {
        set<action_t> a_set;
        for(action_t a : environment->actions) {
            a_set.insert(a);
        }
        int state_counts = node_info_map[state_node].counts;
        for(out_arc_it_t arc(graph, state_node); arc!=INVALID; ++arc) {
            auto action_node_info = node_info_map[graph.target(arc)];
            action_t action = action_node_info.action;
            double upper = action_node_info.value;
            upper += sqrt(2*log(state_counts)/action_node_info.counts);
            a_set.erase(action);
            upper_bounds.push_back(make_tuple(upper,action));
        }
        for(action_t a : a_set) {
            a_vector.push_back(a);
        }
    }

    // selec unsampled action if there is any
    if(a_vector.size()>0) {
        is_terminal = true;
        return random_select(a_vector);
    }

    // use UCB1 otherwise
    is_terminal = false;
    std::sort(upper_bounds.begin(),upper_bounds.end());
    vector<tuple<double,action_t>> max_upper_bounds;
    for(auto reverse_it = upper_bounds.rbegin(); reverse_it!=upper_bounds.rend(); ++reverse_it) {
        if(max_upper_bounds.size()==0 || get<0>(max_upper_bounds.back())==get<0>(*reverse_it)){
            max_upper_bounds.push_back(*reverse_it);
        }
    }
    return get<1>(random_select(max_upper_bounds));
}
