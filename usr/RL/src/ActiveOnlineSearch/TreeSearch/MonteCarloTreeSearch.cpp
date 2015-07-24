#include "MonteCarloTreeSearch.h"

#include "TreePolicy.h"
#include "ValueHeuristic.h"
#include "BackupMethod.h"
#include "NodeFinder.h"

#include <vector>
#include <tuple>
#include <functional>
#include <memory>

#include <lemon/adaptors.h> // reverseDigraph()
#include <lemon/maps.h>

#include "../graph_util.h"
#include <util/QtUtil.h>
#include <util/util.h>

#define DEBUG_LEVEL 0
#include <util/debug.h>

#include <util/return_tuple.h>
#include <util/return_tuple_macros.h>

using lemon::INVALID;
using std::tuple;
using std::make_tuple;
using std::vector;
using std::deque;
using std::shared_ptr;
using std::make_shared;

void MonteCarloTreeSearch::RolloutItem::write(std::ostream & out) const {
    out <<
        *(this->action) << " -->	" <<
        *(this->observation) << "	(" <<
        this->reward << ")	" <<
        (this->type==OBSERVATION_NODE?"OBSERVATION":"ACTION     ") << "	" <<
        this->weight << " / " << this->discounted_return;
}

void MonteCarloTreeSearch::MCTSNodeInfo::set_value(reward_t val,
                                                   reward_t val_variance,
                                                   reward_t min_val,
                                                   reward_t max_val) {
    if(val_variance<0) {
        if(fabs(val_variance)<1e-50) {
            DEBUG_ERROR("Negative variance provided: " << val_variance);
        }
        val_variance = -val_variance;
    }
    value=val;
    value_variance=val_variance;
    min_value = min_val;
    max_value = max_val;
}

MonteCarloTreeSearch::MonteCarloTreeSearch(shared_ptr<AbstractEnvironment> environment_,
                                           double discount_,
                                           shared_ptr<node_finder::NodeFinder> node_finder_,
                                           shared_ptr<tree_policy::TreePolicy> tree_policy_,
                                           shared_ptr<value_heuristic::ValueHeuristic> value_heuristic_,
                                           shared_ptr<backup_method::BackupMethod> backup_method_,
                                           BACKUP_TYPE backup_type_,
                                           int rollout_length_,
                                           shared_ptr<tree_policy::TreePolicy> recommendation_policy_,
                                           int max_depth_,
                                           ROLLOUT_STORAGE rollout_storage_,
                                           bool perform_data_backups_):
    SearchTree(environment_, discount_, node_finder_),
    rollout_storage(rollout_storage_),
    mcts_node_info_map(graph),
    mcts_arc_info_map(graph),
    tree_policy(tree_policy_),
    value_heuristic(value_heuristic_),
    backup_method(backup_method_),
    backup_type(backup_type_),
    node_hash(graph),
    recommendation_policy(recommendation_policy_),
    max_depth(max_depth_),
    rollout_length(rollout_length_),
    perform_data_backups(perform_data_backups_)
{
    tree_policy->init(environment,
                      graph,
                      node_info_map,
                      mcts_node_info_map,
                      mcts_arc_info_map);
    value_heuristic->init(discount,
                          environment);
    backup_method->init(discount,
                        environment,
                        graph,
                        node_info_map,
                        mcts_node_info_map,
                        mcts_arc_info_map,
                        perform_data_backups);
    if(recommendation_policy==nullptr) {
        recommendation_policy.reset(new tree_policy::Optimal());
    }
    recommendation_policy->init(environment,
                                graph,
                                node_info_map,
                                mcts_node_info_map,
                                mcts_arc_info_map);
    DEBUG_EXPECT(recommendation_policy!=nullptr);
}

void MonteCarloTreeSearch::next_do() {
    // remember the trajectory
    typedef tuple<node_t,arc_t,node_t,arc_t,reward_t,shared_ptr<RolloutItem>> trajectory_item_t;
    vector<trajectory_item_t> trajectory;

    /* ========================================================================
       (1) follow tree-policy to leaf-node (2) go one more step to expand this
       leaf node (3) stop if newly created node is a leaf or node or goto 1 if
       it is a non-leaf node (only in DAGs)
       ======================================================================== */
    node_t leaf_node = INVALID;
    bool is_real_leaf_node = true;
    {
        DEBUG_OUT(2,"Follow tree-policy...");
        node_t current_node = root_node;
        environment->reset_state();
        node_set_t node_set({root_node},0,node_hash);
        DEBUG_OUT(2,"    Starting at node " << graph.id(root_node));
        for(int depth=0; max_depth<0 || depth<max_depth; ++depth) {

            // get tree-policy action
            action_handle_t action = tree_policy->get_action(current_node);

            // find or create action node
            T(arc_t, to_action_arc,
              node_t, action_node,
              bool, new_action_arc,
              bool, new_action_node) = find_or_create_action_node(current_node, action);

            // perfrom transition
            T(observation_handle_t, observation_to, reward_t, reward) = environment->transition(action);

            // find or create observation node
            T(arc_t, to_observation_arc,
              node_t, observation_node,
              bool, new_observation_arc,
              bool, new_observation_node) = find_or_create_observation_node(action_node, observation_to);
            DEBUG_OUT(2,"    Transition:");
            DEBUG_OUT(2,"        action " << *action << " / node " << graph.id(action_node));
            DEBUG_OUT(2,"        observation " << *observation_to << " / node " << graph.id(observation_node));
            DEBUG_OUT(2,"        reward " << reward);

            // add to trajectory (set discounted return and next-pointer during backpropagation)
            trajectory.push_back(trajectory_item_t(current_node,
                                                   to_action_arc,
                                                   action_node,
                                                   to_observation_arc,
                                                   reward,
                                                   make_shared<RolloutItem>(action,
                                                                            observation_to,
                                                                            reward)));

            // update current node and state
            current_node = observation_node;

            // break in terminal nodes
            if(environment->is_terminal_state()) break;
            // break if new observation node was added
            if(new_observation_node) break;
            // break if a node was visited before during this rollout (this can
            // happen in the case of FullGraph node finder, the problem is that
            // the node data is not up-to-date because it depends on the current
            // rollout) or update node set
            if(node_set.find(current_node)!=node_set.end()) {
                is_real_leaf_node = false;
                break;
            } else {
                node_set.insert(current_node);
            }
        }
        DEBUG_OUT(2,(is_real_leaf_node?"...reached leaf-node":"...reached closed loop"));
        leaf_node = current_node;
    }

    /* =======================================
       perform a rollout
       ======================================= */
    shared_ptr<RolloutItem> leaf_node_rollout_item;
    switch(rollout_storage) {
    case ROLLOUT_STORAGE::NONE:
        leaf_node_rollout_item = rollout(leaf_node);
        leaf_node_rollout_item->next = nullptr;
        break;
    case ROLLOUT_STORAGE::CONDENSED:
        leaf_node_rollout_item = rollout(leaf_node);
        break;
    case ROLLOUT_STORAGE::FULL:
    {
        // explicit rollout
        node_t current_node = leaf_node;
        for(int depth=0; (max_depth<0 || depth<max_depth) && !environment->is_terminal_state(); ++depth) {

            // get tree-policy action
            action_handle_t action = util::random_select(environment->get_actions());

            // find or create action node
            T(arc_t, to_action_arc,
              node_t, action_node,
              bool, new_action_arc,
              bool, new_action_node) = find_or_create_action_node(current_node, action);

            // perfrom transition
            T(observation_handle_t, observation_to, reward_t, reward) = environment->transition(action);

            // find or create observation node
            T(arc_t, to_observation_arc,
              node_t, observation_node,
              bool, new_observation_arc,
              bool, new_observation_node) = find_or_create_observation_node(action_node, observation_to);

            // add to trajectory (set discounted return and next-pointer during backpropagation)
            trajectory.push_back(trajectory_item_t(current_node,
                                                   to_action_arc,
                                                   action_node,
                                                   to_observation_arc,
                                                   reward,
                                                   make_shared<RolloutItem>(action,
                                                                            observation_to,
                                                                            reward)));

            // update current node and state
            current_node = observation_node;
        }
        mcts_node_info_map[current_node].set_value(0,0,0,0);
        leaf_node_rollout_item = make_shared<RolloutItem>();
        leaf_node = current_node;
    }
    }

    /* =======================================
       get a heuristic value estimate
       ======================================= */
    // first add rollout to leaf node (also reinitialize weights)
    add_rollout(leaf_node, leaf_node_rollout_item);
    init_rollout_weights(leaf_node);
    // get value
    value_heuristic->add_value_estimate(leaf_node,
                                        mcts_node_info_map);

    /* =======================================
       backpropagate
       ======================================= */

    DEBUG_OUT(2,"Backup nodes...");

    /* We need to propagate back the returns to allow MC backups. If backup_type
     * is BACKUP_TRACE we also do the backups here. In that case only nodes that
     * lie on the trajectory will be backed up. (In trees this is the only way
     * but in general BACKUP_PROPAGATE will backup more nodes.) */
    shared_ptr<RolloutItem> root_node_rollout_item;
    {
        // initialize next rollout item with leaf-node's rollout item
        shared_ptr<RolloutItem> next_rollout_item = leaf_node_rollout_item;
        shared_ptr<RolloutItem> current_rollout_item;
        // follow the trace back to root node
        node_t observation_node, action_node;
        arc_t to_action_arc, to_observation_arc;
        reward_t reward;
        for(auto transition=trajectory.rbegin(); transition!=trajectory.rend(); ++transition) {
            t(observation_node,to_action_arc,action_node,to_observation_arc,reward,current_rollout_item) = *transition;
            // calculate discounted return
            reward_t discounted_return = reward + discount*next_rollout_item->discounted_return;
            DEBUG_OUT(2,discounted_return << " = " << reward << " + " << discount << "⋅" << current_rollout_item->discounted_return);
            // update
            add_transition(observation_node,
                           to_action_arc,
                           action_node,
                           to_observation_arc,
                           reward);
            auto action_rollout_item = make_shared<RolloutItem>(*current_rollout_item);
            {
                current_rollout_item->next = action_rollout_item;
                action_rollout_item->next = next_rollout_item;
                current_rollout_item->discounted_return = discounted_return;
                action_rollout_item->discounted_return = discounted_return;
                action_rollout_item->type = ACTION_NODE;
                current_rollout_item->is_new = false;
            }
            add_rollout(observation_node, current_rollout_item);
            add_rollout(action_node, action_rollout_item);
            // update next-pointer
            next_rollout_item = current_rollout_item;
            // debug info
            DEBUG_OUT(2,QString("    update observation-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(observation_node)).
                      arg(mcts_node_info_map[observation_node].action_counts).
                      arg(mcts_node_info_map[observation_node].rollout_counts).
                      arg(mcts_node_info_map[observation_node].return_sum));
            DEBUG_OUT(2,QString("    update action-node(%1):	counts=%2/%3	return_sum=%4").
                      arg(graph.id(action_node)).
                      arg(mcts_node_info_map[action_node].action_counts).
                      arg(mcts_node_info_map[action_node].rollout_counts).
                      arg(mcts_node_info_map[action_node].return_sum));
            DEBUG_OUT(2,QString("    reward=%1, return=%2").arg(reward).arg(discounted_return));
            // backup if back_type is BACKUP_TRACE
            if(backup_type==BACKUP_TYPE::TRACE) {
                backup_method->backup_action_node(action_node);
                backup_method->backup_observation_node(observation_node);
            }
        }
        root_node_rollout_item = current_rollout_item;
    }

    if(backup_type==BACKUP_TYPE::TRACE) {
        // backups were done above
    } else if(backup_type==BACKUP_TYPE::PROPAGATE) {
        /* Backups will be propagated through the graph. */

        DEBUG_OUT(1,"Propagate backups from leaf-node " << graph.id(leaf_node));

        // construct check-change function
        if(old_values==nullptr) {
            old_values.reset(new mcts_node_info_map_t(graph));
            lemon::mapCopy(graph, mcts_node_info_map, *old_values);
        }
        graph_t::NodeMap<bool> changed(graph,true); // re-initialize every time
        std::function<bool(node_t node)> check_changed_function = [&](node_t node) {
            DEBUG_OUT(2,"Check node " << graph.id(node));
            bool return_value = changed[node] || !equal((*old_values)[node], mcts_node_info_map[node]);
            update(mcts_node_info_map[node], (*old_values)[node]);
            changed[node] = false;
            DEBUG_OUT(3,"    node " << graph.id(node) << (return_value?" CHANGED":" no change"));
            return return_value;
        };

        // construct graph propagation object, set source, check-change
        // function, and initialize
        auto graph_propagation = graph_util::GraphPropagationFactory(lemon::reverseDigraph(graph));
        graph_propagation.
            add_source(leaf_node).
            allow_incomplete_updates(true).
            set_check_change_function(check_changed_function).
            init();

        // process nodes
        int update_counter = 0;
        int max_updates = 10000*lemon::countNodes(graph);
        for(node_t next_node = leaf_node;
            next_node!=INVALID;
            next_node = graph_propagation.next()) {

            if(out_arc_it_t(graph,next_node)==INVALID) {
                DEBUG_OUT(2,"    skipping leaf-node " << graph.id(next_node));
                continue;
            }

            // update
            switch(node_info_map[next_node].type) {
            case OBSERVATION_NODE:
                backup_method->backup_observation_node(next_node);
                DEBUG_OUT(2,"    update observ.-node " << graph.id(next_node));
                break;
            case ACTION_NODE:
                backup_method->backup_action_node(next_node);
                DEBUG_OUT(2,"    update action-node "      << graph.id(next_node));
                break;
            }

            // debug info
            DEBUG_OUT(3,QString("        counts=%2/%3	return_sum=%4").
                      arg(mcts_node_info_map[next_node].action_counts).
                      arg(mcts_node_info_map[next_node].rollout_counts).
                      arg(mcts_node_info_map[next_node].return_sum));

            // "emergency" break
            if(++update_counter>max_updates) {
                DEBUG_WARNING("Interrupt backup propagation after " << max_updates << " updates");
                break;
            }
        }
    } else DEBUG_DEAD_LINE;

    // print rollout
    IF_DEBUG(3) {
        DEBUG_OUT(1,"Trace");
        shared_ptr<RolloutItem> current_rollout_item = root_node_rollout_item;
        while(current_rollout_item->next!=nullptr) {
            DEBUG_OUT(1,"    " << *current_rollout_item);
            current_rollout_item = current_rollout_item->next;
        }
        DEBUG_OUT(1,"    final weight: " << current_rollout_item->weight);
    }

    // backup root node (for global models)
    backup_method->backup_root(root_node);
}

MonteCarloTreeSearch::action_handle_t MonteCarloTreeSearch::recommend_action() const {
    DEBUG_EXPECT(recommendation_policy!=nullptr);
    return recommendation_policy->get_action(root_node);
    // old version
    environment->reset_state();
    vector<action_handle_t> optimal_actions({*(environment->get_actions().begin())});
    double max_value = -DBL_MAX;
    for(out_arc_it_t arc(graph, root_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        double value = mcts_node_info_map[action_node].value;
        if(value>max_value) {
            optimal_actions.clear();
            max_value = value;
        }
        if(value>=max_value) {
            optimal_actions.push_back(node_info_map[action_node].action);
        }
    }
    return util::random_select(optimal_actions);
}

MonteCarloTreeSearch::action_value_list_t MonteCarloTreeSearch::get_action_values() const {
    action_value_list_t action_values;
    for(out_arc_it_t arc(graph,root_node); arc!=INVALID; ++arc) {
        node_t action_node = graph.target(arc);
        action_handle_t action = node_info_map[action_node].action;
        double value = mcts_node_info_map[action_node].value;
        action_values.push_back(make_tuple(action,value));
    }
    return action_values;
}

void MonteCarloTreeSearch::plot_graph(const char* file_name,
                                      const char* command,
                                      const char* parameters,
                                      bool delete_dot_file) const {

    //-----------------------------------------//
    // get min and max value/reward and counts //
    //-----------------------------------------//
    double min_val = DBL_MAX;
    double max_val = -DBL_MAX;
    int max_counts = 0;
    for(node_it_t node(graph); node!=INVALID; ++node) {
        min_val = std::min(min_val, mcts_node_info_map[node].value);
        max_val = std::max(max_val, mcts_node_info_map[node].value);
        max_counts = std::max(max_counts, mcts_node_info_map[node].action_counts);
    }
    double norm = std::max(fabs(min_val), fabs(max_val));
    norm = norm==0?1:norm;

    graph_t::NodeMap<QString> node_map(graph);
    for(node_it_t node(graph); node!=INVALID; ++node) {
        double value = mcts_node_info_map[node].value;
        QString node_label = QString(R"(
<TABLE BORDER="0" CELLBORDER="0" CELLPADDING="0" CELLSPACING="0">
    <TR><TD ALIGN="center" COLSPAN="5" CELLPADDING="4"><B>%1</B></TD></TR>
    <TR><TD ALIGN="right"> id </TD><TD ALIGN="left"> %4         </TD></TR>
    <TR><TD ALIGN="right"> #a </TD><TD ALIGN="left"> %2         </TD><TD>&nbsp;&nbsp;</TD>
        <TD ALIGN="right"> #r </TD><TD ALIGN="left"> %6         </TD></TR>
    <TR><TD ALIGN="right"> V  </TD><TD ALIGN="left"> %3         </TD><TD></TD>
        <TD ALIGN="right"> V~ </TD><TD ALIGN="left"> %7         </TD></TR>
    <TR><TD ALIGN="right"> V↹ </TD><TD ALIGN="left" COLSPAN="4"> %11 / %12  </TD></TR>
    <TR><TD ALIGN="right"> R  </TD><TD ALIGN="left"> %5         </TD><TD></TD>
        <TD ALIGN="right"> R² </TD><TD ALIGN="left"> %8         </TD></TR>
    <TR><TD ALIGN="right"> R↹ </TD><TD ALIGN="left" COLSPAN="4"> %9 / %10   </TD></TR>
    <TR><TD ALIGN="right"> #⤳ </TD><TD ALIGN="left"> %13       </TD></TR>
</TABLE>)").
            arg(str_html(node)).
            arg(mcts_node_info_map[node].action_counts).
            arg(value,0,'g',4).
            arg(graph.id(node)).
            arg(mcts_node_info_map[node].return_sum,0,'g',4).
            arg(mcts_node_info_map[node].rollout_counts).
            arg(mcts_node_info_map[node].value_variance,0,'g',4).
            arg(mcts_node_info_map[node].squared_return_sum,0,'g',4).
            arg(mcts_node_info_map[node].min_return,0,'g',4).
            arg(mcts_node_info_map[node].max_return,0,'g',4).
            arg(mcts_node_info_map[node].max_value,0,'g',4).
            arg(mcts_node_info_map[node].max_value,0,'g',4).
            arg(mcts_node_info_map[node].rollout_set.size());
        node_map[node] = QString(R"(shape=%1 label=<%2> fillcolor="%3 %4 1" penwidth=%5)").
            arg(node_info_map[node].type==OBSERVATION_NODE?"square":"circle").
            arg(node_label).
            arg(value>0?0.3:0).
            arg(color_rescale(fabs(value/norm))).
            arg(10.*mcts_node_info_map[node].action_counts/max_counts+0.1);
    }

    graph_t::ArcMap<QString> arc_map(graph);
    for(arc_it_t arc(graph); arc!=INVALID; ++arc) {
        node_t source = graph.source(arc);
        double value = mcts_arc_info_map[arc].reward_sum/mcts_arc_info_map[arc].transition_counts;
        QString arc_label = QString(R"(
<TABLE BORDER="0" CELLPADDING="0" CELLSPACING="0">
    <TR><TD ALIGN="right"> #  </TD><TD ALIGN="left"> %1      </TD></TR>
    <TR><TD ALIGN="right"> r  </TD><TD ALIGN="left"> %2      </TD></TR>
    <TR><TD ALIGN="right"> r² </TD><TD ALIGN="left"> %3      </TD></TR>
    <TR><TD ALIGN="right"> r↹ </TD><TD ALIGN="left"> %4 / %5 </TD></TR>
</TABLE>
)").
            arg(mcts_arc_info_map[arc].transition_counts).
            arg(mcts_arc_info_map[arc].reward_sum,0,'g',3).
            arg(mcts_arc_info_map[arc].squared_reward_sum,0,'g',3).
            arg(mcts_arc_info_map[arc].min_reward,0,'g',3).
            arg(mcts_arc_info_map[arc].max_reward,0,'g',3);
        if(node_info_map[source].type==OBSERVATION_NODE) {
            arc_map[arc] = QString(R"(style=dashed label=<%1> color="%2 %3 %3" len=1)").
                arg(arc_label).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm)));
        } else {
            arc_map[arc] = QString(R"(style=solid label=<%1> color="%2 %3 %3" len=2 penwidth=%4)").
                arg(arc_label).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(5.*mcts_arc_info_map[arc].transition_counts/mcts_node_info_map[source].action_counts+0.1);
        }
    }

    util::plot_graph(file_name,
                     graph,
                     "style=filled truecolor=true width=0 height=0 margin=0",
                     &node_map,
                     "",
                     &arc_map,
                     "splines=true overlap=false",
                     delete_dot_file,
                     command,
                     parameters);
}

const MonteCarloTreeSearch::mcts_node_info_map_t & MonteCarloTreeSearch::get_mcts_node_info_map() const {
    return mcts_node_info_map;
}
const MonteCarloTreeSearch::mcts_arc_info_map_t & MonteCarloTreeSearch::get_mcts_arc_info_map() const {
    return mcts_arc_info_map;
}

void MonteCarloTreeSearch::data_backups(bool b) {
    perform_data_backups = b;
    backup_method->perform_data_backups = b;
}

MonteCarloTreeSearch::arc_node_t MonteCarloTreeSearch::find_or_create_observation_node(const node_t & action_node,
                                                                                       const observation_handle_t & observation) {
    auto return_value = SearchTree::find_or_create_observation_node(action_node, observation);
    RETURN_TUPLE(arc_t, to_observation_arc,
                 node_t, to_observation_node,
                 bool, new_arc,
                 bool, new_node) = return_value;
    if(new_arc && rollout_storage==ROLLOUT_STORAGE::CONDENSED) {
        arc_t to_action_arc = in_arc_it_t(graph,action_node);
        node_t from_observation_node = graph.source(to_action_arc);
        node_t via_action_node = action_node;
        bool transferred = transfer_rollouts(from_observation_node,
                                             to_action_arc,
                                             via_action_node,
                                             to_observation_arc,
                                             to_observation_node);
        if(transferred) {
            deque<node_t> to_be_procesed({to_observation_node});
            while(!to_be_procesed.empty()) {
                from_observation_node = to_be_procesed.front();
                to_be_procesed.pop_front();
                for(out_arc_it_t to_action_arc(graph,from_observation_node); to_action_arc!=INVALID; ++to_action_arc) {
                    via_action_node = graph.target(to_action_arc);
                    for(out_arc_it_t to_observation_arc(graph,via_action_node); to_observation_arc!=INVALID; ++to_observation_arc) {
                        to_observation_node = graph.target(to_observation_arc);
                        // DEBUG_WARNING("Interrupt transfer from " <<
                        //               graph.id(from_observation_node) << " via " <<
                        //               graph.id(via_action_node) << " to " <<
                        //               graph.id(to_observation_node));
                        transferred = transfer_rollouts(from_observation_node,
                                                        to_action_arc,
                                                        via_action_node,
                                                        to_observation_arc,
                                                        to_observation_node);
                        if(transferred) {
                            to_be_procesed.push_back(to_observation_node);
                        }
                    }
                }
            }
        }
    }
    return return_value;
}

bool MonteCarloTreeSearch::transfer_rollouts(node_t from_observation_node,
                                             arc_t to_action_arc,
                                             node_t via_action_node,
                                             arc_t to_observation_arc,
                                             node_t to_observation_node) {
    DEBUG_EXPECT(node_info_map[from_observation_node].type==OBSERVATION_NODE);
    DEBUG_EXPECT(node_info_map[via_action_node].type==ACTION_NODE);
    DEBUG_EXPECT(node_info_map[to_observation_node].type==OBSERVATION_NODE);
    auto action = node_info_map[via_action_node].action;
    auto observation = node_info_map[to_observation_node].observation;
    auto & rollout_set = mcts_node_info_map[from_observation_node].rollout_set;
    int transferred_rollouts = 0;
    DEBUG_OUT(1,"Transferring rollouts from " <<
              graph.id(from_observation_node) << " via " <<
              graph.id(via_action_node) << " to " <<
              graph.id(to_observation_node));
    for(auto rollout_item : rollout_set) {
        DEBUG_EXPECT(rollout_item->type==OBSERVATION_NODE);
        if(!(rollout_item->type==OBSERVATION_NODE)) {
            DEBUG_ERROR("");
        }
        // skip old and terminal rollouts
        if(!rollout_item->is_new) {
            DEBUG_OUT(2,"    skip old rollout");
            continue;
        }
        if(rollout_item->next==nullptr) {
            DEBUG_OUT(2,"    skip terminal rollout");
            rollout_item->is_new = false;
            continue;
        }
        IF_DEBUG(1) {
            auto item = rollout_item;
            int steps = 0;
            while(item!=nullptr) {
                DEBUG_EXPECT(item->type==OBSERVATION_NODE);
                if(!(item->type==OBSERVATION_NODE)) {
                    DEBUG_ERROR("Type " << steps << " step ahead is not OBSERVATION_NODE");
                }
                item = item->next;
                ++steps;
            }
        }
        // check if rollout took the required path
        if(*action==*(rollout_item->action) && *observation==*(rollout_item->observation)) {
            // mark as done
            rollout_item->is_new = false;
            DEBUG_OUT(2,"    match");
            DEBUG_OUT(2,"        " << *action << " ←→ " << *(rollout_item->action));
            DEBUG_OUT(2,"        " << *observation << " ←→ " << *(rollout_item->observation));
            // update transition as for backups but add rollout only to action
            // node for now (adding to next observation node is done see below
            // -- this is different from backups)
            add_transition(from_observation_node,
                           to_action_arc,
                           via_action_node,
                           to_observation_arc,
                           rollout_item->reward);
            auto action_rollout_item = make_shared<RolloutItem>(*rollout_item);
            {
                action_rollout_item->next = rollout_item->next;
                rollout_item->next = action_rollout_item;
                action_rollout_item->type = ACTION_NODE;
            }
            add_rollout(via_action_node, action_rollout_item);
            // go to next rollout item
            rollout_item = action_rollout_item->next;
            DEBUG_EXPECT(rollout_item!=nullptr);
            // now also add return to observation node
            add_rollout(to_observation_node, rollout_item);
            // increment counts
            ++transferred_rollouts;
        } else {
            DEBUG_OUT(2,"    nomatch");
            DEBUG_OUT(2,"        " << *action << " ←→ " << *(rollout_item->action));
            DEBUG_OUT(2,"        " << *observation << " ←→ " << *(rollout_item->observation));
        }
    }
    DEBUG_OUT(1,"    " << transferred_rollouts << " rollouts transferred");
    return transferred_rollouts>0;
}
void MonteCarloTreeSearch::add_transition(node_t from_observation_node,
                                          arc_t to_action_arc,
                                          node_t action_node,
                                          arc_t to_observation_arc,
                                          reward_t reward) {
    DEBUG_OUT(1,"add transition");
    DEBUG_OUT(1,"    observ. " << graph.id(from_observation_node));
    DEBUG_OUT(1,"    action  " << graph.id(action_node));
    DEBUG_OUT(1,"    reward  " << reward);
    // action / transition counts
    mcts_node_info_map[from_observation_node].action_counts++;
    mcts_arc_info_map[to_action_arc         ].transition_counts++;
    mcts_node_info_map[action_node          ].action_counts++;
    mcts_arc_info_map[to_observation_arc    ].transition_counts++;
    // reward statistics
    mcts_arc_info_map[to_action_arc         ].reward_sum += reward;
    mcts_arc_info_map[to_action_arc         ].squared_reward_sum += pow(reward,2);
    mcts_arc_info_map[to_action_arc         ].min_reward = std::min(reward,mcts_arc_info_map[to_action_arc].min_reward);
    mcts_arc_info_map[to_action_arc         ].max_reward = std::max(reward,mcts_arc_info_map[to_action_arc].max_reward);
    mcts_arc_info_map[to_observation_arc    ].reward_sum += reward;
    mcts_arc_info_map[to_observation_arc    ].squared_reward_sum += pow(reward,2);
    mcts_arc_info_map[to_observation_arc    ].min_reward = std::min(reward,mcts_arc_info_map[to_observation_arc].min_reward);
    mcts_arc_info_map[to_observation_arc    ].max_reward = std::max(reward,mcts_arc_info_map[to_observation_arc].max_reward);
}

void MonteCarloTreeSearch::add_rollout(node_t node,
                                       shared_ptr<RolloutItem> rollout) {
    DEBUG_OUT(1,"add rollout to node " << graph.id(node) << " (" << rollout->discounted_return << ")");
    DEBUG_EXPECT(rollout!=nullptr);
    DEBUG_EXPECT(rollout->type==node_info_map[node].type);
    if(!(rollout->type==node_info_map[node].type)) {
        DEBUG_ERROR("");
    }
    mcts_node_info_map[node].rollout_set.insert(rollout);
    mcts_node_info_map[node].rollout_counts++;
    mcts_node_info_map[node].return_sum += rollout->discounted_return;
    mcts_node_info_map[node].squared_return_sum += pow(rollout->discounted_return,2);
    mcts_node_info_map[node].min_return = std::min(rollout->discounted_return,mcts_node_info_map[node].min_return);
    mcts_node_info_map[node].max_return = std::max(rollout->discounted_return,mcts_node_info_map[node].max_return);
}

shared_ptr<MonteCarloTreeSearch::RolloutItem> MonteCarloTreeSearch::rollout(node_t leaf_node) {
    if(environment->is_terminal_state()) {
        // no rollout if we're in a terminal state
        return make_shared<RolloutItem>();
    } else {
        // do rollout of specified length (rollout_length).
        int length = rollout_length;

        // If rollout_length is not specified (smaller than zero) and
        // environment does not have a terminal state do one-step
        // rollout. Otherwise do infinite rollout (util terminal state).
        if(length<0 && !environment->has_terminal_state()) {
            length = 1;
        }

        // do the rollout
        observation_handle_t observation;
        action_handle_t action;
        reward_t reward;
        reward_t discounted_return = 0;
        double discount_factor = 1;
        int k=0;
        vector<shared_ptr<RolloutItem>> rollout_list;
        while((length<0 || k<length) && !environment->is_terminal_state()) {
            // select action uniformly from available actions
            action = util::random_select(environment->get_actions());
            // sample transition
            t(observation,reward) = environment->transition(action);
            // update return
            discounted_return += discount_factor*reward;
            // update counter and discount factor
            ++k;
            discount_factor*=discount;
            // update last rollout (update discounted return and next item
            // later)
            rollout_list.push_back(make_shared<RolloutItem>(action,observation,reward));
        }

        // add "empty" rollout item for last node
        rollout_list.push_back(make_shared<RolloutItem>());

        // update discounted return and next pointers
        {
            reward_t reverse_discounted_return = 0;
            shared_ptr<RolloutItem> next = nullptr;
            for(auto rollout_item_it=rollout_list.rbegin(); rollout_item_it!=rollout_list.rend(); ++rollout_item_it) {
                reverse_discounted_return = (*rollout_item_it)->reward + discount * reverse_discounted_return;
                (*rollout_item_it)->discounted_return = reverse_discounted_return;
                (*rollout_item_it)->next = next;
                next = (*rollout_item_it);
            }
            DEBUG_EXPECT_APPROX(reverse_discounted_return,discounted_return);
        }
        return rollout_list.front();
    }
}

void MonteCarloTreeSearch::init_rollout_weights(node_t node) {
    auto & rollout_set = mcts_node_info_map[node].rollout_set;
    double weight = 1./rollout_set.size();
    for(auto rollout_item : rollout_set) {
        if(rollout_item->next==nullptr || rollout_item->next->weight==-1) {
            rollout_item->weight = weight;
        }
    }
}

double MonteCarloTreeSearch::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}

bool MonteCarloTreeSearch::equal(const MCTSNodeInfo & lhs, const MCTSNodeInfo & rhs) {
     double tolerance = 1e-10;
    if(lhs.action_counts!=rhs.action_counts)                  return false;
    if(lhs.rollout_counts!=rhs.rollout_counts)                return false;
    if(fabs(lhs.value-rhs.value)>tolerance)                   return false;
    if(fabs(lhs.value_variance-rhs.value_variance)>tolerance) return false;
    if(fabs(lhs.min_value-rhs.min_value)>tolerance)           return false;
    if(fabs(lhs.max_value-rhs.max_value)>tolerance)           return false;
    if(lhs.return_sum!=rhs.return_sum)                        return false;
    if(lhs.squared_return_sum!=rhs.squared_return_sum)        return false;
    if(lhs.min_return!=rhs.min_return)                        return false;
    if(lhs.max_return!=rhs.max_return)                        return false;
    //if(lhs.rollout_set!=rhs.rollout_set)                      return false;
    return true;
 }

void MonteCarloTreeSearch::update(const MCTSNodeInfo & from, MCTSNodeInfo & to) {
    to.action_counts      = from.action_counts;
    to.rollout_counts     = from.rollout_counts;
    to.value              = from.value;
    to.value_variance     = from.value_variance;
    to.min_value          = from.min_value;
    to.max_value          = from.max_value;
    to.return_sum         = from.return_sum;
    to.squared_return_sum = from.squared_return_sum;
    to.min_return         = from.min_return;
    to.max_return         = from.max_return;
    //to.rollout_set        = from.rollout_set;
}
