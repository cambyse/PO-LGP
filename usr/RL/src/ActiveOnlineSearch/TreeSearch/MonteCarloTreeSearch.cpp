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
using std::shared_ptr;

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

void MonteCarloTreeSearch::MCTSNodeInfo::add_rollout_return(reward_t ret) {
    ++rollout_counts;
    return_sum+=ret;
    squared_return_sum+=ret*ret;
    min_return = std::min(min_return,ret);
    max_return = std::max(max_return,ret);
}

void MonteCarloTreeSearch::MCTSNodeInfo::add_rollout_to_list(rollout_t rollout) {
    DEBUG_EXPECT(0,!rollout.empty());
    rollout_list.push_back(rollout);
}

void MonteCarloTreeSearch::MCTSArcInfo::add_reward(reward_t reward) {
    ++transition_counts;
    reward_sum+=reward;
    squared_reward_sum+=reward*reward;
    min_reward = std::min(min_reward,reward);
    max_reward = std::max(max_reward,reward);
}

MonteCarloTreeSearch::MonteCarloTreeSearch(std::shared_ptr<AbstractEnvironment> environment_,
                                           double discount_,
                                           std::shared_ptr<node_finder::NodeFinder> node_finder_,
                                           std::shared_ptr<tree_policy::TreePolicy> tree_policy_,
                                           std::shared_ptr<value_heuristic::ValueHeuristic> value_heuristic_,
                                           std::shared_ptr<backup_method::BackupMethod> backup_method_,
                                           BACKUP_TYPE backup_type_,
                                           std::shared_ptr<tree_policy::TreePolicy> recommendation_policy_,
                                           int max_depth_,
                                           ROLLOUT_STORAGE rollout_storage_):
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
    max_depth(max_depth_)
{
    tree_policy->init(environment,graph,node_info_map,mcts_node_info_map,mcts_arc_info_map);
    value_heuristic->init(discount,environment);
    backup_method->init(discount,environment,graph,node_info_map,mcts_arc_info_map);
    if(recommendation_policy==nullptr) {
        recommendation_policy.reset(new tree_policy::Optimal());
    }
    recommendation_policy->init(environment,graph,node_info_map,mcts_node_info_map,mcts_arc_info_map);
    DEBUG_EXPECT(0,recommendation_policy!=nullptr);
}

void MonteCarloTreeSearch::next_do() {
    // remember the trajectory
    typedef tuple<node_t,arc_t,node_t,arc_t,reward_t> trajectory_item_t;
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

            // add to trajectory
            trajectory.push_back(trajectory_item_t(current_node, to_action_arc, action_node, to_observation_arc, reward));

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
       get a heuristic value estimate
       ======================================= */
    if(is_real_leaf_node) {
        switch(rollout_storage) {
        case ROLLOUT_STORAGE::NONE:
            value_heuristic->add_value_estimate(leaf_node,
                                                mcts_node_info_map);
            break;
        case ROLLOUT_STORAGE::CONDENSED:
        {
            value_heuristic->add_value_estimate(leaf_node,
                                                mcts_node_info_map);
            auto rollout_heuristic = std::dynamic_pointer_cast<value_heuristic::Rollout>(value_heuristic);
            if(rollout_heuristic!=nullptr && !rollout_heuristic->get_last_rollout().empty()) {
                mcts_node_info_map[leaf_node].add_rollout_to_list(rollout_heuristic->get_last_rollout());
            }
            break;
        }
        case ROLLOUT_STORAGE::FULL:
        {
            // explicit rollout
            node_t current_node = leaf_node;
            for(int depth=0; max_depth<0 || depth<max_depth; ++depth) {

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

                // add to trajectory
                trajectory.push_back(trajectory_item_t(current_node, to_action_arc, action_node, to_observation_arc, reward));

                // update current node and state
                current_node = observation_node;

                // break in terminal nodes
                if(environment->is_terminal_state()) {
                    mcts_node_info_map[current_node].add_rollout_return(0);
                    mcts_node_info_map[current_node].set_value(0,0,0,0);
                    break;
                }
            }
            leaf_node = current_node;
        }
        }
    }

    /* =======================================
       backpropagate
       ======================================= */

    DEBUG_OUT(2,"Backup nodes...");

    /* We need to propagate back the returns to allow MC backups. If backup_type
     * is BACKUP_TRACE we also do the backups here. In that case only nodes that
     * lie on the trajectory will be backed up. (In trees this is the only way
     * but in general BACKUP_PROPAGATE will backup more nodes.) */
    {
        // initialize discounted return of this rollout with leaf-node's return
        reward_t discounted_return = 0;
        if(mcts_node_info_map[leaf_node].rollout_counts>0) {
            discounted_return = mcts_node_info_map[leaf_node].return_sum/mcts_node_info_map[leaf_node].rollout_counts;
        }
        // follow the trace back to root node
        node_t observation_node, action_node;
        arc_t to_action_arc, to_observation_arc;
        reward_t reward;
        for(auto transition=trajectory.rbegin(); transition!=trajectory.rend(); ++transition) {
            t(observation_node,to_action_arc,action_node,to_observation_arc,reward) = *transition;
            // calculate discounted return
            discounted_return = reward + discount*discounted_return;
            // update counts, reward, and return
            add_transition(observation_node, to_action_arc, action_node, to_observation_arc, reward);
            mcts_node_info_map[observation_node  ].add_rollout_return(discounted_return);
            mcts_node_info_map[action_node       ].add_rollout_return(discounted_return);
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
                backup_method->backup(observation_node,
                                      action_node,
                                      mcts_node_info_map);
            }
        }
    }

    if(backup_type==BACKUP_TYPE::TRACE) {
        // backups were done above
    } else if(backup_type==BACKUP_TYPE::PROPAGATE) {
        /* Backups will be propagated through the graph. */

        // construct and initialize graph propagation object
        auto graph_propagation = graph_util::GraphPropagationFactory(lemon::reverseDigraph(graph));
        graph_propagation.add_source(leaf_node).init();

        // process nodes
        for(node_t next_node = leaf_node;
            next_node!=INVALID;
            next_node = graph_propagation.next()) {

            // skip action nodes (will be backed up via observation nodes)
            if(node_info_map[next_node].type==ACTION_NODE) continue;

            // iterate through all observation-action pairs that can lead to
            // this observation node
            for(in_arc_it_t to_observation_arc(graph,next_node);
                to_observation_arc!=INVALID;
                ++to_observation_arc) {
                    node_t action_node = graph.source(to_observation_arc);
                    arc_t to_action_arc = in_arc_it_t(graph,action_node);
                    node_t observation_node = graph.source(to_action_arc);
                    backup_method->backup(observation_node,
                                          action_node,
                                          mcts_node_info_map);
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
            }

        }
    } else DEBUG_DEAD_LINE;

    // backup root node (for global models)
    backup_method->backup_root(root_node,
                               mcts_node_info_map);
}


MonteCarloTreeSearch::action_handle_t MonteCarloTreeSearch::recommend_action() const {
    DEBUG_EXPECT(0,recommendation_policy!=nullptr);
    return recommendation_policy->get_action(root_node);
    // old version
    environment->reset_state();
    std::vector<action_handle_t> optimal_actions({*(environment->get_actions().begin())});
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
<TABLE BORDER="0" CELLPADDING="0" CELLSPACING="0">
    <TR><TD ALIGN="center" COLSPAN="2" CELLPADDING="4"><B>%1</B></TD></TR>
    <TR><TD ALIGN="right"> id </TD><TD ALIGN="left"> %4         </TD></TR>
    <TR><TD ALIGN="right"> #a </TD><TD ALIGN="left"> %2         </TD></TR>
    <TR><TD ALIGN="right"> #r </TD><TD ALIGN="left"> %6         </TD></TR>
    <TR><TD ALIGN="right"> V  </TD><TD ALIGN="left"> %3 ± %7    </TD></TR>
    <TR><TD ALIGN="right"> V↹ </TD><TD ALIGN="left"> %11 / %12  </TD></TR>
    <TR><TD ALIGN="right"> R  </TD><TD ALIGN="left"> %5         </TD></TR>
    <TR><TD ALIGN="right"> R² </TD><TD ALIGN="left"> %8         </TD></TR>
    <TR><TD ALIGN="right"> R↹ </TD><TD ALIGN="left"> %9 / %10   </TD></TR>
    <TR><TD ALIGN="right"> #⤳ </TD><TD ALIGN="left"> %13 (%14)  </TD></TR>
</TABLE>)").
            arg(str_html(node)).
            arg(mcts_node_info_map[node].action_counts).
            arg(value,0,'g',4).
            arg(graph.id(node)).
            arg(mcts_node_info_map[node].return_sum,0,'g',4).
            arg(mcts_node_info_map[node].rollout_counts).
            arg(sqrt(mcts_node_info_map[node].value_variance),0,'g',4).
            arg(mcts_node_info_map[node].squared_return_sum,0,'g',4).
            arg(mcts_node_info_map[node].min_return,0,'g',4).
            arg(mcts_node_info_map[node].max_return,0,'g',4).
            arg(mcts_node_info_map[node].max_value,0,'g',4).
            arg(mcts_node_info_map[node].max_value,0,'g',4).
            arg(mcts_node_info_map[node].rollout_list.size()).
            arg(
                [&](){int sum=0; for(auto roll:mcts_node_info_map[node].rollout_list) {sum+=roll.size();} return sum;}()
                );
        node_map[node] = QString(R"(shape=%1 width=0 height=0 margin=0 label=<%2> fillcolor="%3 %4 1" penwidth=%5)").
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
            arc_map[arc] = QString(R"(style=dashed label=<%1> color="%2 %3 %3")").
                arg(arc_label).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm)));
        } else {
            arc_map[arc] = QString(R"(style=solid label=<%1> color="%2 %3 %3" penwidth=%4)").
                arg(arc_label).
                arg(value>0?0.3:0).
                arg(color_rescale(fabs(value/norm))).
                arg(5.*mcts_arc_info_map[arc].transition_counts/mcts_node_info_map[source].action_counts+0.1);
        }
    }

    util::plot_graph(file_name,
                     graph,
                     "style=filled truecolor=true",
                     &node_map,
                     "",
                     &arc_map,
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
            std::deque<node_t> to_be_procesed({to_observation_node});
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
    DEBUG_EXPECT(0,node_info_map[from_observation_node].type==OBSERVATION_NODE);
    DEBUG_EXPECT(0,node_info_map[via_action_node].type==ACTION_NODE);
    DEBUG_EXPECT(0,node_info_map[to_observation_node].type==OBSERVATION_NODE);
    auto action = node_info_map[via_action_node].action;
    auto observation = node_info_map[to_observation_node].observation;
    auto & rollout_list = mcts_node_info_map[from_observation_node].rollout_list;
    vector<rollout_list_t::iterator> erase_from_list;
    int transferred_rollouts = 0;
    for(auto rollout_it=rollout_list.begin(); rollout_it!=rollout_list.end(); ++rollout_it) {
        // check if rollout took the required path
        if(*action==*(rollout_it->front().action) && *observation==*(rollout_it->front().observation)) {
            // should be removed from parent node to free up space and make
            // future iterations more efficient
            erase_from_list.push_back(rollout_it);
            // update transition as for backups but don't add return to
            // from_observation_node because that was done when it got that
            // rollout added to its list (see below)
            add_transition(from_observation_node,
                           to_action_arc,
                           via_action_node,
                           to_observation_arc,
                           rollout_it->front().reward);
            mcts_node_info_map[via_action_node].add_rollout_return(rollout_it->front().discounted_return);
            // remove first transition (starting from parent node) from rollout
            rollout_it->pop_front();
            if(!rollout_it->empty()) {
                // here we update the return for the observation node
                // (cf. above, where we didn't)
                mcts_node_info_map[to_observation_node].add_rollout_return(rollout_it->front().discounted_return);
                // add rollout to list of to-observation node
                mcts_node_info_map[to_observation_node].add_rollout_to_list(*rollout_it);
                // set flag
                ++transferred_rollouts;
            } else {
                DEBUG_OUT(1,"Not transferring empty rollout from " <<
                          graph.id(from_observation_node) << " via " <<
                          graph.id(via_action_node) << " to " <<
                          graph.id(to_observation_node));
            }
        }
    }
    for(auto it : erase_from_list) {
        rollout_list.erase(it);
    }
    DEBUG_OUT(1,"transferred " << transferred_rollouts << " rollouts from " <<
              graph.id(from_observation_node) << " via " <<
              graph.id(via_action_node) << " to " <<
              graph.id(to_observation_node));
    return transferred_rollouts>0;
}

void MonteCarloTreeSearch::add_transition(node_t from_observation_node,
                                          arc_t to_action_arc,
                                          node_t action_node,
                                          arc_t to_observation_arc,
                                          reward_t reward) {
    mcts_node_info_map[from_observation_node].action_counts++;
    mcts_arc_info_map[to_action_arc         ].add_reward(reward);
    mcts_node_info_map[action_node          ].action_counts++;
    mcts_arc_info_map[to_observation_arc    ].add_reward(reward);
}

double MonteCarloTreeSearch::color_rescale(const double& d) const {
    if(use_sqrt_scale) {
        return sqrt(d);
    } else {
        return d;
    }
}
