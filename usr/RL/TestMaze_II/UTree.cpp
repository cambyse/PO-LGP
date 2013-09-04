#include "UTree.h"
#include "util.h"
#include "util/KolmogorovSmirnovTest.h"
#include "util/ChiSquareTest.h"

#include <queue>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#ifdef BATCH_MODE_QUIET
#define DEBUG_LEVEL 0
#else
#define DEBUG_LEVEL 1
#endif
#define DEBUG_STRING "UTree: "
#include "debug.h"

using std::vector;
using std::map;
using std::set;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;
using std::tuple;
using std::make_tuple;
using std::get;
using std::priority_queue;

using lemon::INVALID;

UTree::NodeInfo::NodeInfo(const Feature * f, const f_ret_t& r):
    instance_vector(0),
    feature(f),
    parent_return_value(r),
    scores_up_to_date(false),
    max_state_action_value(0),
    max_value_action(action_t::STAY),
    statistics_up_to_date(false)
{}

UTree::UTree(const double& d):
        root_node(INVALID),
        node_info_map(graph),
        discount(d)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>(int)-Config::k; --k_idx) {
        // actions
        for(action_t action : actionIt_t::all) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(state_t state : stateIt_t::all) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        for(reward_t reward : rewardIt_t::all) {
            RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
            basis_features.push_back(reward_feature);
            DEBUG_OUT(2,"Added " << basis_features.back()->identifier() << " to basis features");
        }
    }
}

UTree::~UTree() {}

void UTree::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward,
        const bool& new_episode
) {
    // call function of parent class
    HistoryObserver::add_action_state_reward_tripel(action,state,reward,new_episode);

    // create root node if necessary
    if(root_node==INVALID) {
        root_node = graph.addNode();
        leaf_nodes.insert(root_node);
    }

    // insert instance at root
    insert_instance(instance_data.back(),root_node);
}

void UTree::clear_data() {
    for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
        node_info_map[node].instance_vector.clear();
        node_info_map[node].scores_up_to_date = false;
        node_info_map[node].statistics_up_to_date = false;
        node_info_map[node].scores.clear();
    }

    // call function of parent class
    HistoryObserver::clear_data();
}

UTree::probability_t UTree::get_prediction(
        const instance_t * instance,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward) const {

    //--------------------------//
    // find the right leaf node //
    //--------------------------//
    node_t node = find_leaf_node(instance);

    //----------------------------------//
    // calculate transition probability //
    //----------------------------------//
    if(node==INVALID) {
        // if no leaf node could be found, return prior probability
        DEBUG_OUT(1,"Returning prior probability");
        return prior_probability(state_to, reward);
    } else {
        // transition probability p(s,r|a) equals the number of times state s
        // and reward r occurred after performing action a, divied by the total
        // number of times action a was performed.
        unsigned long transition_counter = 0;
        unsigned long action_counter = 0;
        for( const instance_t * insPtr : node_info_map[node].instance_vector ) {
            const_instanceIt_t next_instance = insPtr->const_it()+1;
            if(next_instance!=util::INVALID) {
                if(next_instance->action==action) {
                    ++action_counter;
                    if(next_instance->state==state_to && next_instance->reward==reward) {
                        ++transition_counter;
                    }
                }
            }
        }
        probability_t prob = transition_counter + prior_probability(state_to,reward)*pseudo_counts;
        prob /= action_counter + pseudo_counts;
        return prob;
    }
}

void UTree::print_tree() {

    if(root_node==INVALID) {
        DEBUG_OUT(0,"Error: Cannot print tree, root_node is INVALID");
        return;
    }

    // initialize variables
    node_vector_t * current_level = new node_vector_t();
    node_vector_t * next_level = new node_vector_t();
    size_t total_arc_counter = 0, total_node_counter = 0, level_counter = 0, leaf_counter = 0;
    current_level->push_back(root_node);

    // print
    DEBUG_OUT(0,"Printing UTree:");
    while(current_level->size()>0) { // while there are still node to print
        DEBUG_OUT(0,"    Level " << level_counter << ": " <<
                  current_level->size() << " nodes");
        for(idx_t idx=0; idx<(idx_t)current_level->size(); ++idx) { // go through all nodes of current level
            ++total_node_counter;
            node_t current_node = (*current_level)[idx];
            if(node_info_map[current_node].feature == nullptr) {
                ++leaf_counter;
            }
            if(DEBUG_LEVEL>=1) { // print information for every single node
                arc_t parent_arc = graph_t::InArcIt(graph,current_node);
                node_t parent_node = INVALID;
                if(parent_arc!=INVALID) {
                    parent_node = graph.source(parent_arc);
                }
                instance_vector_t& insVec = node_info_map[current_node].instance_vector;
                const Feature * fPtr = node_info_map[current_node].feature;
                f_ret_t pVal = node_info_map[current_node].parent_return_value;
                DEBUG_OUT(0,"        Node Id " << graph.id(current_node) << ":");
                if(parent_node!=INVALID) {
                        DEBUG_OUT(0,"            Parent Id   : " << graph.id(graph.source(graph_t::InArcIt(graph,current_node))) );
                } else {
                    if(current_node!=root_node) {
                        DEBUG_OUT(0,"Error: node has no parent but is not root_node?!");
                    } else {
                        DEBUG_OUT(0,"            Parent Id   : No parent, is root" );
                    }
                }
                if(fPtr!=nullptr) {
                        DEBUG_OUT(0,"            Feature     : " << *fPtr );
                } else {
                        DEBUG_OUT(0,"            Feature     : NULL" );
                }
                        DEBUG_OUT(0,"            Parent value: " << pVal );
                        DEBUG_OUT(0,"            Instances   : " << insVec.size() );

                if(DEBUG_LEVEL>=2) {
                    for(idx_t ins_idx=0; ins_idx<(idx_t)insVec.size(); ++ins_idx) {
                        DEBUG_OUT(0,"                          " << *(insVec[ins_idx]) );
                    }
                }
            }
            for(graph_t::OutArcIt out_arc(graph,current_node); out_arc!=INVALID; ++out_arc) { // remember children
                ++total_arc_counter;
                next_level->push_back(graph.target(out_arc));
            }
        }

        // clear current level and swap with next level
        current_level->clear();
        node_vector_t * tmp = current_level;
        current_level = next_level;
        next_level = tmp;
        ++level_counter;
    }

    // delete pointers
    delete current_level;
    delete next_level;

    // print totals
    DEBUG_OUT(0,"    Tree has a total of " <<
              total_node_counter << " nodes (" <<
              leaf_counter << " leaves) and " <<
              total_arc_counter << " arcs"
        );
}

void UTree::print_leaves() {

    if(root_node==INVALID) {
        DEBUG_OUT(0,"Error: Cannot print leaves, root_node is INVALID");
        return;
    }

    for(auto current_leaf_node : leaf_nodes) {
        DEBUG_OUT(0,"    Node Id " << graph.id(current_leaf_node) << ":");
        DEBUG_OUT(0,"        Features:");
        if(current_leaf_node==root_node) {
            DEBUG_OUT(0,"            Root node is leaf node");
        } else {
            arc_t parent_arc;
            node_t parent_node, current_node = current_leaf_node;
            do {
                // get parent node
                parent_arc = graph_t::InArcIt(graph,current_node);
                if(parent_arc==INVALID) {
                    DEBUG_OUT(0,"Error: leaf node has no parent");
                    break;
                }
                parent_node = graph.source(parent_arc);
                DEBUG_OUT(0,"            " << *(node_info_map[parent_node].feature) <<
                          " = " << node_info_map[current_node].parent_return_value );
                current_node = parent_node;
            } while(parent_node!=root_node);
        }
        instance_vector_t& insVec = node_info_map[current_leaf_node].instance_vector;
        DEBUG_OUT(0,"        Instances   : " << insVec.size() );
        if(DEBUG_LEVEL>=2) {
            for(idx_t ins_idx=0; ins_idx<(idx_t)insVec.size(); ++ins_idx) {
                DEBUG_OUT(0,"            " << *(insVec[ins_idx]) );
            }
        }
        DEBUG_OUT(0,"        Optimal Action: " << node_info_map[current_leaf_node].max_value_action );
        DEBUG_OUT(0,"        Action Value: " << node_info_map[current_leaf_node].max_state_action_value );
        if(DEBUG_LEVEL>=1) {
            DEBUG_OUT(0,"        Action Values:");
            for(auto action_values : node_info_map[current_leaf_node].state_action_values) {
                DEBUG_OUT(0,"            " << action_values.first << " --> " << action_values.second );
            }
        }
    }
}

void UTree::clear_tree() {
    // clear old graph
    graph.clear();
    leaf_nodes.clear();

    // construct empty graph, consisting of the root node only
    root_node = graph.addNode();
    leaf_nodes.insert(root_node);

    // reinsert data into root node
    if(number_of_data_points>0) {
        for(instance_t * current_episode : instance_data) {
            for(const_instanceIt_t insIt=current_episode->const_first(); insIt!=util::INVALID; ++insIt) {
                insert_instance(insIt,root_node);
            }
        }
    }
}

int UTree::get_tree_size() const {
    int size = 0;
    for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
        ++size;
    }
    return size;
}

double UTree::expand_leaf_node(const double& score_threshold) {

    DEBUG_OUT(1,"Expanding...");

    //----------------//
    // maximum values //
    //----------------//
    double max_score = -DBL_MAX;
    vector<node_t> max_nodes;
    vector<Feature*> max_features;

    //-----------------------------------------//
    // calculate scores for node-feature pairs //
    //-----------------------------------------//
    for(auto leafIt=leaf_nodes.begin(); leafIt!=leaf_nodes.end(); ++leafIt) {
        node_t node = *leafIt;
        bool up_to_date = node_info_map[node].scores_up_to_date;
        for(auto featurePtr : basis_features) {
            double score;
            if(!up_to_date) {
                score = score_leaf_node(node, featurePtr);
                node_info_map[node].scores[featurePtr] = score;
            } else {
                score = node_info_map[node].scores[featurePtr];
            }
            if(score>max_score) {
                max_score = score;
                max_nodes.assign(1,node);
                max_features.assign(1,featurePtr);
            } else if(score==max_score) {
                max_nodes.push_back(node);
                max_features.push_back(featurePtr);
            }
            DEBUG_OUT(3,"Node " << graph.id(node) << ", feature " << *featurePtr << ", score " << score);
        }
        node_info_map[node].scores_up_to_date = true;
    }

    //---------------------------//
    // check for score threshold //
    //---------------------------//
    if(max_score<score_threshold) {
        DEBUG_OUT(1,"No leaf expanded: Score threshold not reached (score: " <<
                  max_score << ", threshold: " << score_threshold << ")" );
        return max_score;
    }

    //----------------------------------------------------//
    // random-select one node-feature pair (if ambiguous) //
    //----------------------------------------------------//
    int rand_select = rand()%max_nodes.size();
    node_t max_node = max_nodes[rand_select];
    Feature * max_feature = max_features[rand_select];

    //------------------------------------------//
    // set feature to make node a non-leaf node //
    //------------------------------------------//
    node_info_map[max_node].feature = max_feature;
    node_info_map[max_node].scores.clear();
    leaf_nodes.erase(max_node);

    //-------------------------------------------------//
    // re-insert all instances to descendants only     //
    // todo: need to copy vector to prevent segfault,  //
    // don't know why (perhaps because nodes are being //
    // added?)                                         //
    //-------------------------------------------------//
    auto instance_vector_copy = node_info_map[max_node].instance_vector;
    for( auto insIt : instance_vector_copy ) {
        insert_instance(insIt,max_node,true);
        DEBUG_OUT(3,"    " << *insIt );
    }

    //---------------------------------------//
    // statistics are not up to date anymore //
    //---------------------------------------//
    for( node_t node : leaf_nodes ) {
        node_info_map[node].statistics_up_to_date = false;
    }

    return max_score;
}

double UTree::q_iteration(const double& alpha) {

    // check if state value map is complete (initialize with zero otherwise)
    for(auto current_leaf_node : leaf_nodes) {
        if(node_info_map[current_leaf_node].state_action_values.size()!=action_t::action_n) {
            DEBUG_OUT(1,"Initializing state-action values (size=" <<
                      node_info_map[current_leaf_node].state_action_values.size() << " != " <<
                      action_t::action_n << ")"
                );
            for(actionIt_t aIt=actionIt_t::first(); aIt!=util::INVALID; ++aIt) {
                node_info_map[current_leaf_node].state_action_values[aIt]=0;
            }
            DEBUG_OUT(1,"    DONE (size=" <<
                      node_info_map[current_leaf_node].state_action_values.size() << ")"
                );
        }
    }

    // run Q-iteration
    bool enought_data = false;
    for(instance_t * current_episode : instance_data) {
        const_instanceIt_t current_instance = current_episode->const_first();
        const_instanceIt_t next_instance = current_instance+1;
        if(next_instance!=util::INVALID) {
            enought_data = true;
            node_t current_leaf_node = find_leaf_node(current_instance);
            node_t next_leaf_node = find_leaf_node(next_instance);
            while(next_instance!=util::INVALID) {

                action_t current_action = current_instance->action;
                double old_Q = node_info_map[current_leaf_node].state_action_values[current_action];

                // calculate update
                double delta_Q = next_instance->reward;
                delta_Q += discount*node_info_map[next_leaf_node].max_state_action_value;
                delta_Q -= old_Q;
                delta_Q *= alpha;

                // update state-action value
                double new_Q = old_Q + delta_Q;
                node_info_map[current_leaf_node].state_action_values[current_action] = new_Q;

                // update data for next loop
                ++current_instance;
                ++next_instance;
                current_leaf_node = next_leaf_node;
                next_leaf_node = find_leaf_node(next_instance);
            }
        }
    }
    if(!enought_data) {
        DEBUG_OUT(0,"Error: Cannot perform Q-Iteration, not enough data");
        return 0;
    }

    // update state values (i.e. maximum state-action value)
    double max_value_diff = -DBL_MAX;
    for(auto current_leaf_node : leaf_nodes) {
        double max_value = -DBL_MAX;
        vector<action_t> max_value_actions;
        map<action_t,double>& value_map = node_info_map[current_leaf_node].state_action_values;
        for(auto valueIt=value_map.begin(); valueIt!=value_map.end(); ++valueIt) {
            if(valueIt->second>max_value) {
                max_value = valueIt->second;
                max_value_actions.assign(1,valueIt->first);
            } else if(valueIt->second==max_value) {
                max_value_actions.push_back(valueIt->first);
            }
        }
        double value_diff = node_info_map[current_leaf_node].max_state_action_value - max_value;
        node_info_map[current_leaf_node].max_state_action_value = max_value;
        node_info_map[current_leaf_node].max_value_action = util::random_select(max_value_actions);
        if(fabs(value_diff)>max_value_diff) {
            max_value_diff = fabs(value_diff);
        }
    }
    return max_value_diff;
}

double UTree::value_iteration() {

    //-------------------------------//
    // check if any leaf nodes exist //
    //-------------------------------//
    if(leaf_nodes.size()<1) {
        DEBUG_OUT(0,"Error: Cannot perform value iteration, not enough leaf nodes");
        return 0;
    }

    //------------------------------------//
    // check if statistics are up-to-date //
    //------------------------------------//
    for( node_t leaf : leaf_nodes ) {
        // update statistics (needs up-to-date Q-function)
        if(!node_info_map[leaf].statistics_up_to_date) {
            DEBUG_OUT(2,"Updating statistics for leaf node " << graph.id(leaf) );
            update_statistics(leaf);
        }
    }

    //-------------------------//
    // perform value iteration //
    //-------------------------//
    double max_diff = -DBL_MAX;
    for( node_t leaf : leaf_nodes ) {

        DEBUG_OUT(2,"Performing value iteration for leaf node " << graph.id(leaf) );

        // reset Q-function but remember old values (only state value is needed below)
        auto old_q_values = node_info_map[leaf].state_action_values;
        node_info_map[leaf].state_action_values.clear();

        // update Q-values
        for( auto transition : node_info_map[leaf].transition_table ) {

            // get necessary data
            action_t action = transition.first.first;                               // action to perform
            node_t state_to = transition.first.second;                              // leaf node to reach
            double prob = transition.second;                                        // probability for this transition
            double exp_rew = node_info_map[leaf].expected_reward[transition.first]; // expeced reward for this transition
            double state_value = node_info_map[state_to].max_state_action_value;    // state value of leaf node to reach

            DEBUG_OUT(3,"    Transition " << graph.id(leaf) << "," << action <<
                      " --> node:" << graph.id(state_to) <<
                      " (prob=" << prob <<
                      ",	exp_rew=" << exp_rew <<
                      ",	state_value=" << state_value
                );

            // assign new value
            double q_increment = prob * (exp_rew + discount*state_value);
            auto q_value = node_info_map[leaf].state_action_values.find(action);
            if(q_value==node_info_map[leaf].state_action_values.end()) { // not found --> insert
                node_info_map[leaf].state_action_values[action] = q_increment;
            } else { // was found --> increment
                q_value->second += q_increment;
            }
        }

        //---------------------------//
        // update maximum difference //
        //---------------------------//
        // go through new Q-values
        for( auto q_value : node_info_map[leaf].state_action_values ) {
            DEBUG_OUT(4,"    Difference for " << q_value.first );
            double diff;
            auto old_value = old_q_values.find(q_value.first);
            if(old_value==old_q_values.end()) { // not found
                DEBUG_OUT(4,"        old = NOT_FOUND");
                diff = q_value.second;
            } else {                            // was found
                DEBUG_OUT(4,"        old = " << old_value->second);
                diff = q_value.second - old_value->second;
                // erase from old
                old_q_values.erase(old_value);
            }
            DEBUG_OUT(4,"        new = " << q_value.second);
            if(fabs(diff)>max_diff) {
                max_diff = fabs(diff);
            }
        }
        // go through remaining old Q-values
        for( auto q_value : old_q_values ) {
            DEBUG_OUT(4,"    Old Q-Value not found " << q_value.first << " : " << q_value.second );
            if(fabs(q_value.second)>max_diff) {
                max_diff = fabs(q_value.second);
            }
        }

        //-------------------------------//
        // update state value and policy //
        //-------------------------------//
        update_state_value_and_policy(leaf);

    }

    return max_diff;
}

UTree::action_t UTree::get_max_value_action(const instance_t * i) {
    node_t node = find_leaf_node(i);
    if(node_info_map[node].state_action_values.size()!=action_t::action_n) {
        DEBUG_OUT(0,"Error: incomplete state action values");
    }
    action_t max_action = node_info_map[node].max_value_action;
    DEBUG_OUT(1,"Maximum value action: " << max_action << " (node " << graph.id(node) << ")" );
    return max_action;
}

void UTree::set_expansion_type(const EXPANSION_TYPE& ex) {
    expansion_type = ex;
    for(node_t node : leaf_nodes) {
        node_info_map[node].scores_up_to_date = false;
    }
}

UTree::node_t UTree::add_child(const node_t& node) {
    node_t new_child = graph.addNode();
    leaf_nodes.insert(new_child);
    graph.addArc(node,new_child);
    return new_child;
}

void UTree::insert_instance(const instance_t * i, const node_t& node, const bool& descendants_only) {

    DEBUG_OUT(4,"Inserting instance " << *i << " into UTree (node " << graph.id(node) << ")" );

    // add instance to node
    if(!descendants_only) {
        node_info_map[node].instance_vector.push_back(i);
        node_info_map[node].scores_up_to_date = false;
        node_info_map[node].statistics_up_to_date = false;
        node_info_map[node].scores.clear();
    }

    // only proceed if node has a valid feature
    if(node_info_map[node].feature != nullptr) {

        // return value for child node
        f_ret_t ret = node_info_map[node].feature->evaluate(i);

        // find matching child node
        for(graph_t::OutArcIt out_arc(graph,node); out_arc!=INVALID; ++out_arc) {
            node_t out_node = graph.target(out_arc);
            if(node_info_map[out_node].parent_return_value==ret) {
                // insert instance into child node and return
                insert_instance(i,out_node);
                return;
            }
        }

        // no matching child node found --> create new one
        node_t new_child = add_child(node);
        node_info_map[new_child].parent_return_value = ret;
        insert_instance(i,new_child); // will only add i and not descend further
    }

}

double UTree::score_leaf_node(const node_t leaf_node, const Feature* feature) const {

    double default_score = 0;

    //----------------------------------//
    // check if it's really a leaf node //
    //----------------------------------//
    if(graph_t::OutArcIt(graph,leaf_node)!=INVALID) {
        DEBUG_OUT(0,"Error: Scoring non-leaf node");
        return default_score;
    }

    DEBUG_OUT(3,"Scoring leaf node " << graph.id(leaf_node) << " with feature " << *feature);

    //--------------------------------------------------------//
    // collect samples from instances, discriminating between //
    // different feature return values and different actions  //
    //--------------------------------------------------------//
    const instance_vector_t& instance_vector = node_info_map[leaf_node].instance_vector;
    map< pair<f_ret_t,action_t>, vector<double                 > > utility_samples;      // for UTILITY_EXPANSION
    map< pair<f_ret_t,action_t>, vector<pair<state_t,reward_t> > > state_reward_samples; // for STATE_REWARD_EXPANSION
    set<f_ret_t> feature_return_values; // corresponds to different child nodes

    // iterate through instances
    for(const instance_t * instance : instance_vector) {

        // next instance (that provides the actual data)
        const_instanceIt_t next_instance = instance->const_it()+1;
        if(next_instance==util::INVALID) {
            continue;
        }

        // construct state-action pair
        f_ret_t f_ret = feature->evaluate(instance); // feature return value defining the potential new leaf node
        action_t action = next_instance->action;     // action that was performed from current node
        pair<f_ret_t,action_t> node_action_pair = make_pair(f_ret,action);

        // remember state/leaf
        feature_return_values.insert(f_ret);

        // update samples

        switch(expansion_type) {
        case UTILITY_EXPANSION:
            // A sample consists of the reward actually received after
            // performing the action plus the discounted state value of the leaf
            // node that was actually reached.
        {
            node_t next_state = find_leaf_node(next_instance);
            double util = next_instance->reward + discount*node_info_map[next_state].max_state_action_value;
            utility_samples[node_action_pair].push_back(util);
            DEBUG_OUT(4,"    Adding utility of " << util << " for f_ret=" << f_ret << "	" << action);
        }
        break;
        case STATE_REWARD_EXPANSION:
            // A sample consists of the state (not leaf node) that was actually
            // reached after performing the action and the reward that was
            // actually received.
            if(next_instance!=util::INVALID) {
                state_reward_samples[node_action_pair].push_back(make_pair(next_instance->state, next_instance->reward));
            }
            break;
        default:
            DEBUG_DEAD_LINE;
        }
    }

    //--------------------------------------------------//
    // check if number of samples equals two (currently //
    // using binary features only!)                     //
    //--------------------------------------------------//
    int feature_return_n = feature_return_values.size();
    if(feature_return_n>2) {
        DEBUG_OUT(0,"Error: Got " << feature_return_n << " different feature values. Expecting only two.");
        return default_score;
    } else if(feature_return_n<2) { // nothing to compare
        DEBUG_OUT(2,"Not enough samples, returning default score");
        return default_score;
    }

    //---------------------//
    // compute score value //
    //---------------------//
    // The score is computed as the sum over the sub-scores for all actions. The
    // sub-scores are computed by comparing the samples for the two different
    // feature return values (i.e. potential leaf nodes) using the
    // Kolmogorov-Smirnov test for the real valued utility (state value)
    // distributions and the Chi-Square test for the discrete state-reward
    // distributions.
    f_ret_t f_val_1 = *(feature_return_values.begin());
    f_ret_t f_val_2 = *(++(feature_return_values.begin()));
    double score = 0;
    for(actionIt_t action=actionIt_t::first(); action!=util::INVALID; ++action) {
        switch(expansion_type) {
        case UTILITY_EXPANSION:
        {
            vector<double>& sample_1 = utility_samples[make_pair(f_val_1,action)];
            vector<double>& sample_2 = utility_samples[make_pair(f_val_2,action)];
            double tmp_score = sample_size_factor( sample_1.size(), sample_2.size() );
            tmp_score *= KolmogorovSmirnovTest::k_s_test(sample_1, sample_2, false);
            score += tmp_score;
            break;
        }
        case STATE_REWARD_EXPANSION:
        {
            vector<pair<state_t,reward_t> >& sample_1 = state_reward_samples[make_pair(f_val_1,action)];
            vector<pair<state_t,reward_t> >& sample_2 = state_reward_samples[make_pair(f_val_2,action)];
            double tmp_score = sample_size_factor( sample_1.size(), sample_2.size() );
            tmp_score *= ChiSquareTest::chi_square_statistic<pair<state_t,reward_t> >(sample_1, sample_2, false);
            score += tmp_score;
            break;
        }
        default:
            DEBUG_DEAD_LINE;
        }
    }

    return score;

}

double UTree::sample_size_factor(const int& n1, const int& n2) const {
    double factor = 1;
    if(n1<10) factor = 0;
    if(n2<10) factor = 0;
    DEBUG_OUT(2,"    Sample size factor: n1=" << n1 << ", n2=" << n2 << ", factor=" << factor);
    return factor;
}

UTree::node_t UTree::find_leaf_node(const instance_t *i) const {

    node_t current_node = root_node;

    if(current_node==INVALID) {
        DEBUG_OUT(0,"Error: root node is INVALID");
        return current_node;
    }

    while(node_info_map[current_node].feature != nullptr) {

        // return value for child node
        f_ret_t ret = node_info_map[current_node].feature->evaluate(i);

        // find matching child node
        bool child_found = false;
        for(graph_t::OutArcIt out_arc(graph,current_node); out_arc!=INVALID; ++out_arc) {
            node_t out_node = graph.target(out_arc);
            if(node_info_map[out_node].parent_return_value==ret) {
                current_node = out_node;
                child_found = true;
                break;
            }
        }

        // no matching child node found
        if(!child_found) {
            DEBUG_OUT(0,"Error: Unable to descend to leaf because no matching child node was found");
            return current_node;
        }
    }

    // return leaf node
    return current_node;
}

UTree::probability_t UTree::prior_probability(const state_t&, const reward_t& r) const {
    if(r==reward_t::min_reward) {
        return 1./(state_t::state_n);
    } else {
        return 0;
    }
}

void UTree::update_statistics(const node_t& leaf_node) {

    DEBUG_OUT(3,"Update statistics for node " << graph.id(leaf_node) );

    //-----------------------------------//
    // update maximum Q-value and policy //
    //-----------------------------------//
    update_state_value_and_policy(leaf_node);

    //----------------//
    // collect counts //
    //----------------//

    // number of times an action was performed
    map< action_t, unsigned long int > action_counts;
    // number of times a leaf node was reached with a specific action
    map< pair<action_t,node_t>, unsigned long int > transition_counts;
    // sum of rewards received for a specific transition (state and action specific)
    map< pair<action_t,node_t>, double > reward_sums;

    // go through instances
    for( const instance_t * ins : node_info_map[leaf_node].instance_vector ) {
        const instance_t * next_instance = ins->const_it()+1;
        action_t action = next_instance->action;
        reward_t reward = next_instance->reward;
        node_t next_state = find_leaf_node(next_instance);
        action_counts[action] += 1;
        transition_counts[make_pair(action,next_state)] += 1;
        reward_sums[make_pair(action,next_state)] += reward;
    }

    //------------------------------------------------//
    // calculate transition probabilities from counts //
    // (store only non-zero probabilities)            //
    //------------------------------------------------//
    // Transition probabilite p(s|a) equals the number of times state s (leaf
    // node) was reached with action a, divided by the total number of times
    // action a was carried out.
    node_info_map[leaf_node].transition_table.clear();
    for( auto current_transition_count : transition_counts ) {
        double prob = current_transition_count.second;
        prob /= action_counts[current_transition_count.first.first];
        node_info_map[leaf_node].transition_table[current_transition_count.first] = prob;
        DEBUG_OUT(4,"    Transition " << current_transition_count.first.first << " --> " <<
                  graph.id(current_transition_count.first.second) << "	prob=" << prob
        );
    }

    //----------------------------//
    // calculate expected rewards //
    //----------------------------//
    // The expected reward r(s,a) equals the sum over all rewards received for a
    // transition to state (leaf node) s after performing action a, divided by
    // the total number of times the transition occurred.
    node_info_map[leaf_node].expected_reward.clear();
    for( auto current_reward_sum : reward_sums ) {
        double exp_rew = current_reward_sum.second;
        exp_rew /= transition_counts[current_reward_sum.first];
        node_info_map[leaf_node].expected_reward[current_reward_sum.first] = exp_rew;
        DEBUG_OUT(4,"    Transition " << current_reward_sum.first.first << " --> " <<
                  graph.id(current_reward_sum.first.second) << "	rew=" << exp_rew
            );
    }

    // mark as up-to-date
    node_info_map[leaf_node].statistics_up_to_date = true;
}

void UTree::update_state_value_and_policy(const node_t& leaf_node) {

    DEBUG_OUT(3,"Update state value and policy for node " << graph.id(leaf_node) );

    double max_q_value = -DBL_MAX;
    vector<action_t> max_value_action_vector;
    for(auto current_q_value : node_info_map[leaf_node].state_action_values) {
        if(current_q_value.second > max_q_value) {
            max_q_value = current_q_value.second;
            max_value_action_vector.assign(1,current_q_value.first);
        } else if(current_q_value.second==max_q_value) {
            max_value_action_vector.push_back(current_q_value.first);
        }
    }

    // set state value and optimal action
    if(max_value_action_vector.size()==0) { // state-action values were empty
        DEBUG_OUT(4,"    Empty Q-values");
        node_info_map[leaf_node].max_state_action_value = 0;
        node_info_map[leaf_node].max_value_action = action_t::STAY;
    } else {
        action_t optimal_action = util::random_select(max_value_action_vector);
        DEBUG_OUT(4,"    state-value=" << max_q_value << "	action=" << optimal_action);
        node_info_map[leaf_node].max_state_action_value = max_q_value;
        node_info_map[leaf_node].max_value_action = optimal_action;
    }

}
