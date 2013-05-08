#include "UTree.h"
#include "util.h"
#include "util/KolmogorovSmirnovTest.h"
#include "util/ChiSquareTest.h"

#include <map>
#include <queue>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#define DEBUG_STRING "UTree: "
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::map;
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
    parent_return_value(r)
{}

UTree::UTree():
        k(Data::k),
        instance_data(nullptr),
        root_node(INVALID),
        node_info_map(graph)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(actionIt_t action=actionIt_t::first(); action!=util::INVALID; ++action) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(stateIt_t state=stateIt_t::first(); state!=util::INVALID; ++state) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        for(rewardIt_t reward=rewardIt_t::first(); reward!=util::INVALID; ++reward) {
            RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
            basis_features.push_back(reward_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
    }
}

UTree::~UTree() {
    delete instance_data;
}

void UTree::add_action_state_reward_tripel(
        const action_t& action,
        const state_t& state,
        const reward_t& reward
) {
    // create instance data or append to existing
    if(instance_data==nullptr) {
        instance_data = instance_t::create(action,state,reward);
    } else {
        instance_data = instance_data->append_instance(action,state,reward);
    }
    DEBUG_OUT(1, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );

    // create root node if necessary
    if(root_node==INVALID) {
        root_node = graph.addNode();
        leaf_nodes.insert(root_node);
    }

    // insert instance at root
    insert_instance(instance_data,root_node);
}

void UTree::clear_data() {
    for(graph_t::NodeIt node(graph); node!=INVALID; ++node) {
        node_info_map[node].instance_vector.clear();
    }
    delete instance_data;
    instance_data = nullptr;
}

UTree::probability_t UTree::get_prediction(
        const instance_t * instance,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward) const {

    // find the right leaf node
    node_t node = find_leaf_node(instance);

    // if no leaf node could be found, return prior probability
    if(node==INVALID) {
        return prior_probability(state_to, reward);
    } else {
        unsigned long counter = 0;
        for(auto insIt=node_info_map[node].instance_vector.begin();
            insIt!=node_info_map[node].instance_vector.end();
            ++insIt
            ) {
            if((*insIt)->action==action && (*insIt)->state==state_to && (*insIt)->reward==reward) {
                ++counter;
            }
        }
        probability_t prob = counter + prior_probability(state_to,reward)*pseudo_counts;
        prob /= pseudo_counts + node_info_map[node].instance_vector.size();
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
    size_t total_arc_counter = 0, total_node_counter = 0, level_counter = 0;
    current_level->push_back(root_node);

    // print
    DEBUG_OUT(0,"Printing UTree:");
    while(current_level->size()>0) { // while there are still node to print
        DEBUG_OUT(0,"    Level " << level_counter << ": " <<
                  current_level->size() << " nodes");
        for(idx_t idx=0; idx<(idx_t)current_level->size(); ++idx) { // go through all nodes of current level
            ++total_node_counter;
            node_t current_node = (*current_level)[idx];
            if(DEBUG_LEVEL>=2) { // print information for every single node
                node_t parent_node = graph.source(graph_t::InArcIt(graph,current_node));
                instance_vector_t& insVec = node_info_map[current_node].instance_vector;
                const Feature * fPtr = node_info_map[current_node].feature;
                f_ret_t pVal = node_info_map[current_node].parent_return_value;
                DEBUG_OUT(0,"    Node Id " << graph.id(current_node) << ":");
                if(parent_node!=INVALID) {
                    DEBUG_OUT(0,"        Parent Id   : " << graph.id(graph.source(graph_t::InArcIt(graph,current_node))) );
                } else {
                    if(current_node!=root_node) {
                        DEBUG_OUT(0,"Error: node has no parent but is not root_node?!");
                    } else {
                        DEBUG_OUT(0,"        Parent Id   : ROOT" );
                    }
                }
                for(idx_t ins_idx=0; ins_idx<insVec.size(); ++ins_idx) {
                    DEBUG_OUT(0,"        Instance    : " << *(insVec[ins_idx]) );
                }
                if(fPtr!=nullptr) {
                    DEBUG_OUT(0,"        Feature     : " << *fPtr );
                } else {
                    DEBUG_OUT(0,"        Feature     : NULL" );
                }
                DEBUG_OUT(0,"        Parent value: " << pVal );
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
    DEBUG_OUT(0,"    Tree has a total of " << total_node_counter << " nodes and " << total_arc_counter << " arcs");
}

int UTree::expand_leaf_nodes(const int& n) {

    // priority queue storing the leaves
    priority_queue<tuple<double,node_t,Feature*> > leaf_queue;

    // calculate scores for node-feature pairs and insert into queue
    for(auto leafIt=leaf_nodes.begin(); leafIt!=leaf_nodes.end(); ++leafIt) {
        for(auto featureIt=basis_features.begin(); featureIt!=basis_features.end(); ++featureIt) {
            node_t node = *leafIt;
            DEBUG_OUT(0,"Calculate score for feature " << **featureIt);
            double score = score_leaf_node(node, *featureIt);
            score *= sample_size_factor(node_info_map[node].instance_vector.size());
            score *= 0.5*(1+drand48()*1e-10); // to disambiguate identical scores, 0.5 to prevent overflow
            leaf_queue.push(make_tuple(-score, node, *featureIt));
            DEBUG_OUT(0,"Score " << score);
        }
    }

    // print and evaluate all basis features with current instance
    for(auto featureIt=basis_features.begin(); featureIt!=basis_features.end(); ++featureIt) {
        DEBUG_OUT(0,"Feature " << **featureIt);
        DEBUG_OUT(0,"Value " << (*featureIt)->evaluate(instance_data) );
    }

    // expand the n highest scored leaf nodes
    int n_ret;
    for(n_ret=0; n_ret<n; ++n_ret) {
        // break if not enough leaf nodes
        if(leaf_queue.size()==0) {
            break;
        }

        // get data
        auto top_leaf = leaf_queue.top();
        node_t node = get<1>(top_leaf);
        Feature * feature = get<2>(top_leaf);

        DEBUG_OUT(0,"Node Id: " << graph.id(node) );
        DEBUG_OUT(0,"Feature: " << *feature );

        // set feature to make node a non-leaf node
        node_info_map[node].feature = feature;

        // print all instances
        for(auto insIt=node_info_map[node].instance_vector.begin();
            insIt!=node_info_map[node].instance_vector.end();
            ++insIt
            ) {
            DEBUG_OUT(0,"Going to insert " << **insIt );
        }

        // re-insert all instances to descendants only
        for(auto insIt=node_info_map[node].instance_vector.begin();
            insIt!=node_info_map[node].instance_vector.end();
            ++insIt
            ) {
            insert_instance(*insIt,node,true);
        }

        // pop processed leaf node
        leaf_queue.pop();
        leaf_nodes.erase(node);
    }

    return n_ret;
}

UTree::node_t UTree::add_child(const node_t& node) {
    node_t new_child = graph.addNode();
    leaf_nodes.insert(new_child);
    graph.addArc(node,new_child);
    return new_child;
}

void UTree::insert_instance(const instance_t * i, const node_t& node, const bool& descendants_only) {
    // add instance to node
    if(!descendants_only) {
        node_info_map[node].instance_vector.push_back(i);
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

    // check if tests can be performed
    if(test_type==KOLMOGOROV_SMIRNOV && score_type!=SCORE_BY_REWARDS) {
        DEBUG_OUT(0,"Error: Can calculate K-S statistic only for rewards");
        return DBL_MAX;
    }

    // check if it's really a leaf node
    if(graph_t::OutArcIt(graph,leaf_node)!=INVALID) {
        DEBUG_OUT(0,"Error: Scoring non-leaf node");
        return DBL_MAX;
    }

    // collect samples from instances
    const instance_vector_t& instance_vector = node_info_map[leaf_node].instance_vector;
    map< f_ret_t, vector<double> >                   reward_samples;        // for SCORE_BY_REWARDS
    map< f_ret_t, vector<action_t> >                 action_samples;        // for SCORE_BY_ACTIONS
    map< f_ret_t, vector<pair<action_t,reward_t> > > action_reward_samples; // for SCORE_BY_BOTH
    for(unsigned int idx=0; idx<instance_vector.size(); ++idx) {
        const_instanceIt_t insIt = instance_vector[idx]->const_it()+1;
        if(insIt==util::INVALID) { // instance has no successor
            continue;
        } else {
            f_ret_t f_ret = feature->evaluate(instance_vector[idx]);
            switch(score_type) {
            case SCORE_BY_REWARDS:
                reward_samples[f_ret].push_back(insIt->reward);
                break;
            case SCORE_BY_ACTIONS:
                action_samples[f_ret].push_back(insIt->action);
                break;
            case SCORE_BY_BOTH:
                action_reward_samples[f_ret].push_back(make_pair(insIt->action, insIt->reward));
                break;
            default:
                DEBUG_DEAD_LINE;
            }
        }
    }

    // check for number of samples
    int samples_size = 0;
    switch(score_type) {
    case SCORE_BY_REWARDS:
        samples_size = reward_samples.size();
        break;
    case SCORE_BY_ACTIONS:
        samples_size = action_samples.size();
        break;
    case SCORE_BY_BOTH:
        samples_size = action_reward_samples.size();
        break;
    default:
        DEBUG_DEAD_LINE;
    }
    if(samples_size>2) {
        DEBUG_OUT(0,"Error: Got " << samples_size << " different feature values. Expecting only two.");
        return DBL_MAX;
    } else if(samples_size<2) { // nothing to compare
        return DBL_MAX;
    }

    // compute score value
    switch(test_type) {
    case KOLMOGOROV_SMIRNOV:
        switch(score_type) {
        case SCORE_BY_REWARDS: {
            vector<double>& sample_1 = reward_samples.begin()->second;
            vector<double>& sample_2 = (++(reward_samples.begin()))->second;
            return KolmogorovSmirnovTest::k_s_test(sample_1, sample_2, false);
        }
        default:
            DEBUG_DEAD_LINE;
        }
    case CHI_SQUARE:
        switch(score_type) {
        case SCORE_BY_REWARDS: {
            vector<double>& sample_1 = reward_samples.begin()->second;
            vector<double>& sample_2 = (++(reward_samples.begin()))->second;
            return ChiSquareTest::chi_square_statistic<double>(sample_1, sample_2, false);
        }
        case SCORE_BY_ACTIONS: {
            vector<action_t>& sample_1 = action_samples.begin()->second;
            vector<action_t>& sample_2 = (++(action_samples.begin()))->second;
            return ChiSquareTest::chi_square_statistic<action_t>(sample_1, sample_2, false);
        }
        case SCORE_BY_BOTH: {
            vector<pair<action_t,reward_t> >& sample_1 = action_reward_samples.begin()->second;
            vector<pair<action_t,reward_t> >& sample_2 = (++(action_reward_samples.begin()))->second;
            return ChiSquareTest::chi_square_statistic<pair<action_t,reward_t> >(sample_1, sample_2, false);
        }
        default:
            DEBUG_DEAD_LINE;
        }
    default:
        DEBUG_OUT(0,"Error: Unknown test type");
        return DBL_MAX;
    }
}

double UTree::sample_size_factor(const int&) const {
    return 1;
}

UTree::node_t UTree::find_leaf_node(const instance_t *i) const {

    node_t current_node = root_node;

    while(current_node!=INVALID && node_info_map[current_node].feature != nullptr) {

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
            current_node = INVALID;
        }
    }

    // return leaf node or INVALID (if no corresponding leaf node exists)
    return current_node;
}

UTree::probability_t UTree::prior_probability(const state_t&, const reward_t&) const {
    int state_n = state_t::max_state - state_t::min_state +1;
    int reward_n = floor((reward_t::max_reward - reward_t::min_reward)/reward_t::reward_increment) + 1;
    return 1./(state_n*reward_n);
}
