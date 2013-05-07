#include "UTree.h"
#include "util.h"
#include "util/KolmogorovSmirnovTest.h"
#include "util/ChiSquareTest.h"

#include <map>
#include <float.h>

#define DEBUG_STRING "UTree: "
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::map;
using std::cout;
using std::endl;

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
    if(instance_data==nullptr) {
        instance_data = instance_t::create(action,state,reward);
    } else {
        instance_data = instance_data->append_instance(action,state,reward);
    }
    DEBUG_OUT(1, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
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
    return 0; // todo
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

void UTree::insert_instance(const instance_t * i, const node_t& node) {
    // add instance to node
    node_info_map[node].instance_vector.push_back(i);

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
        node_t new_child = graph.addNode();
        leaf_nodes.push_back(new_child);
        graph.addArc(node,new_child);
        node_info_map[new_child].parent_return_value = ret;
        insert_instance(i,new_child); // will only add i and not descend further
    }
}

double UTree::score_reward(const node_t leaf_node, const Feature* feature) {
    // check if it's really a leaf node
    if(graph_t::OutArcIt(graph,leaf_node)!=INVALID) {
        DEBUG_OUT(0,"Error: Scoring non-leaf node");
        return DBL_MAX;
    }

    // collect samples from instances
    instance_vector_t& instance_vector = node_info_map[leaf_node].instance_vector;
    map< f_ret_t, vector<double> > samples;
    for(unsigned int idx=0; idx<instance_vector.size(); ++idx) {
        const_instanceIt_t insIt = instance_vector[idx]->const_it()+1;
        if(insIt==util::INVALID) { // instance has no successor
            continue;
        } else {
            f_ret_t f_ret = feature->evaluate(instance_vector[idx]);
            samples[f_ret].push_back(insIt->reward);
        }
    }

    // compute score value
    if(samples.size()>2) {
        DEBUG_OUT(0,"Error: Got " << samples.size() << " different feature values. Expecting only two.");
        return DBL_MAX;
    } else if(samples.size()<2) { // nothing to compare
        return DBL_MAX;
    } else {
        auto sIt = samples.begin();
        vector<double>& sample_1 = sIt->second;
        ++sIt;
        vector<double>& sample_2 = sIt->second;
        switch(test_type) {
        case KOLMOGOROV_SMIRNOV:
            return KolmogorovSmirnovTest::k_s_test(sample_1, sample_2, false);
            break;
        case CHI_SQUARE:
            return ChiSquareTest::chi_square_statistic(sample_1, sample_2, false);
            break;
        default:
            DEBUG_OUT(0,"Error: Unknown test type");
            return DBL_MAX;
        }
    }
}
