
#include "UTree.h"

#include "util.h"

#include <list>
#include <tuple>

#define DEBUG_STRING "UTree: "
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::list;
using std::tuple;
using std::make_tuple;
using std::get;
using std::pair;
using std::make_pair;
using std::set;
using std::cout;
using std::endl;

using util::INVALID;

UTree::UTree():
        k(Data::k),
        old_active_features_size(0),
        instance_data(nullptr),
        compound_features_sorted(false)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>=-k; --k_idx) {
        // actions
        for(actionIt_t action=actionIt_t::first(); action!=INVALID; ++action) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(stateIt_t state=stateIt_t::first(); state!=INVALID; ++state) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        for(rewardIt_t reward=rewardIt_t::first(); reward!=INVALID; ++reward) {
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

UTree::probability_t UTree::get_prediction(
        const instance_t * instance,
        const action_t& action,
        const state_t& state_to,
        const reward_t& reward) const {
    return 0;
}

void UTree::construct_compound_features(const int& n) {

    DEBUG_OUT(1, "Constructing compound features...");

    compound_features.clear();

    if(n<0) {
        DEBUG_OUT(0, "    Multiplicity must be non-negative");
        return;
    }
    if(n==0) {
        DEBUG_OUT(1, "    Multiplicity is zero, no features constructed");
        return;
    }

    // Temporally use list structure to make sorting and erasing more efficient
    list<AndFeature> compound_feature_list;

    // Add active features
    for(uint f_idx = 0; f_idx < active_features.size(); ++f_idx) {
        DEBUG_OUT(2,"Including " << active_features[f_idx].identifier() << " in base set");
        compound_feature_list.push_back(AndFeature(active_features[f_idx]));
    }

    // Add NullFeature if active features were empty
    if(compound_feature_list.size()==0) {
        compound_feature_list.push_back(AndFeature());
        DEBUG_OUT(2,"Used " <<  compound_feature_list.front().identifier() << " as base");
    }

    // Replace current features by those that can
    // be reached by combining one of the current
    // features with n basis features.
    for(int order=1; order<=n; ++order) {
        int counter = compound_feature_list.size();
        list<AndFeature>::iterator cf_it = compound_feature_list.begin();
        while( counter>0 ) {
            for(uint bf_idx = 0; bf_idx < basis_features.size(); ++bf_idx) {
                DEBUG_OUT(2,"Compound: " << cf_it->identifier() << ", Basis(" << bf_idx << "): " << basis_features[bf_idx]->identifier() )
                AndFeature and_feature(*basis_features[bf_idx],*cf_it);
                DEBUG_OUT(2,"    --> " << and_feature.identifier() );
                // make sure the basis feature is not already
                // part of the compound feature (duplicates
                // are removed below)
                if(and_feature!=*cf_it) {
                    compound_feature_list.push_back(and_feature);
                    DEBUG_OUT(2,"    accepted");
                } else {
                    DEBUG_OUT(2,"    rejected");
                }
            }
            ++cf_it;
            --counter;
            compound_feature_list.pop_front();
        }
    }

    compound_feature_list.sort();
    list<AndFeature>::iterator cf_it_1 = compound_feature_list.begin();
    list<AndFeature>::iterator cf_it_2 = compound_feature_list.begin();
    ++cf_it_2;
    while( cf_it_1!=compound_feature_list.end() ) {
        if(cf_it_2!=compound_feature_list.end() && *cf_it_2==*cf_it_1) {
            DEBUG_OUT(2, "    Remove " << cf_it_2->identifier() );
            cf_it_2 = compound_feature_list.erase(cf_it_2);
        } else {
            DEBUG_OUT(2, "    Keep   " << cf_it_1->identifier() );
            compound_features.push_back(*cf_it_1);
            ++cf_it_1;
            ++cf_it_2;
        }
    }

    DEBUG_OUT(1, "    Constructed " << compound_features.size() << " features");

    compound_feature_scores.assign(compound_features.size(),0.0);

    DEBUG_OUT(1, "DONE");
}
