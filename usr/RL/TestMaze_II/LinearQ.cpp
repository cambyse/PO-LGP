#include "LinearQ.h"
#include "util.h"

#include <queue>
#include <list>
#include <utility> // for std::pair
#include <tuple>
#include <float.h> // for DBL_MAX

#include <armadillo>

#define DEBUG_STRING "LinearQ: "
#define DEBUG_LEVEL 1
#include "debug.h"

using std::vector;
using std::list;
using std::cout;
using std::endl;
using std::pair;
using std::make_pair;
using std::tuple;
using std::make_tuple;
using std::get;
using std::priority_queue;

using arma::mat;
using arma::vec;
using arma::randu;
using arma::mat44;

using util::INVALID;

LinearQ::LinearQ(const double& d):
        k(Data::k),
        instance_data(nullptr),
        discount(d)
{

    //----------------------------------------//
    // Constructing basis indicator features  //
    //----------------------------------------//

    // delayed action, state, and reward features
    for(int k_idx = 0; k_idx>-k; --k_idx) {
        // actions
        for(action_t action : actionIt_t::all) {
            ActionFeature * action_feature = ActionFeature::create(action,k_idx);
            basis_features.push_back(action_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // states
        for(state_t state : stateIt_t::all) {
            StateFeature * state_feature = StateFeature::create(state,k_idx);
            basis_features.push_back(state_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
        // reward
        for(reward_t reward : rewardIt_t::all) {
            RewardFeature * reward_feature = RewardFeature::create(reward,k_idx);
            basis_features.push_back(reward_feature);
            DEBUG_OUT(1,"Added " << basis_features.back()->identifier() << " to basis features");
        }
    }
}

LinearQ::~LinearQ() {
    delete instance_data;
}

void LinearQ::add_action_state_reward_tripel(
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
    DEBUG_OUT(2, "added (action,state,reward) = (" << action << "," << state << "," << reward << ")" );
}

double LinearQ::optimize(const double& reg) {

    // get dimension
    idx_t feature_n = active_features.size();

    // declare variables
    mat L(feature_n,feature_n);
    vec r(feature_n);
    vec w;

    // fill variables
    //todo

    // solve
    bool solved = solve(w, L, r);
    if(!solved) {
        DEBUG_OUT(0,"Error: Could not solve equations");
        return 0;
    }

    w.print();

    return 0;
}

void LinearQ::clear_data() {
    delete instance_data;
    instance_data = nullptr;
}

action_t LinearQ::get_max_value_action(const instance_t * i) {
    return action_t(); // todo
}

void LinearQ::construct_candidate_features(const int& n) {

    DEBUG_OUT(1, "Constructing candidate features...");

    candidate_features.clear();

    if(n<0) {
        DEBUG_OUT(0, "    Multiplicity must be non-negative");
        return;
    }
    if(n==0) {
        DEBUG_OUT(1, "    Multiplicity is zero, no features constructed");
        return;
    }

    // Temporally use list structure to make sorting and erasing more efficient
    list<AndFeature> candidate_feature_list;

    // Add active features
    for(uint f_idx = 0; f_idx < active_features.size(); ++f_idx) {
        DEBUG_OUT(2,"Including " << active_features[f_idx].identifier() << " in base set");
        candidate_feature_list.push_back(AndFeature(active_features[f_idx]));
    }

    // Add NullFeature if active features were empty
    if(candidate_feature_list.size()==0) {
        candidate_feature_list.push_back(AndFeature());
        DEBUG_OUT(2,"Used " <<  candidate_feature_list.front().identifier() << " as base");
    }

    // Replace current features by those that can
    // be reached by combining one of the current
    // features with n basis features.
    for(int order=1; order<=n; ++order) {
        int counter = candidate_feature_list.size();
        list<AndFeature>::iterator cf_it = candidate_feature_list.begin();
        while( counter>0 ) {
            for(uint bf_idx = 0; bf_idx < basis_features.size(); ++bf_idx) {
                DEBUG_OUT(2,"Candidate: " << cf_it->identifier() << ", Basis(" << bf_idx << "): " << basis_features[bf_idx]->identifier() )
                AndFeature and_feature(*basis_features[bf_idx],*cf_it);
                DEBUG_OUT(2,"    --> " << and_feature.identifier() );
                // make sure the basis feature is not already
                // part of the candidate feature (duplicates
                // are removed below)
                if(and_feature!=*cf_it) {
                    candidate_feature_list.push_back(and_feature);
                    DEBUG_OUT(2,"    accepted");
                } else {
                    DEBUG_OUT(2,"    rejected");
                }
            }
            ++cf_it;
            --counter;
            candidate_feature_list.pop_front();
        }
    }

    candidate_feature_list.sort();
    list<AndFeature>::iterator cf_it_1 = candidate_feature_list.begin();
    list<AndFeature>::iterator cf_it_2 = candidate_feature_list.begin();
    ++cf_it_2;
    while( cf_it_1!=candidate_feature_list.end() ) {
        if(cf_it_2!=candidate_feature_list.end() && *cf_it_2==*cf_it_1) {
            DEBUG_OUT(2, "    Remove " << cf_it_2->identifier() );
            cf_it_2 = candidate_feature_list.erase(cf_it_2);
        } else {
            DEBUG_OUT(2, "    Keep   " << cf_it_1->identifier() );
            candidate_features.push_back(*cf_it_1);
            ++cf_it_1;
            ++cf_it_2;
        }
    }

    DEBUG_OUT(1, "    Constructed " << candidate_features.size() << " features");

    DEBUG_OUT(1, "DONE");
}

LinearQ::probability_t LinearQ::prior_probability(const state_t&, const reward_t& r) const {
    if(r==reward_t::min_reward) {
        return 1./(state_t::state_n);
    } else {
        return 0;
    }
}
