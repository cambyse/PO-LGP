#include "ConjunctiveAdjacency.h"

#include "../Representation/Feature.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

using std::set;
using std::dynamic_pointer_cast;
using std::make_pair;

ConjunctiveAdjacency::f_set_t ConjunctiveAdjacency::expand_with_basis_features(
    const f_set_t& current_features,
    const f_set_t& basis_features) const {
    DEBUG_OUT(2,"Expanding with basis features");
    f_set_t extension_feature_set;
    f_ptr_t new_feature;
    for(f_ptr_t basis_f : basis_features) {
        new_feature = f_ptr_t(new AndFeature(basis_f));
        extension_feature_set.insert(new_feature);
        DEBUG_OUT(3,"    Added basis feature: " << *new_feature);
    }
    for(f_ptr_t feature_1 : current_features) {
        // combine with all basis features
        DEBUG_OUT(3,"Extending " << *feature_1);
        for(f_ptr_t basis_f : basis_features) {
            new_feature = f_ptr_t(new AndFeature(feature_1,basis_f));
            extension_feature_set.insert(new_feature);
            DEBUG_OUT(3,"    Built feature: " << *new_feature);
        }
        // combine with other features
        if(combine_features) {
            for(f_ptr_t feature_2 : current_features) {
                new_feature = f_ptr_t(new AndFeature(feature_1,feature_2));
                extension_feature_set.insert(new_feature);
                DEBUG_OUT(3,"    Built feature: " << *new_feature);
            }
        }
    }
    return extension_feature_set;
}

ConjunctiveAdjacency::f_set_t ConjunctiveAdjacency::operator()(
    const f_set_t& current_features
    ) const {

    // find delays for current action, observation, and reward features
    DEBUG_OUT(2,"Finding delays");
    set<int> action_delays, observation_delays, reward_delays;
    for(f_ptr_t f : current_features) {
        add_delay(f,action_delays,observation_delays,reward_delays);
    }
    // extend delays by horizon extension
    DEBUG_OUT(2,"Extending delays");
    set<int> new_action_delays, new_observation_delays, new_reward_delays;
    for(auto delays : {make_pair(&action_delays,&new_action_delays),
                make_pair(&observation_delays,&new_observation_delays),
                make_pair(&reward_delays,&new_reward_delays)}) {
        for(int old_delay : *(delays.first)) {
            for(int delay_extension = -horizon_extension; delay_extension<=horizon_extension; ++delay_extension) {
                int new_delay = old_delay+delay_extension;
                if((new_delay<=max_horizon &&              // respect max_horizon
                    new_delay>=min_horizon) ||             // respect min_horizon
                   (allow_zero_delay && new_delay==0)) {   // allow zero delay?
                    delays.second->insert(new_delay);
                }
            }
        }
    }
    // construct basis features (always include zero order)
    DEBUG_OUT(2,"Constructing basis features");
    f_set_t basis_features;
    f_ptr_t new_feature;
    for(action_ptr_t action : action_space) {
        new_feature = ActionFeature::create(action,0);
        basis_features.insert(new_feature);
        DEBUG_OUT(3,"Added " << *new_feature);
        for(int delay : new_action_delays) {
            new_feature = ActionFeature::create(action,delay);
            basis_features.insert(new_feature);
            DEBUG_OUT(3,"Added " << *new_feature);
        }
    }
    for(observation_ptr_t observation : observation_space) {
        new_feature = ObservationFeature::create(observation,0);
        basis_features.insert(new_feature);
        DEBUG_OUT(3,"Added " << *new_feature);
        for(int delay : new_observation_delays) {
            new_feature = ObservationFeature::create(observation,delay);
            basis_features.insert(new_feature);
            DEBUG_OUT(3,"Added " << *new_feature);
        }
    }
    for(reward_ptr_t reward : reward_space) {
        new_feature = RewardFeature::create(reward,0);
        basis_features.insert(new_feature);
        DEBUG_OUT(3,"Added " << *new_feature);
        for(int delay : new_reward_delays) {
            new_feature = RewardFeature::create(reward,delay);
            basis_features.insert(new_feature);
            DEBUG_OUT(3,"Added " << *new_feature);
        }
    }
    return this->expand_with_basis_features(current_features,basis_features);
}

void ConjunctiveAdjacency::set_horizon_extension(int h) {
    if(action_space==action_ptr_t() ||
       observation_space==observation_ptr_t() ||
       reward_space==reward_ptr_t()) {
        DEBUG_WARNING("Spaces not set: " << action_space
                      << "/" << observation_space
                      << "/" << reward_space);
    }
    if(h<0) {
        DEBUG_WARNING("Cannot use negative horizon extension of " << h);
        h = 0;
    }
    horizon_extension = h;
}

void ConjunctiveAdjacency::set_max_horizon(int h) {
    max_horizon = h;
}

void ConjunctiveAdjacency::set_min_horizon(int h) {
    min_horizon = h;
}

void ConjunctiveAdjacency::set_combine_features(bool b) {
    combine_features = b;
}

void ConjunctiveAdjacency::add_delay(f_ptr_t f,
                                     set<int>& action_delays,
                                     set<int>& observation_delays,
                                     set<int>& reward_delays) const {
    switch(f->get_feature_type()) {
    case Feature::CONST_FEATURE:
        break;
    case Feature::ACTION:
    {
        auto af = dynamic_pointer_cast<const ActionFeature>(f);
        if(af!=nullptr) {
            action_delays.insert(af->get_delay());
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    break;
    case Feature::OBSERVATION:
    {
        auto of = dynamic_pointer_cast<const ObservationFeature>(f);
        if(of!=nullptr) {
            observation_delays.insert(of->get_delay());
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    break;
    case Feature::REWARD:
    {
        auto rf = dynamic_pointer_cast<const RewardFeature>(f);
        if(rf!=nullptr) {
            reward_delays.insert(rf->get_delay());
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    break;
    case Feature::AND:
    {
        auto andf = dynamic_pointer_cast<const AndFeature>(f);
        if(andf!=nullptr) {
            for(f_ptr_t sub_f : andf->get_subfeatures()) {
                add_delay(sub_f,action_delays,observation_delays,reward_delays);
            }
        } else {
            DEBUG_DEAD_LINE;
        }
    }
    break;
    case Feature::ABSTRACT:
        DEBUG_WARNING("Abstract feature");
        break;
    default:
        DEBUG_WARNING("Unknown feature type: " << *f);
    }
}
