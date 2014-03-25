#include "TemporallyExtendedModel.h"

#include "ConjunctiveAdjacency.h"

#define DEBUG_LEVEL 0
#include "../util/debug.h"

typedef TemporallyExtendedModel TEM;

TEM::TemporallyExtendedModel(std::shared_ptr<ConjunctiveAdjacency> N): N_plus(N) {}

TEM::probability_t TEM::get_prediction(const_instance_ptr_t,
                                       const action_ptr_t&,
                                       const observation_ptr_t&,
                                       const reward_ptr_t&) const {
    return 0;
}

void TEM::extend_features() {
    DEBUG_OUT(2,"Extending features");
    feature_set_t extension_features = (*N_plus)(feature_set);
    feature_set.insert(extension_features.begin(),extension_features.end());
    DEBUG_OUT(2,"DONE (" << feature_set.size() << " features)");
}

void TEM::print_features() const {
    DEBUG_OUT(0,"Feature Set:");
    int f_idx = 0;
    for(f_ptr_t f : feature_set) {
        DEBUG_OUT(0,"    f[" << f_idx << "]: " << *f);
        ++f_idx;
    }
}
