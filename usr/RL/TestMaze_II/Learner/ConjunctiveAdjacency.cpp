#include "ConjunctiveAdjacency.h"

#include "../Representation/Feature.h"

ConjunctiveAdjacency::feature_set_t ConjunctiveAdjacency::operator()(
    const feature_set_t& current_features,
    const feature_set_t& basis_features) const {
    feature_set_t new_features;
    for(f_ptr_t feature_1 : current_features) {
        // combine with all basis features
        for(f_ptr_t basis : basis_features) {
            new_features.insert(f_ptr_t(new AndFeature(feature_1,basis)));
        }
        // combine with other features
        if(!use_basis_features_only) {
            for(f_ptr_t feature_2 : current_features) {
                new_features.insert(f_ptr_t(new AndFeature(feature_1,feature_2)));
            }
        }
    }
    return new_features;
}
