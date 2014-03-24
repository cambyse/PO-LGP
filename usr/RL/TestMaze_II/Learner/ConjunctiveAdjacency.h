#ifndef CONJUNCTIVEADJACENCY_H_
#define CONJUNCTIVEADJACENCY_H_

#include "AdjacencyOperator.h"

class ConjunctiveAdjacency: public AdjacencyOperator {
public:
    ConjunctiveAdjacency() = default;
    virtual ~ConjunctiveAdjacency() = default;
    virtual feature_set_t operator()(
        const feature_set_t& current_features = feature_set_t(),
        const feature_set_t& basis_features = feature_set_t()) const override;
    bool use_basis_features_only = false;
};

#endif /* CONJUNCTIVEADJACENCY_H_ */
