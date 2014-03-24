#ifndef ADJACENCYOPERATOR_H_
#define ADJACENCYOPERATOR_H_

#include "../Representation/Feature.h"

#include <set>

class AdjacencyOperator {
public:
    typedef Feature::const_feature_ptr_t f_ptr_t;
    typedef std::set<f_ptr_t> feature_set_t;
    AdjacencyOperator() = default;
    virtual ~AdjacencyOperator() = default;
    virtual feature_set_t operator()(
        const feature_set_t& current_features = feature_set_t(),
        const feature_set_t& basis_features = feature_set_t()) const = 0;
};

#endif /* ADJACENCYOPERATOR_H_ */
