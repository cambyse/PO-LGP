#ifndef ADJACENCYOPERATOR_H_
#define ADJACENCYOPERATOR_H_

#include "../Config.h"

class AdjacencyOperator {
public:
    USE_CONFIG_TYPEDEFS;
    AdjacencyOperator() = default;
    virtual ~AdjacencyOperator() = default;
    virtual feature_set_t operator()(
        const feature_set_t& current_features = feature_set_t()
        ) const = 0;
};

#endif /* ADJACENCYOPERATOR_H_ */
