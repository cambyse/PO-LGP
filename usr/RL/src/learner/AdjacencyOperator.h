#ifndef ADJACENCYOPERATOR_H_
#define ADJACENCYOPERATOR_H_

#include <config/Config.h>

class AdjacencyOperator {
public:
    USE_CONFIG_TYPEDEFS;
    AdjacencyOperator() = default;
    virtual ~AdjacencyOperator() = default;
    virtual f_set_t operator()(
        const f_set_t& current_features = f_set_t()
        ) const = 0;
};

#endif /* ADJACENCYOPERATOR_H_ */
