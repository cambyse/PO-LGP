#ifndef CONJUNCTIVEADJACENCY_H_
#define CONJUNCTIVEADJACENCY_H_

#include "AdjacencyOperator.h"
#include "../SpaceManager.h"

class ConjunctiveAdjacency: public AdjacencyOperator, public SpaceManager {
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(AdjacencyOperator);
    bool extend_with_basis_features_only = false;
    ConjunctiveAdjacency() = default;
    virtual ~ConjunctiveAdjacency() = default;
    virtual f_set_t expand_with_basis_features(
        const f_set_t& current_features = f_set_t(),
        const f_set_t& basis_features = f_set_t()
        ) const;
    virtual f_set_t operator()(
        const f_set_t& current_features = f_set_t()
        ) const;
    virtual void horizon_extension_on(int);
    virtual void horizon_extension_off();
private:
    int horizon_extension = 0;
    void add_delay(f_ptr_t,std::set<int>&,std::set<int>&,std::set<int>&) const;
};

#endif /* CONJUNCTIVEADJACENCY_H_ */
