#ifndef CONJUNCTIVEADJACENCY_H_
#define CONJUNCTIVEADJACENCY_H_

#include "AdjacencyOperator.h"
#include "../SpaceManager.h"

class ConjunctiveAdjacency: public AdjacencyOperator, public SpaceManager {

    //----typedefs/classes----//
    DISAMBIGUATE_CONFIG_TYPEDEFS(AdjacencyOperator);

    //----members----//
private:
    bool extend_with_basis_features_only = false;
    int horizon_extension = 0;
    int max_horizon = -1;

    //----methods----//
public:
    ConjunctiveAdjacency() = default;
    virtual ~ConjunctiveAdjacency() = default;
    virtual f_set_t expand_with_basis_features(
        const f_set_t& current_features = f_set_t(),
        const f_set_t& basis_features = f_set_t()
        ) const;
    virtual f_set_t operator()(
        const f_set_t& current_features = f_set_t()
        ) const;
    virtual void set_horizon_extension(int h = 0);
    virtual void set_max_horizon(int h = -1);

    /** \brief Whether to also use conjunctions of pairs of existing features or
     * only conjunctions of existing features and basis features. */
    virtual void combine_features(bool b = false);
private:
    void add_delay(f_ptr_t,std::set<int>&,std::set<int>&,std::set<int>&) const;
};

#endif /* CONJUNCTIVEADJACENCY_H_ */
