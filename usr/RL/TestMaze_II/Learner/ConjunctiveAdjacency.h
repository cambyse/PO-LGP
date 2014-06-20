#ifndef CONJUNCTIVEADJACENCY_H_
#define CONJUNCTIVEADJACENCY_H_

#include "AdjacencyOperator.h"
#include "../SpaceManager.h"

class ConjunctiveAdjacency: public AdjacencyOperator, public SpaceManager {

    //----typedefs/classes----//
public:
    DISAMBIGUATE_CONFIG_TYPEDEFS(AdjacencyOperator);
    enum T_ZERO_FEATURES { NONE, ACTION, ACTION_OBSERVATION_REWARD };

    //----members----//
private:
    bool combine_features = false;
    int horizon_extension = 0;
    int max_horizon = -1;
    int min_horizon = -1;
    bool allow_zero_delay = true;
    T_ZERO_FEATURES t_zero_features = ACTION_OBSERVATION_REWARD;

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
    virtual int get_horizon_extension() const { return horizon_extension; }
    virtual void set_horizon_extension(int h = 0);
    virtual int get_max_horizon() const { return max_horizon; }
    virtual void set_max_horizon(int h = -1);
    virtual int get_min_horizon() const { return min_horizon; }
    virtual void set_min_horizon(int h = -1);

    /** \brief Whether to also use conjunctions of pairs of existing features or
     * only conjunctions of existing features and basis features. */
    virtual bool get_combine_features() const { return combine_features; }
    virtual void set_combine_features(bool b);
    virtual void set_t_zero_features(T_ZERO_FEATURES f) { t_zero_features = f; }
private:
    void add_delay(f_ptr_t,std::set<int>&,std::set<int>&,std::set<int>&) const;
};

#endif /* CONJUNCTIVEADJACENCY_H_ */
