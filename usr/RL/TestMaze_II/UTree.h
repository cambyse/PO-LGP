#ifndef UTREE_H_
#define UTREE_H_

#include <memory>
#include <vector>
#include <set>
#include <map>
#include <tuple>

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

class UTree
{
public:

    typedef Data::idx_t         idx_t;
    typedef Data::size_t        size_t;
    typedef Data::probability_t probability_t;

    UTree();

    virtual ~UTree();

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data() {
        delete instance_data;
        instance_data = nullptr;
    }

    probability_t get_prediction(const instance_t *, const action_t&, const state_t&, const reward_t&) const;
    probability_t (UTree::*get_prediction_ptr())(const instance_t *, const action_t&, const state_t&, const reward_t&) const {
        return &UTree::get_prediction;
    }

private:

    int k, old_active_features_size;
    instance_t * instance_data;
    std::vector<Feature*> basis_features;
    std::vector<AndFeature> active_features, compound_features;
    std::vector<double> compound_feature_scores;
    bool compound_features_sorted;

    void construct_compound_features(const int& n);
};

#endif /* UTREE_H_ */
