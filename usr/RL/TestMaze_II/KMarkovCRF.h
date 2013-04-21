/*
 * KMarkovCRF.h
 *
 *  Created on: Oct 30, 2012
 *      Author: robert
 */

#ifndef KMARKOVCRF_H_
#define KMARKOVCRF_H_

#include <math.h>
#include <memory>
#include <vector>
#include <set>
#include <map>
#include <tuple>

#include <QtCore/QString>

#include <lbfgs.h>

#include "Data.h"
#include "Representation/Representation.h"
#include "Feature.h"

class KMarkovCRF
{
public:

    typedef Data::idx_t         idx_t;
    typedef Data::size_t        size_t;
    typedef Data::probability_t probability_t;

    KMarkovCRF();

    virtual ~KMarkovCRF();

    static lbfgsfloatval_t static_evaluate_model(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    lbfgsfloatval_t evaluate_model(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
    );

    static int static_progress_model(
            void *instance,
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    int progress_model(
            const lbfgsfloatval_t *x,
            const lbfgsfloatval_t *g,
            const lbfgsfloatval_t fx,
            const lbfgsfloatval_t xnorm,
            const lbfgsfloatval_t gnorm,
            const lbfgsfloatval_t step,
            int n,
            int k,
            int ls
    );

    int optimize_model(lbfgsfloatval_t l1 = 0, unsigned int max_iter = 0, lbfgsfloatval_t * mean_likelihood = nullptr);

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data() {
        // todo what to do here!?!
    }

    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

    void evaluate_features();

    void score_features_by_gradient(const int& n = 1);

    void sort_scored_features(bool divide_by_complexity = false);

    void add_compound_features_to_active(const int& n);

    void erase_zero_features();

    void erase_all_features();

    unsigned long int get_number_of_features();

    unsigned long int get_training_data_length();

    probability_t get_prediction(const instance_t *, const action_t&, const state_t&, const reward_t&) const;
    probability_t (KMarkovCRF::*get_prediction_ptr())(const instance_t *, const action_t&, const state_t&, const reward_t&) const {
        return &KMarkovCRF::get_prediction;
    }

    probability_t get_kmdp_prediction(const instance_t *, const action_t&, const state_t&, const reward_t&) const;
    probability_t (KMarkovCRF::*get_kmdp_prediction_ptr())(const instance_t *, const action_t&, const state_t&, const reward_t&) const {
        return &KMarkovCRF::get_kmdp_prediction;
    }

    void update_prediction_map();

private:

    typedef std::tuple<const instance_t*, action_t, state_t, reward_t> prediction_tuple_t;
    typedef std::map<prediction_tuple_t,probability_t> prediction_map_t;

    typedef std::tuple<const instance_t*, action_t> input_tuple_t;
    typedef std::set<input_tuple_t> input_set_t;

    int k, old_active_features_size;
    instance_t * instance_data;
    lbfgsfloatval_t * lambda;
    std::vector<Feature*> basis_features;
    std::vector<AndFeature> active_features, compound_features;
    std::vector<double> compound_feature_scores;
    bool compound_features_sorted, kmdp_prediction_up_to_data;
    prediction_map_t prediction_map;
    input_set_t input_set;

    void check_lambda_size();
    void construct_compound_features(const int& n);
};

#endif /* KMARKOVCRF_H_ */
