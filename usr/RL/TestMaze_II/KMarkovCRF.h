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

#include <QtCore/QString>

#include <lbfgs.h>

#include "Data.h"
#include "Feature.h"

class KMarkovCRF
{
public:

    typedef Data::action_t                 action_t;
    typedef Data::state_t                  state_t;
    typedef Data::reward_t                 reward_t;
    typedef Data::probability_t            probability_t;
    typedef Data::episode_t                episode_t;
    typedef Data::const_episode_iterator_t const_episode_iterator_t;
    typedef Data::data_point_t             data_point_t;
    typedef Data::input_data_t             input_data_t;
    typedef Data::output_data_t            output_data_t;
    typedef Data::OutputIterator           OutputIterator;

    KMarkovCRF(const int& k);

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

    void optimize_model(lbfgsfloatval_t l1 = 0);

    void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    void clear_data() { episode_data.clear(); }

    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

//    probability_t probability(input_data_t input_data);
//    probability_t probability(input_data_t input_data, output_data_t output_data);

    void evaluate_features();

//    void rank_pair_features();

    void score_features();

private:

    typedef std::unique_ptr<Feature> unique_feature_ptr;

    int k;
    episode_t episode_data;
    lbfgsfloatval_t * lambda;
    std::vector<int> parameter_indices;
    std::set<unique_feature_ptr> feature_set;
    std::vector<Feature*> basic_features, active_features;

//    int data_features_n, predict_features_n, combined_features_n;
//    std::vector<unique_feature_ptr> features, output_features, subfeatures;
//    std::vector<Feature*> data_features, predict_features, combined_features;

};

#endif /* KMARKOVCRF_H_ */
