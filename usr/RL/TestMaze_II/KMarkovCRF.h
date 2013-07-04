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

#include "Config.h"

#include "Feature.h"
#include "HistoryObserver.h"

class KMarkovCRF: public HistoryObserver
{
public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value f_ret_t;

    KMarkovCRF();

    virtual ~KMarkovCRF();

    /** \brief Calls evaluate_model() on instance. */
    static lbfgsfloatval_t static_evaluate_model(
            void *instance,
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n,
            const lbfgsfloatval_t step
    );

    /** \brief Evaluate objective and gradient. */
    lbfgsfloatval_t evaluate_model(
            const lbfgsfloatval_t *x,
            lbfgsfloatval_t *g,
            const int n
    );

    /** \brief Calls progress_model() on instance. */
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

    /** \brief Prints progress information during optimization. */
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

    /** \brief Optimize the model.
     *
     * \param l1              Coefficient for \f$L^1\f$-regularization.
     * \param max_iter        Maximum number of iterations (zero for unlimited until convergence).
     * \param mean_likelihood For returning the mean likelihood after the last iteration.
     * */
    int optimize_model(lbfgsfloatval_t l1 = 0, unsigned int max_iter = 0, lbfgsfloatval_t * mean_likelihood = nullptr);

    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

    void evaluate_features();

    void score_features_by_gradient(const int& n = 1);

    void sort_scored_features(bool divide_by_complexity = false);

    void add_candidate_features_to_active(const int& n);

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

    void test();

private:

    typedef std::tuple<const instance_t*, action_t, state_t, reward_t> prediction_tuple_t;
    typedef std::map<prediction_tuple_t,probability_t> prediction_map_t;

    typedef std::tuple<const instance_t*, action_t> input_tuple_t;
    typedef std::set<input_tuple_t> input_set_t;

    //------------------------//
    // Features, Weights etc. //
    //------------------------//
    lbfgsfloatval_t * lambda;                        ///< Coefficients for active features.
    std::vector<Feature*> basis_features;            ///< Basis features used to construct new candidates.
    std::vector<AndFeature> active_features;         ///< Set of currently active features.
    std::vector<AndFeature> candidate_features;      ///< Set of candidate features.
    std::vector<double> candidate_feature_scores;    ///< Scores for candidate features.
    int old_active_features_size;                    ///< Number of active features before adding new candidates.
    bool candidate_features_sorted;                  ///< Whether the candidate features are sorted.

    //--------------------------//
    // Performance Optimization //
    //--------------------------//
    enum PRECOMPUTATION_TYPE { NONE, COMPOUND_LOOK_UP, BASE_LOOK_UP };
    const PRECOMPUTATION_TYPE precomputation_type = BASE_LOOK_UP;               ///< Technique for precomputing feature values.
    bool                      feature_values_precomputed;                       ///< Whether feature values are up-to-date.
    std::vector<f_ret_t>      compound_feature_values;                          ///< The precomputed feature values (COMPOUND_LOOK_UP).
    std::vector<std::vector<Feature::look_up_map_t> >  base_feature_values;     ///< The precomputed feature values (BASE_LOOK_UP).
    std::vector<idx_t>        base_feature_indices;                             ///< State-reward index for given instance.

    //------------------//
    // k-MDP Prediction //
    //------------------//
    prediction_map_t prediction_map;                 ///< Stores relative counts used for k-MDP predictions.
    input_set_t input_set;                           ///< Stores all k-MDP inputs for which relative counts were computed.

    //------------------//
    // Member Functions //
    //------------------//

    /** \brief Check whether the size of parameter vector matches number of
     * active features and adjust otherwise. */
    void check_lambda_size();

    /** \brief Constructs new candidate features. */
    void construct_candidate_features(const int& n);

    /** \brief Get index for precomputed feature values.
     *
     * Using this function ensures that the correct value is accessed even if
     * iteration is done in a different order. */
    idx_t precomputed_feature_idx(
        const idx_t& instance_idx,
        const idx_t& feature_idx,
        const idx_t& feature_n,
        const state_t& state,
        const reward_t& reward,
        const bool& use_state_and_reward
        );
    idx_t precomputed_feature_idx(
        const idx_t& instance_idx,
        const idx_t& feature_idx,
        const idx_t& feature_n
        );
    idx_t precomputed_feature_idx(
        const idx_t& instance_idx,
        const idx_t& feature_idx,
        const idx_t& feature_n,
        const state_t& state,
        const reward_t& reward
        );

    /** \brief Precompute compound feature values. */
    void precompute_compound_feature_values();

    /** \brief Precompute base feature values. */
    void precompute_base_feature_values();

    /** \brief Erase all features that never become non-zero from active
     * features. */
    void erase_const_zero_candidate_features();
};

#endif /* KMARKOVCRF_H_ */
