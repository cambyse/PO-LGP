#ifndef LINEARQ_H_
#define LINEARQ_H_

#include "../Config.h"

#include "../Representation/Feature.h"
#include "../HistoryObserver.h"
#include "FeatureLearner.h"
#include "../Planning/Policy.h"
#include "../optimization/LBFGS_Optimizer.h"

#define ARMA_NO_DEBUG
#include <armadillo>

#include <lbfgs.h>

#include <vector>

#include "../util/debug.h"

class LinearQ: public HistoryObserver, public LBFGS_Optimizer, public FeatureLearner, public Policy
{
public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value    f_ret_t;
    typedef Feature::const_feature_ptr_t     f_ptr_t;
    typedef std::vector<const_instance_ptr_t>  instance_vector_t;

    /** \brief What strategy to use for optimizting the features weights. */
    enum class OPTIMIZATION_TYPE {
        TD_RIDGE, ///< Ridge regression on the TD error (solving a system of linear equations)
        TD_L1,    ///< Minimize TD error with L1-regularization (objective can be computed efficiently)
        BELLMAN   ///< Minimize Bellman error
    };

    LinearQ(const double&);
    virtual ~LinearQ();

    /** \brief Add a new instance to the tree. */
    virtual void add_action_observation_reward_tripel(
            const action_ptr_t& action,
            const observation_ptr_t& observation,
            const reward_ptr_t& reward,
            const bool& new_episode
    );

    /** \brief Clears all data (all instances). */
    virtual void clear_data();

    action_ptr_t get_max_value_action(const_instance_ptr_t);

    virtual action_ptr_t get_action(const_instance_ptr_t i) { return get_max_value_action(i); }

    /*! \brief Set the discount rate used for computing observation and action values. */
    void set_discount(const double& d) { discount = d; }

    /** \brief Constructs new candidate features. */
    void construct_candidate_features(const int& n);

    /** \brief Score the candidate features by their gradient.
     *
     * Temporally adds the canidates to the active features and calls the
     * objective to compute the gradient. Sorts the candidates by their
     * gradient.*/
    void score_candidates_by_gradient();

    /** \brief Add the n highest scored candidates. Never add zero scored
     * features. For n==0 add all non-zero scored. */
    void add_candidates_by_score(int n);

    /** \brief Construct and add all candidates with 'distance' n. */
    void add_all_candidates(const int& n);

    /** \brief Erase from active features all that have a weight less than or
     * equal to threshold. */
    void erase_features_by_weight(const double& threshold = 0);

    /** \brief Overrides the function in LBFGS_Optimizer and calls the objective
     * defined by LinearQ::optimization_type. */
    virtual lbfgsfloatval_t objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
        ) override;

    /** \brief Objective for OPTIMIZATION_TYPE::TD_L1. */
    virtual lbfgsfloatval_t td_error_objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
        );

    /** \brief Objective for OPTIMIZATION_TYPE::BELLMAN. */
    virtual lbfgsfloatval_t bellman_objective(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n
        );

    /** \brief Prints progress information during optimization. */
    virtual int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        ) override;

    /** \brief Optimize weights and return loss.
     *
     * This function wraps the corresponding function in LBFGS_Optimizer but
     * additionally reports the results */
    virtual lbfgsfloatval_t optimize(
        int * return_code = nullptr,
        std::string * return_code_description = nullptr
        ) override;

    /** \brief Sets the regularization factor for both ridge and
     * L1-regularization. */
    virtual LinearQ& set_regularization(lbfgsfloatval_t f );

    /** \brief Calls the correpsonding private function in LBFGS_Optimizer. */
    virtual LinearQ& set_lbfgs_delta(lbfgsfloatval_t f ) override;

    /** \brief Calls the correpsonding private function in LBFGS_Optimizer. */
    virtual LinearQ& set_maximum_iterations(unsigned int n ) override;

    /** \brief Calls the correpsonding private function in LBFGS_Optimizer. */
    virtual LinearQ& set_lbfgs_epsilon(lbfgsfloatval_t f ) override;

    /** \brief Sets LinearQ::optimization_type. */
    virtual LinearQ& set_optimization_type(OPTIMIZATION_TYPE t );

    /** \brief Convenience function. */
    LinearQ& set_optimization_type_TD_RIDGE() { return set_optimization_type(OPTIMIZATION_TYPE::TD_RIDGE); }
    /** \brief Convenience function. */
    LinearQ& set_optimization_type_TD_L1() { return set_optimization_type(OPTIMIZATION_TYPE::TD_L1); }
    /** \brief Convenience function. */
    LinearQ& set_optimization_type_BELLMAN() { return set_optimization_type(OPTIMIZATION_TYPE::BELLMAN); }

    virtual LinearQ& set_alpha(const double a) { alpha = a; return *this; }
    virtual double get_alpha() { return alpha; }

    int get_number_of_features() const { return active_features.size(); }

private:

    //--------------//
    // General Data //
    //--------------//
    double discount; ///< Discount for computing value.
    double alpha = 3; ///< Alpha for Soft-Max.
    OPTIMIZATION_TYPE optimization_type = OPTIMIZATION_TYPE::BELLMAN; ///< Optimization strategy.

    //------------------------//
    // Features, Weights etc. //
    //------------------------//
    std::vector<double> feature_weights;             ///< Coefficients for active features.
    std::vector<AndFeature> active_features;         ///< Set of currently active features.
    std::vector<AndFeature> candidate_features;      ///< Set of candidate features.
    std::vector<double> candidate_scores;            ///< Scores for the candidate features.

    //-------------------------//
    // Terms for Loss Function //
    //-------------------------//
    arma::mat L;                ///< for computing \f$ \ell(\omega) = c + \rho^\top \omega + \omega^\top \mathcal{L} \; \omega \f$.
    arma::vec rho;              ///< for computing \f$ \ell(\omega) = c + \rho^\top \omega + \omega^\top \mathcal{L} \; \omega \f$.
    double c;                   ///< for computing \f$ \ell(\omega) = c + \rho^\top \omega + \omega^\top \mathcal{L} \; \omega \f$.
    bool loss_terms_up_to_date; ///<< Whether LinearQ::L, LinearQ::rho, and LinearQ::c are up-to-date.

    //------------------//
    // Member Functions //
    //------------------//

    /** \brief Optimize weights via ridge regression.
     *
     * Optimize the weights for the currently active features using a
     * \f$L^2\f$-regularization and returning the optimum value of the objective
     * function.
     *
     * @param reg Coefficient for the \f$L^2\f$-regularization. */
    double optimize_TD_ridge(const double& reg);

    /** \brief Erase from feature_vector all features that are never
     * activated/non-zero. Modify weight_vector accordingly, if provided. */
    void erase_inactive_features(
        std::vector<AndFeature> * feature_vector,
        std::vector<double> * weight_vector = nullptr
        );

    /** \brief Erase from active features all that are never activated/non-zero
     * and modify weight_vector accordingly. */
    void erase_inactive_features_from_active() {
        erase_inactive_features(&active_features,&feature_weights);
        loss_terms_up_to_date = false;
    }

    /** \brief Update LinearQ::L, LinearQ::rho, and LinearQ::c. */
    void update_loss_terms();

    probability_t prior_probability(const observation_ptr_t&, const reward_ptr_t&) const;

    /** \brief Calculate loss function. */
    double loss_function(const arma::vec& w) const { return arma::as_scalar(c + 2*rho.t()*w + w.t()*L*w); }

    /** \brief Calculate gradient of the loss function. */
    arma::vec loss_gradient(const arma::vec& w) const { return 2 * rho + 2 * L * w; }

    /** \brief Set the number of variables to the number of active features and
     * transfers weights. */
    virtual void set_number_of_variables_to_active();

    /** \brief Sort candidate features by their score. */
    void sort_candidates_by_score(bool divide_by_complexity = false);
};

#include "../util/debug_exclude.h"

#endif /* LINEARQ_H_ */
