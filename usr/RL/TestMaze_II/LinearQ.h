#ifndef LINEARQ_H_
#define LINEARQ_H_

#include "Config.h"

#include "Feature.h"
#include "HistoryObserver.h"

#define ARMA_NO_DEBUG
#include <armadillo>

#include <lbfgs.h>

#include <vector>

class LinearQ: public HistoryObserver
{
public:

    USE_CONFIG_TYPEDEFS;
    typedef Feature::feature_return_value    f_ret_t;
    typedef std::vector<const instance_t *>  instance_vector_t;

    LinearQ(const double&);
    virtual ~LinearQ();

    /** \brief Add a new instance to the tree. */
    virtual void add_action_state_reward_tripel(
            const action_t& action,
            const state_t& state,
            const reward_t& reward
    );

    /** \brief Optimize weights via ridge regression.
     *
     * Optimize the weights for the currently active features using a
     * \f$L^2\f$-regularization and returning the optimum value of the objective
     * function.
     *
     * @param reg Coefficient for the \f$L^2\f$-regularization. */
    double optimize_ridge(const double& reg);

    /** \brief Optimize weights via \f$L^1\f$-regression.
     *
     * Optimize the weights for the currently active features using a
     * \f$L^1\f$-regularization, write the optimum value of the objective
     * function into loss (if provided) and return L-BFGS return code.
     *
     * @param reg      Coefficient for the \f$L^1\f$-regularization.
     * @param max_iter Maximum number of iterations (zero for unlimited until convergence).
     * @param loss     Pointer to return the value of the objective function after the last iteration.
     * */
    int optimize_l1(const double& reg, const int& max_iter = 0, double * loss = nullptr);

    /** \brief Clears all data (all instances). */
    virtual void clear_data();

    action_t get_max_value_action(const instance_t *);

    /*! \brief Set the discount rate used for computing state and action values. */
    void set_discount(const double& d) { discount = d; }

    void add_candidates(const int& n);

    void erase_zero_features();

    void erase_zero_weighted_features(const double& threshold = 0);

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

    void check_derivatives(const int& number_of_samples,
                           const double& range,
                           const double& max_variation,
                           const double& max_relative_deviation
        );

private:

    //--------------//
    // General Data //
    //--------------//
    double discount;                                 ///< Discount for computing value.

    //------------------------//
    // Features, Weights etc. //
    //------------------------//
    std::vector<double> feature_weights;             ///< Coefficients for active features.
    std::vector<Feature*> basis_features;            ///< Basis features used to construct new candidates.
    lbfgsfloatval_t * lambda;                        ///< Feature weights for L1-optimization.
    std::vector<AndFeature> active_features;         ///< Set of currently active features.
    std::vector<AndFeature> candidate_features;      ///< Set of candidate features.

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

    /** \brief Update LinearQ::L, LinearQ::rho, and LinearQ::c. */
    void update_loss_terms();

    /** \brief Constructs new candidate features. */
    void construct_candidate_features(const int& n);

    probability_t prior_probability(const state_t&, const reward_t&) const;

    /** \brief Calculate loss function. */
    double loss_function(const arma::vec& w) const { return arma::as_scalar(c + 2*rho.t()*w + w.t()*L*w); }

    /** \brief Calculate gradient of the loss function. */
    arma::vec loss_gradient(const arma::vec& w) const { return 2 * rho + 2 * L * w; }
};

#endif /* LINEARQ_H_ */
