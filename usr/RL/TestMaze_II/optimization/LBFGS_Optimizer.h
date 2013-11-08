#ifndef LBFGS_OPTIMIZER_H_
#define LBFGS_OPTIMIZER_H_

#include <string>

#include <lbfgs.h>

/** \brief Abstract class for using L-BFGS optimizer.
 *
 * Inherit from this class and reimplement the LBFGS_Optimizer::objective
 * function to use the L-BFGS optimizer.*/
class LBFGS_Optimizer {

public:

    LBFGS_Optimizer();
    virtual ~LBFGS_Optimizer();

    /** \brief Calls instance->objective(). */
    static lbfgsfloatval_t static_objective(
        void *instance,             ///< LBFGS_Optimizer object use
        const lbfgsfloatval_t *x,   ///< Current variable values
        lbfgsfloatval_t *g,         ///< Gradient to be computed
        const int n,                ///< Number of variables
        const lbfgsfloatval_t step  ///< Line-search step in this iteration
    );

    /** \brief Calls instance->progress(). */
    static int static_progress(
        void *instance,                ///< LBFGS_Optimizer object use
        const lbfgsfloatval_t *x,      ///< Current variable values
        const lbfgsfloatval_t *g,      ///< Gradient to be computed
        const lbfgsfloatval_t fx,      ///< Current value of the objective
        const lbfgsfloatval_t xnorm,   ///< Euclidian norm of the variables
        const lbfgsfloatval_t gnorm,   ///< Euclidian norm of the gradient
        const lbfgsfloatval_t step,    ///< Line-search step in this iteration
        int n,                         ///< Number of variables
        int k,                         ///< Iteration count
        int ls                         ///< Number of evaluation for this iteration
        );

    /** \brief Optimize the objective and return its final value. */
    virtual lbfgsfloatval_t optimize(
        int * return_code = nullptr,
        std::string * return_code_description = nullptr
        );

    /** \brief Compute numerical gradient and compare to analytically computed
     * gradient.
     *
     * The function draws uniformely distributed random samples for all
     * variables. It then computes the numerical gradient \e grad_n and compares
     * it to the analytical gradient \e grad_a computed in the objective()
     * function. If for any variable \e v the relative deviation |grad_n(v) -
     * grad_a(v)|/min(|grad_a(v)|,|grad_n(v)|) is larger than the maximum
     * allowed deviation it reports an error.
     *
     * @param number_of_samples The number of random samples to be drawn.
     *
     * @param range Samples are uniformely drawn from [-range,+range] for every
     * variable (possibly shifted by the current variable values see \e
     * use_current_values).
     *
     * @param delta The numerical gradient is computed by computing the finite
     * difference of the objective for v+delta and v-delta.
     *
     * @param allowed_maximum_relative_deviation The maximum allowed relative
     * deviation not considered as an error.
     *
     * @param use_current_values Whether sampling is centered around the current
     * variable values (stored in lbfgs_variables) or around zero. */
    void check_derivatives(const int& number_of_samples,
                           const double& range,
                           const double& delta = 1e-10,
                           const double& allowed_maximum_relative_deviation = 1e-10,
                           const bool use_current_values = false
        );

    /** \brief Get the number of variables that are optimized. */
    virtual unsigned int    get_number_of_variables() const {return number_of_variables;}

    /** \brief Get maximum number of iterations. */
    virtual unsigned int    get_maximum_iterations() const {return maximum_iterations;}

    /** \brief Get the factor for L1-regularization. */
    virtual lbfgsfloatval_t get_l1_factor() const {return l1_factor;}

    /** \brief Get lbfgs_epsilon. */
    virtual lbfgsfloatval_t get_lbfgs_epsilon() const {return lbfgs_epsilon;}

    /** \brief Get lbfgs_delta. */
    virtual lbfgsfloatval_t get_lbfgs_delta() const {return lbfgs_delta;}


protected:

    lbfgsfloatval_t * lbfgs_variables = nullptr; ///< Variable values.


    /** \brief Compute gradient and return value of objective. */
    virtual lbfgsfloatval_t objective(
        const lbfgsfloatval_t *x,       ///< Current variable values
        lbfgsfloatval_t *g,             ///< Gradient to be computed
        const int n                     ///< Number of variables
        );


    /** \brief Prints progress information during optimization. */
    virtual int progress(
        const lbfgsfloatval_t *x,      ///< Current variable values
        const lbfgsfloatval_t *g,      ///< Gradient to be computed
        const lbfgsfloatval_t fx,      ///< Current value of the objective
        const lbfgsfloatval_t xnorm,   ///< Euclidian norm of the variables
        const lbfgsfloatval_t gnorm,   ///< Euclidian norm of the gradient
        const lbfgsfloatval_t step,    ///< Line-search step in this iteration
        int n,                         ///< Number of variables
        int k,                         ///< Iteration count
        int ls                         ///< Number of evaluation for this iteration
        );

    /** \brief Set the number of variables that are optimized. */
    virtual LBFGS_Optimizer& set_number_of_variables(unsigned int n, bool zero = false);

    /** \brief Set the factor for L1-regularization. */
    virtual LBFGS_Optimizer& set_l1_factor(lbfgsfloatval_t f );

    /** \brief Set lbfgs_delta. */
    virtual LBFGS_Optimizer& set_lbfgs_delta(lbfgsfloatval_t f );

    /** \brief Set maximum number of iterations. */
    virtual LBFGS_Optimizer& set_maximum_iterations(unsigned int n );

    /** \brief Set lbfgs_epsilon. */
    virtual LBFGS_Optimizer& set_lbfgs_epsilon(lbfgsfloatval_t f );

private:

    unsigned int      number_of_variables = 0;       ///< The dimension of the objective.
    unsigned int      maximum_iterations  = 0;       ///< Maximum number of iteration (0 for infinite).
    lbfgsfloatval_t   l1_factor           = 0;       ///< Factor for L^1 regularization (0 for no regularization).
    lbfgsfloatval_t   lbfgs_delta         = 0;       ///< Minimum change of objective (f-f')/f as stopping criterion (default 0).
    lbfgsfloatval_t   lbfgs_epsilon       = 1e-5;    ///< Minimum change of variables ||g||/max(1,||x||) as stopping criterion (default 1e-5).

};

#endif /* LBFGS_OPTIMIZER_H_ */
