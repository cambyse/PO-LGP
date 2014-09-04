#ifndef LBFGS_OBJECT_H_
#define LBFGS_OBJECT_H_

#include <functional>
#include <vector>

#include <lbfgs.h>

class LBFGS_Object {

public:

    /** \brief Type for objective function.
     *
     * Computes objective and gradient. Value of objective is returned.
     * @param x current parameters
     * @param g gradient (to be filled) */
    typedef std::function<lbfgsfloatval_t(const lbfgsfloatval_t* x,
                                          lbfgsfloatval_t* g)> objective_t;

    /** \brief Type for progress function.
     *
     * @param x current parameters
     * @param g gradient
     * @param fx objective value
     * @param norm of current parameters
     * @param norm of gradient
     * @param step ???
     * @param nr_variables number of parameters
     * @param iteration_nr current number of iterations
     * @param ls ??? */
    typedef std::function<int(const lbfgsfloatval_t *x,
                              const lbfgsfloatval_t *g,
                              const lbfgsfloatval_t fx,
                              const lbfgsfloatval_t xnorm,
                              const lbfgsfloatval_t gnorm,
                              const lbfgsfloatval_t step,
                              int nr_variables,
                              int iteration_nr,
                              int ls)> progress_t;

    LBFGS_Object() = default;

    virtual ~LBFGS_Object();

    virtual lbfgsfloatval_t optimize(objective_t objective = nullptr,
                                     lbfgsfloatval_t * values = nullptr,
                                     int * return_code = nullptr,
                                     std::string * return_code_description = nullptr
        );

    virtual lbfgsfloatval_t optimize(std::vector<lbfgsfloatval_t> & values,
                                     int * return_code = nullptr,
                                     std::string * return_code_description = nullptr
        );

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

    bool check_derivatives(const int& number_of_samples,
                           const double& range,
                           const double& delta = 1e-5,
                           const double& allowed_maximum_relative_deviation = 1e-5,
                           const double& minimum_gradient = 0,
                           const bool use_current_values = false
        );

    /** \brief Set the objective function. */
    virtual LBFGS_Object& set_objective(objective_t objective);

    /** \brief Set values of the variables. */
    virtual LBFGS_Object& set_variables(const std::vector<lbfgsfloatval_t> & values);

    /** \brief Set the objective function. */
    virtual LBFGS_Object& set_progress(progress_t progress);

    /** \brief Get the number of variables that are optimized. */
    virtual unsigned int get_number_of_variables() const {return number_of_variables;}

    /** \brief Set the number of variables that are optimized. */
    virtual LBFGS_Object& set_number_of_variables(unsigned int n, bool zero = false);

    /** \brief Get maximum number of iterations. */
    virtual unsigned int get_maximum_iterations() const {return maximum_iterations;}

    /** \brief Set maximum number of iterations. */
    virtual LBFGS_Object& set_maximum_iterations(unsigned int n );

    /** \brief Get the factor for L1-regularization. */
    virtual lbfgsfloatval_t get_l1_factor() const {return l1_factor;}

    /** \brief Set the factor for L1-regularization. */
    virtual LBFGS_Object& set_l1_factor(lbfgsfloatval_t f );

    /** \brief Get epsilon. */
    virtual lbfgsfloatval_t get_epsilon() const {return epsilon;}

    /** \brief Set epsilon. */
    virtual LBFGS_Object& set_epsilon(lbfgsfloatval_t f );

    /** \brief Get delta. */
    virtual lbfgsfloatval_t get_delta() const {return delta;}

    /** \brief Set delta. */
    virtual LBFGS_Object& set_delta(lbfgsfloatval_t f );

private:
    lbfgsfloatval_t * variables           = nullptr;       ///< Variable values.
    unsigned int      number_of_variables = 0;             ///< The dimension of the objective.
    unsigned int      maximum_iterations  = 0;             ///< Maximum number of iteration (0 for infinite).
    lbfgsfloatval_t   l1_factor           = 0;             ///< Factor for L^1 regularization (0 for no regularization).
    lbfgsfloatval_t   delta               = 0;             ///< Minimum change of objective (f-f')/f as stopping criterion (default 0).
    lbfgsfloatval_t   epsilon             = 1e-5;          ///< Minimum change of variables ||g||/max(1,||x||) as stopping criterion (default 1e-5).
    objective_t       objective           = nullptr; ///< The objective function that is to be optimized.
    progress_t        progress            = nullptr;  ///< The progress function used to print info.

    void call_lbfgs(int & ret, lbfgsfloatval_t & fx);
};

#endif /* LBFGS_OBJECT_H_ */
