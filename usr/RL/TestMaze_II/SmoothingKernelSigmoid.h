#ifndef SMOOTHINGKERNELSIGMOID_H
#define SMOOTHINGKERNELSIGMOID_H

#include <lbfgs.h>

#include <vector>
#include <utility> // std::pair

class QCustomPlot;

class SmoothingKernelSigmoid {

public:
    SmoothingKernelSigmoid(const double& min_x,
                           const double& max_x,
                           const int& point_n_,
                           const double& width,
                           const double& g
        );
    ~SmoothingKernelSigmoid();
    void add_new_point(const double& x, const double& y);
    void initialize_data();
    double print_to_QCP(QCustomPlot * plotter,
                      const bool& print_upper = true,
                      const bool& print_lower = true
        ) const;
    void optimize_sigmoid(const int& iterations);
    void optimize_upper_bound(const double& strength, const int& iterations);
    void optimize_lower_bound(const double& strength, const int& iterations);

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
    int optimize_model(unsigned int max_iter = 0);

    void check_derivatives(const int& number_of_samples, const double& range, const double& max_variation, const double& max_relative_deviation);

protected:
    const int control_point_n;
    int raw_data_n;
    bool initialized;
    double bias_factor, kernel_width, gamma;
    double infinity; // used when the objective is has infinite values
    double bound_scaling;
    std::vector<double> x_data, y_data, x_sig, y_sig, y_sig_upper, y_sig_lower;
    lbfgsfloatval_t * y_values;

    void optimize(const int& iterations = -1);
    void initialize();
    void lbfgs_to_vec();
    void vec_to_lbfgs();
    double kernel(const double& x1, const double& x2) const;
    double kernel_smoothed_data(const double& x) const;
    void bounds(const double& d,
                double& cost,
                double& grad) const;
    void mean_dev(const double& x, double& mean, double& dev) const;
};

#endif // SMOOTHINGKERNELSIGMOID_H
