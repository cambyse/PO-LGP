#ifndef ACTIVESIGMOIDOPTIMIZATION_H
#define ACTIVESIGMOIDOPTIMIZATION_H

#include "SplineInterpolation.h"

#include <vector>
#include <utility> // std::pair

class QCustomPlot;

class ActiveSigmoidOptimization {

public:
    ActiveSigmoidOptimization(const double& min_x,
                              const double& max_x,
                              const double& y_init,
                              const int& point_n_
        );
    ~ActiveSigmoidOptimization();
    void add_new_point(const double& x, const double& y);
    void initialize_data();
    void print_to_QCP(QCustomPlot * plotter,
                      const bool& print_upper = true,
                      const bool& print_lower = true
        ) const;
    void optimize_sigmoid(const int& iterations);
    void optimize_upper_bound(const double& strength, const int& iterations);
    void optimize_lower_bound(const double& strength, const int& iterations);
protected:
    const int control_point_n;
    bool data_initialized;
    double bias_factor;
    std::vector<double> x_data, y_data, x_sig, y_sig, y_sig_upper, y_sig_lower;
    std::vector<std::pair<int,int> > data_per_segment;
    void optimize(const int& iterations = -1);
    double optimize_control_point(const int& i);
    void initialize_to_linear();
    /* double get_rms_error() const; */
    /* double bias_term() const; */
    /* double stiffness_term() const; */
};

#endif // ACTIVESIGMOIDOPTIMIZATION_H
