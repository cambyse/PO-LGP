#ifndef SMOOTHINGKERNELSIGMOID_H
#define SMOOTHINGKERNELSIGMOID_H

#include <vector>

class QCustomPlot;

class SmoothingKernelSigmoid {

public:
    SmoothingKernelSigmoid(const double& width,
                           const double& g,
                           const double& min,
                           const double& max);
    ~SmoothingKernelSigmoid();
    void add_new_point(const double& x, const double& y);
    double get_max_uncertain(const int& sample_n = 1000) const;
    void print_to_QCP(QCustomPlot * plotter,
                      const bool& raw_data = true,
                      const bool& smooth_data = true,
                      const bool& bounds = true,
                      const bool& deviation = true,
                      const bool& uncert = true) const;
    void mean_dev_weights(const double& x,
                          double& mean,
                          double& mean_dev,
                          double& dev) const;
    void mean_dev_weights(const double& x, double& mean) const {
        double d1, d2;
        mean_dev_weights(x, mean, d1, d2);
    }
protected:
    int raw_data_n;
    double kernel_width, gamma, min_x, max_x;
    std::vector<double> x_data, y_data;
    double kernel(const double& x1, const double& x2) const;
};

#endif // SMOOTHINGKERNELSIGMOID_H
