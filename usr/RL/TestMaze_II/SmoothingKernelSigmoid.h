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
    double print_to_QCP(QCustomPlot * plotter,
                        const bool& raw_data = true,
                        const bool& smooth_data = true,
                        const bool& bounds = true,
                        const bool& deviation = true) const;
protected:
    int raw_data_n;
    double kernel_width, gamma, min_x, max_x;
    std::vector<double> x_data, y_data;
    double kernel(const double& x1, const double& x2) const;
    void mean_dev(const double& x, double& mean, double& dev) const;
};

#endif // SMOOTHINGKERNELSIGMOID_H
