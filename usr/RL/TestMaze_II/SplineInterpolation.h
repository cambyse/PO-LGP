#ifndef SPLINEINTERPOLATION_H
#define SPLINEINTERPOLATION_H

#include <vector>

class QCustomPlot; // forward declaration

class SplineInterpolation {
public:
    SplineInterpolation();
    ~SplineInterpolation();
    static double f(const std::vector<double>& x_data,
                    const std::vector<double>& y_data,
                    const double& x_querry
        );
    static void print_curve_to_file(const std::vector<double>& x_data,
                                    const std::vector<double>& y_data,
                                    const char* file_name,
                                    const double& x_min,
                                    const double& x_max,
                                    const unsigned int& points = 1000);
    static void print_curve_to_file(const std::vector<double>& x_data,
                                    const std::vector<double>& y_data,
                                    const char* file_name,
                                    const unsigned int& points = 1000);
    static void print_curve_to_QCP(const std::vector<double>& x_data,
                                   const std::vector<double>& y_data,
                                   QCustomPlot * plotter,
                                   const unsigned int& interp_n = 1000);
protected:
    static void find_enclosing_points(const std::vector<double>& x_data,
                                      const double& x_querry,
                                      int& idx_1,
                                      int& idx_2
        );
};

#endif // SPLINEINTERPOLATION_H
