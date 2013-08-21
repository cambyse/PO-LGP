#ifndef SPLINEINTERPOLATION_H
#define SPLINEINTERPOLATION_H

#include <vector>

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
protected:
    static void find_enclosing_points(const std::vector<double>& x_data,
                                      const double& x_querry,
                                      int& idx_1,
                                      int& idx_2
        );
};

#endif // SPLINEINTERPOLATION_H
