#ifndef ACTIVESIGMOIDOPTIMIZATION_H
#define ACTIVESIGMOIDOPTIMIZATION_H

#include <vector>

class ActiveSigmoidOptimization {

public:

    struct Point;
    typedef std::vector<Point> data_t;

    ActiveSigmoidOptimization();

    ~ActiveSigmoidOptimization();

    void add_new_point(const double& x, const double& y);

    void print_curve_to_file(const char* file_name, const double& x_min, const double& x_max, const unsigned int& points = 1000);
    void print_curve_to_file(const char* file_name, const unsigned int& points = 1000);

    double f(const double& x);

protected:

    bool data_sorted;
    data_t data;

    void sort_data();

    void find_enclosing_points(const double& x, int& idx_1, int& idx_2);

};

#endif // ACTIVESIGMOIDOPTIMIZATION_H
