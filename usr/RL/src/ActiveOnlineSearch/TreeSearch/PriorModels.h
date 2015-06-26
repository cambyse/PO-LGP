#ifndef PRIORMODELS_H_
#define PRIORMODELS_H_

#include <limits>
#include <cmath>
#include <vector>

namespace prior_models {

    struct PriorCounts {
        //----members----//
        const double sum;
        const double squares_sum;
        const double counts;
        const double min;
        const double max;
        const double prior_counts;
        const double mean;
        const double variance;
        const double variance_of_mean;

        //----methods----//
    PriorCounts(double sum,
                double squares_sum,
                double counts,
                double min = 0,
                double max = 0,
                double prior_counts = 0);
        virtual ~PriorCounts() = default;
        static double compute_mean(double sum,
                                   double counts,
                                   double min,
                                   double max,
                                   double prior_counts);
        static double compute_variance(double mean,
                                       double squares_sum,
                                       double counts,
                                       double min,
                                       double max,
                                       double prior_counts);
    };

    struct Dirichlet {
        const double sum;
        const std::vector<double> mean;
        const std::vector<std::vector<double>> covariance;
        Dirichlet(std::vector<double> counts);
        static double compute_sum(std::vector<double> counts);
        static std::vector<double> compute_mean(std::vector<double> counts, double sum);
        static std::vector<std::vector<double>> compute_covariance(std::vector<double> counts, double sum);
    };

}; // end namespace prior_models

#endif /* PRIORMODELS_H_ */
