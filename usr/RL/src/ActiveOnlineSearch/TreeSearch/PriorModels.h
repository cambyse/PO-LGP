#ifndef PRIORMODELS_H_
#define PRIORMODELS_H_

#include <limits.h>

namespace prior_models {

    class PriorCounts {
        //----typedefs/classes----//

        //----members----//
    public:
        const double sum;
        const double squares_sum;
        const double counts;
        const double min;
        const double max;
        const double prior_counts;
        const double mean;
        const double variance;

        //----methods----//
    public:
    PriorCounts(double sum,
                double squares_sum,
                double counts,
                double min = 0,
                double max = 0,
                double prior_counts = 0):
        sum(sum),
            squares_sum(squares_sum),
            counts(counts),
            min(min),
            max(max),
            prior_counts(prior_counts),
            mean(compute_mean(sum,counts,min,max,prior_counts)),
            variance(compute_variance(mean,squares_sum,counts,min,max,prior_counts))
            {}
        virtual ~PriorCounts() = default;
        static double compute_mean(double sum,
                                   double counts,
                                   double min,
                                   double max,
                                   double prior_counts) {
            return (sum + min*prior_counts/2 + max*prior_counts/2) / (counts + prior_counts);
        }
        static double compute_variance(double mean,
                                       double squares_sum,
                                       double counts,
                                       double min,
                                       double max,
                                       double prior_counts) {
            if(counts + prior_counts - 1 <= 0) {
                return std::numeric_limits<double>::infinity();
            } else {
                double var = (squares_sum +
                              pow(min,2)*prior_counts/2 +
                              pow(max,2)*prior_counts/2);
                var /= (counts + prior_counts);
                var -= pow(mean,2);
                return (counts + prior_counts)/(counts + prior_counts - 1)*var;
            }
        }
    };

}; // end namespace prior_models

#endif /* PRIORMODELS_H_ */
