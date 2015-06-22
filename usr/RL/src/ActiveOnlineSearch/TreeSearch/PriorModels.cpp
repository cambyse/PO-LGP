#include "PriorModels.h"

#include <util/util.h>

using std::vector;

namespace prior_models {

    PriorCounts::PriorCounts(double sum,
                             double squares_sum,
                             double counts,
                             double min,
                             double max,
                             double prior_counts):
        sum(sum),
        squares_sum(squares_sum),
        counts(counts),
        min(min),
        max(max),
        prior_counts(prior_counts),
        mean(compute_mean(sum,counts,min,max,prior_counts)),
        variance(compute_variance(mean,squares_sum,counts,min,max,prior_counts))
    {}

    double PriorCounts::compute_mean(double sum,
                                     double counts,
                                     double min,
                                     double max,
                                     double prior_counts) {
        return (sum + min*prior_counts/2 + max*prior_counts/2) / (counts + prior_counts);
    }

    double PriorCounts::compute_variance(double mean,
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

    Dirichlet::Dirichlet(vector<double> counts):
        sum(compute_sum(counts)),
        mean(compute_mean(counts, sum)),
        covariance(compute_covariance(counts, sum))
    {}

    inline double Dirichlet::compute_sum(vector<double> counts) {
        return util::sum(counts);
    }

    inline vector<double> Dirichlet::compute_mean(vector<double> counts, double sum) {
        vector<double> mean = counts;
        for(auto & m : mean) m /= sum;
        return mean;
    }

    inline vector<vector<double>> Dirichlet::compute_covariance(vector<double> counts, double sum) {
        int size = counts.size();
        vector<vector<double>> covariance(size,vector<double>(size,1./(sum*sum*(sum+1))));
        for(int idx_1=0; idx_1<size; ++idx_1) {
            for(int idx_2=0; idx_2<size; ++idx_2) {
                if(idx_1==idx_2) {
                    covariance[idx_1][idx_2] *= counts[idx_1]*(sum-counts[idx_1]);
                } else {
                    covariance[idx_1][idx_2] *= -counts[idx_1]*counts[idx_2];
                }
            }
        }
        return covariance;
    }

}; // end namespace prior_models
