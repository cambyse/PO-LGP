#ifndef SOFT_MAX_H_
#define SOFT_MAX_H_

#include "debug.h"

namespace util {

    /** Implements the SoftMax function. Given input vector \f$u\f$ and
     * temperature \f$T\f$ the return vector \f$v\f$ is computed as \f$v_{i} =
     * \frac{\exp\left[u_{i}/T\right]}{\sum_{j}\exp\left[u_{j}/T\right]}\f$. */
    template < typename Vec >
        Vec soft_max(const Vec& vec, double temperature) {
        DEBUG_EXPECT(temperature>=0);
        if(vec.size()==0) {
            Vec ret = vec;
            return ret;
        }
        //----------------------------------//
        // low-temperature / hard-max limit //
        //----------------------------------//
        if(temperature==0) {
            double max = std::numeric_limits<double>::lowest();
            double max_counts = 0;
            for(auto value : vec) {
                if(value>max) {
                    max_counts = 1;
                    max = value;
                } else if(value==max) {
                    ++max_counts;
                }
            }
            DEBUG_EXPECT(max_counts>0);
            Vec ret = vec;
            for(int idx=0; idx<(int)vec.size(); ++idx) {
                if(vec[idx]==max) {
                    ret[idx] = 1./max_counts;
                } else {
                    ret[idx] = 0;
                }
            }
            return ret;
        }
        //----------------------------------//
        // high-temperature / uniform limit //
        //----------------------------------//
        if(isinf(temperature)) {
            Vec ret = vec;
            int size = vec.size();
            for(auto & v : ret) v = 1./size;
            return ret;
        }
        //-----------------------------------------------------------------------//
        // Normal computation: We need to check for inf values and handle them
        // separately. Also, overflow may occur for large values, which may be
        // avoided by shifting all values such that the maximum value is zero
        // (underflow for non-maximum values is ok since those terms drop out
        // anyway). Note that shifting values by a constant does not affect the
        // result.
        // -----------------------------------------------------------------------//
        // compute max and check for (positive) inf values
        double max = std::numeric_limits<double>::lowest();
        int positive_inf_values = 0;
        for(int idx=0; idx<(int)vec.size(); ++idx) {
            max = std::max(max,vec[idx]);
            if(isinf(vec[idx]) && vec[idx]>0) ++positive_inf_values;
        }
        // assign uniform weights to inf entries, if such entries exist, or
        // compute unnormalized weights and normalization otherwise
        Vec ret = vec;
        double normalization = 0;
        for(int idx=0; idx<(int)vec.size(); ++idx) {
            if(positive_inf_values) {
                if(isinf(vec[idx]) && vec[idx]>0)
                    ret[idx] = 1./positive_inf_values;
                else
                    ret[idx] = 0;
            } else {
                double weight = exp((vec[idx]-max)/temperature);
                ret[idx] = weight;
                normalization += weight;
            }
        }
        // renormalize weights (only if no inf values existed)
        if(!positive_inf_values)
            for(int idx=0; idx<(int)vec.size(); ++idx)
                ret[idx] /= normalization;
        // debug check
        IF_DEBUG(0) {
            double weight_sum = 0;
            for(auto & w : ret) weight_sum+=w;
            DEBUG_EXPECT_APPROX(weight_sum,1);
            if(fabs(weight_sum-1)>1e-10) {
                DEBUG_WARNING(positive_inf_values);
                DEBUG_WARNING(ret);
            }
        }
        return ret;
    }

} // end namespace util

#include "debug_exclude.h"

#endif /* SOFT_MAX_H_ */
