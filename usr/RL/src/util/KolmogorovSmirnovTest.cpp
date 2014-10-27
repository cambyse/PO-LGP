#include "KolmogorovSmirnovTest.h"

#include <algorithm>
#include <float.h>

#define DEBUG_STRING "K-S-Test: "
#include <util/debug.h>

using std::vector;

double KolmogorovSmirnovTest::k_s_statistic(
    const std::vector<double>& s1,
    const std::vector<double>& s2,
    bool sorted
    ) {

    // number of data points
    unsigned long n1 = s1.size();
    unsigned long n2 = s2.size();

    // return maximum possible if a sample is empty
    if(n1==0 || n2==0) {
        return 1;
    }

    // copy data only of not sorted
    const vector<double> *s1Ptr, *s2Ptr ;
    if(sorted) {
        s1Ptr = &s1;
        s2Ptr = &s2;
    } else {
        vector<double> *tmp_s1 = new vector<double>(s1);
        vector<double> *tmp_s2 = new vector<double>(s2);
        sort(tmp_s1->begin(), tmp_s1->end());
        sort(tmp_s2->begin(), tmp_s2->end());
        s1Ptr = tmp_s1;
        s2Ptr = tmp_s2;
    }

    // use reference to data
    const vector<double> &samples_1 = *s1Ptr;
    const vector<double> &samples_2 = *s2Ptr;

    // determine maximum difference between the two cumulative probability distributions
    auto samples_1_it = samples_1.begin();
    auto samples_2_it = samples_2.begin();
    double max_diff = -DBL_MAX;
    unsigned long s1_cumulative = 0, s2_cumulative = 0;
    while(samples_1_it!=samples_1.end() || samples_2_it!=samples_2.end()) {

        // determine next change point and update cumulative distributions
        if(samples_1_it!=samples_1.end() && samples_2_it!=samples_2.end()) { // both valid
            if(*samples_1_it<*samples_2_it) {
                ++s1_cumulative;
                ++samples_1_it;
            } else if(*samples_1_it>*samples_2_it) {
                ++s2_cumulative;
                ++samples_2_it;
            } else { // same value for both
                double value = *samples_1_it;
                if(value!=*samples_2_it) {
                    DEBUG_ERROR("Samples should be equal but really are not");
                }
                while(samples_1_it!=samples_1.end() && *samples_1_it==value) {
                    ++s1_cumulative;
                    ++samples_1_it;
                }
                while(samples_2_it!=samples_2.end() && *samples_2_it==value) {
                    ++s2_cumulative;
                    ++samples_2_it;
                }
            }
        } else if(samples_1_it!=samples_1.end()) { // only s1 valid
            ++s1_cumulative;
            ++samples_1_it;
        } else if(samples_2_it!=samples_2.end()) { // only s2 valid
            ++s2_cumulative;
            ++samples_2_it;
        } else {
            DEBUG_ERROR("Invalid iterators in Kolmogorov-Smirnov test");
        }

        // check difference
        if( fabs( (double)s1_cumulative/n1 - (double)s2_cumulative/n2 ) > max_diff ) {
            max_diff = fabs( (double)s1_cumulative/n1 - (double)s2_cumulative/n2 );
        }
    }

    // if data was copied to sort, delete copies
    if(!sorted) {
        delete s1Ptr;
        delete s2Ptr;
    }

    return max_diff;
}





double KolmogorovSmirnovTest::k_s_probability(const double& eff_alpha) {
    double prob = 0, crit1 = 0.001, crit2 = 1e-8;
    double a2 = -2*pow(eff_alpha,2);
    double sign = 2;
    double current_term, last_term = 0;
    for(int j=1; j<=100; ++j) {
        current_term = sign * exp( pow(j,2) * a2 );
        prob += current_term;
        if( fabs(current_term) <= crit1*last_term || fabs(current_term) <= crit2*prob) {
            return 1 - prob;
        }
        sign =- sign;
        last_term = abs(current_term);
    }
    return 0;
}

double KolmogorovSmirnovTest::k_s_test(
    const std::vector<double>& s1,
    const std::vector<double>& s2,
    bool sorted
    ) {

    // number of data points
    unsigned long n1 = s1.size();
    unsigned long n2 = s2.size();

    // get K-S statistic
    double ks_stat = k_s_statistic(s1,s2,sorted);

    double eff_n_sqrt = sqrt( (double)n1*n2/(n1+n2) );

    double eff_alpha = ( eff_n_sqrt + 0.12 + 0.11/eff_n_sqrt ) * ks_stat;

    return k_s_probability(eff_alpha);
}

double KolmogorovSmirnovTest::k_s_test(
    const double& ks_stat,
    unsigned long n1,
    unsigned long n2
    ) {
    double eff_n_sqrt = sqrt( (double)n1*n2/(n1+n2) );
    double eff_alpha = ( eff_n_sqrt + 0.12 + 0.11/eff_n_sqrt ) * ks_stat;
    return k_s_probability(eff_alpha);
}
