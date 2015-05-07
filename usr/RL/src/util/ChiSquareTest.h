#ifndef CHI_SQUARE_TEST_H_
#define CHI_SQUARE_TEST_H_

#include <vector>
#include <algorithm>
#include <float.h>

#define DEBUG_STRING "Chi-Square: "
#include "debug.h"

/** \example Chi_Square_Example.cpp This is an an example of how to use the
 * ChiSquareTest class. */

/** \brief This class provides tools for calculating the Chi-Square test. */
class ChiSquareTest {

public:

    /** \brief Calculate the Chi-Square statistic of two samples. */
    template < class T >
        static double chi_square_statistic(
            const std::vector<T>& s1,
            const std::vector<T>& s2,
            bool sorted = false
            );

    /* /\** \brief Calculate the Chi-Square statistic of two samples (overloaded */
    /*  * version for doubles).*\/ */
    /* static double chi_square_statistic( */
    /*     const std::vector<double>& s1, */
    /*     const std::vector<double>& s2, */
    /*     bool sorted = false */
    /*     ); */
};

template < class T >
double ChiSquareTest::chi_square_statistic(
    const std::vector<T>& s1,
    const std::vector<T>& s2,
    bool sorted
    ) {

    // number of data points
    unsigned long n1 = s1.size();
    unsigned long n2 = s2.size();

    // return maximum possible if both samples are empty
    if(n1==0 && n2==0) {
        return DBL_MAX;
    }

    // copy data only of not sorted
    const std::vector<T> *s1Ptr, *s2Ptr ;
    if(sorted) {
        s1Ptr = &s1;
        s2Ptr = &s2;
    } else {
        std::vector<T> *tmp_s1 = new std::vector<T>(s1);
        std::vector<T> *tmp_s2 = new std::vector<T>(s2);
        sort(tmp_s1->begin(), tmp_s1->end());
        sort(tmp_s2->begin(), tmp_s2->end());
        s1Ptr = tmp_s1;
        s2Ptr = tmp_s2;
    }

    // use reference to data
    const std::vector<T> &samples_1 = *s1Ptr;
    const std::vector<T> &samples_2 = *s2Ptr;

    std::vector<T> bin_values;
    std::vector<int> s1_counts, s2_counts;

    // create bins and count hits
    auto samples_1_it = samples_1.begin();
    auto samples_2_it = samples_2.begin();
    while(samples_1_it!=samples_1.end() || samples_2_it!=samples_2.end()) {

        // determine next value and update counts
        T current_value;
        int s1_hits = 0, s2_hits = 0;
        if(samples_1_it!=samples_1.end() && samples_2_it!=samples_2.end()) { // both valid
            if(*samples_1_it<*samples_2_it) {
                current_value = *samples_1_it;
                s1_hits = 1;
                ++samples_1_it;
            } else if(*samples_1_it>*samples_2_it) {
                current_value = *samples_2_it;
                s2_hits = 1;
                ++samples_2_it;
            } else { // same value for both
                current_value = *samples_1_it;
                if(current_value!=*samples_2_it) {
                    DEBUG_OUT(1,"Error: Samples should be equal but really are not");
                }
                while(samples_1_it!=samples_1.end() && *samples_1_it==current_value) {
                    ++s1_hits;
                    ++samples_1_it;
                }
                while(samples_2_it!=samples_2.end() && *samples_2_it==current_value) {
                    ++s2_hits;
                    ++samples_2_it;
                }
            }
        } else if(samples_1_it!=samples_1.end()) { // only s1 valid
            current_value = *samples_1_it;
            ++s1_hits;
            ++samples_1_it;
        } else if(samples_2_it!=samples_2.end()) { // only s2 valid
            current_value = *samples_2_it;
            ++s2_hits;
            ++samples_2_it;
        } else {
            DEBUG_OUT(1,"Error: Invalid iterators in Kolmogorov-Smirnov test");
        }

        // apply
        if(bin_values.size()==0 || bin_values.back()!=current_value) {
            bin_values.push_back(current_value);
            s1_counts.push_back(s1_hits);
            s2_counts.push_back(s2_hits);
        } else {
            s1_counts.back() += s1_hits;
            s2_counts.back() += s2_hits;
        }

    }

    // perform summation
    double chi_square = 0;
    DEBUG_OUT(1,"Performing summation:");
    for(unsigned int idx=0; idx<bin_values.size(); ++idx) {
        double term = pow((double)(s1_counts[idx] - s2_counts[idx]), 2) / (s1_counts[idx] + s2_counts[idx]);
        DEBUG_OUT(1,"    += " << term << " (" << s1_counts[idx] << "," << s2_counts[idx] << ")");
        chi_square += term;
    }
    DEBUG_OUT(1,bin_values.size() << " bins");

    // if data was copied to sort, delete copies
    if(!sorted) {
        delete s1Ptr;
        delete s2Ptr;
    }

    return chi_square;
}

#include "debug_exclude.h"

#endif /* CHI_SQUARE_TEST_H_ */
