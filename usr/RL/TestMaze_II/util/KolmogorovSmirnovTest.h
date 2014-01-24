#ifndef KOLMOGOROV_SMIRNOV_TEST_H_
#define KOLMOGOROV_SMIRNOV_TEST_H_

#include <vector>

/** \example Kolmogorov_Smirnov_Example.cpp This is an an example of how to use
 * the KolmogorovSmirnovTest class. */

/** \brief This class provides tools for calculating the Kolmogorov–Smirnov test.
 *
 * The Kolmogorov–Smirnov test is a measure of how much two one-dimensional
 * probability density distributions differ. k_s_statistic computes the
 * Kolmogorov–Smirnov statistic of two samples while k_s_probability
 * approximates the cumulative Kolmogorov–Smirnov distribution. k_s_test uses
 * these two functions to compute the probability of two samples to originate
 * from two different probability distributions.
 *
 * My implementation follows the instructions from this <a
 * href="../doc_files/k-s-approx.pdf" target="_blank"><b>Paper</b></a>.
 *
 * See */

class KolmogorovSmirnovTest {

public:
    /** \brief Calculate the Kolmogorov–Smirnov statistic of two samples. */
    static double k_s_statistic(
        const std::vector<double>& s1,
        const std::vector<double>& s2,
        bool sorted = false
        );

    /** \brief Calculate approximate K-S-Distribution.
     *
     * This function approximates the following sum \f[
     *
     * Q_{KS}(\alpha) = \sum_{j=1}^{\infty} 2 (-1)^{j-1} \exp(-2 j^2 \alpha^2)
     *
     * \f] by consecutively evaluating the first 100 terms inside the sum,
     * returning if either the absolute value of current term is below 1e-8 or
     * the absolute value of the quotient of the current term divided by the
     * last term is below 0.001. If the function does not converge within the
     * first 100 terms it returns 1 instead. */
    static double k_s_probability(const double& eff_alpha);

    static double k_s_test(
        const std::vector<double>& s1,
        const std::vector<double>& s2,
        bool sorted = false
        );

    static double k_s_test(
        const double& ks_stat,
        unsigned long n1,
        unsigned long n2
        );
};

#endif /* KOLMOGOROV_SMIRNOV_TEST_H_ */
