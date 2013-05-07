#ifndef CHI_SQUARE_TEST_H_
#define CHI_SQUARE_TEST_H_

#include <vector>

/** \example Chi_Square_Example.cpp This is an an example of how to use the
 * ChiSquareTest class. */

/** \brief This class provides tools for calculating the Chi-Square test. */
class ChiSquareTest {

public:
    /** \brief Calculate the Chi-Square statistic of two samples. */
    static double chi_square_statistic(
        const std::vector<double>& s1,
        const std::vector<double>& s2,
        bool sorted = false
        );
};

#endif /* CHI_SQUARE_TEST_H_ */
