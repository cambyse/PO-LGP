#include "SandBox.h"

#include <gtest/gtest.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <vector>
#include <QString>

#include <omp.h>

#include "../util/util.h"
#include "../util/QtUtil.h"

#include "../util/debug.h"

using std::vector;
using std::cout;
using std::endl;

TEST(SandBox, SomeTest) {

    //omp_set_num_threads(10);
    //omp_set_nested(1);
    int counter = 0;
#pragma omp parallel
    {
        int first_thread_num = omp_get_thread_num();
        int first_num_threads = omp_get_num_threads();
#pragma omp parallel
        {
            int second_thread_num = omp_get_thread_num();
            int second_num_threads = omp_get_num_threads();
#pragma omp parallel
            {
                int third_thread_num = omp_get_thread_num();
                int third_num_threads = omp_get_num_threads();
#pragma omp critical
                {
                    cout << "count: " << counter
                         << "	first: " << first_thread_num << "/" << first_num_threads
                         << "	second: " << second_thread_num << "/" << second_num_threads
                         << "	third: " << third_thread_num << "/" << third_num_threads << endl;
                    ++counter;
                }
            }
        }
    }

}
