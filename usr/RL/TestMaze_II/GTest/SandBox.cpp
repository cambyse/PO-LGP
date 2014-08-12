#include "SandBox.h"

#include <gtest/gtest.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <vector>
#include <QString>

#include "../util/util.h"
#include "../util/QtUtil.h"

#include "../util/debug.h"

using std::vector;
using std::cout;
using std::endl;

TEST(SandBox, SomeTest) {

    int i, n = 10;

    // set up random generator
    gsl_rng * rand_gen = gsl_rng_alloc(gsl_rng_default);
    gsl_rng_set(rand_gen, time(nullptr));

    static const int N = 3;
    double alpha[N];
    double theta[N];
    for(int idx : util::Range(N)) {
        alpha[idx] = 0.01;
    }

    for (i = 0; i < n; i++) {
        gsl_ran_dirichlet(rand_gen, N, alpha, theta);
        for(int idx : util::Range(N)) {
            cout << QString("%1 ").arg(theta[idx],7,'f',4);
        }
        cout << endl;
    }

    // clean up
    gsl_rng_free(rand_gen);
}
