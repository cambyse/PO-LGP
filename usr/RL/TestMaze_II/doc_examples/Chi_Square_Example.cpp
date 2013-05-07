#include "../util/ChiSquareTest.h"

#include <math.h>
#include <algorithm>

#define DEBUG_STRING ""
#include "../debug.h"

using std::sort;

int main(int, char **)
{

    srand48(time(nullptr));

    typedef std::vector<double> vec_t;

    //=======================//
    //  testing two samples  //
    //=======================//
    vec_t vec1, vec2;
    unsigned int n_bins = 5;

    DEBUG_OUT(0, "# N	n1	n2	Chi-Square");

    for(int i=0; i<1000; ++i) {
        double r = drand48();
        if(rand()%2==0) {
            vec1.push_back( floor(n_bins*r) );
        } else {
            vec2.push_back( floor( n_bins * (0.5*r + 0.5*pow(r,2)) ) );
        }
        if(i%1==0) {
            sort(vec1.begin(), vec1.end());
            sort(vec2.begin(), vec2.end());
            DEBUG_OUT(0,
                      i << "	" <<
                      vec1.size() << "	" <<
                      vec2.size() << "	" <<
                      ChiSquareTest::chi_square_statistic(vec1, vec2, true)
                );
        }
    }

    DEBUG_OUT(0, "");
    DEBUG_OUT(0, "");


    DEBUG_OUT(0, "# sample-1");
    DEBUG_OUT(0, "# value	index");
    for(unsigned int i=0; i<vec1.size(); ++i) {
        DEBUG_OUT(0, vec1[i] << "	" << i );
    }

    DEBUG_OUT(0, "");
    DEBUG_OUT(0, "");

    DEBUG_OUT(0, "# sample-2");
    DEBUG_OUT(0, "# value	index");
    for(unsigned int i=0; i<vec2.size(); ++i) {
        DEBUG_OUT(0, vec2[i] << "	" << i );
    }

    return 0;
}
