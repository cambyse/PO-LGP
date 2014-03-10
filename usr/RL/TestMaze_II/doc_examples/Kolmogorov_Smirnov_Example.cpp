#include "../util/KolmogorovSmirnovTest.h"

#include <math.h>
#include <algorithm>

#define DEBUG_STRING ""
#include "../debug.h"

using std::sort;

int main(int, char **)
{
    typedef std::vector<double> vec_t;

    //=======================//
    //    K-S distribution   //
    //=======================//

    DEBUG_OUT(0, "# alpha	Qks");

    for(double a=0; a<3; a+=0.001) {
        DEBUG_OUT(0,
                  a << "	" <<
                  KolmogorovSmirnovTest::k_s_probability(a)
            );
    }

    DEBUG_OUT(0, "");
    DEBUG_OUT(0, "");

    //=======================//
    //  testing two samples  //
    //=======================//
    vec_t vec1, vec2;

    DEBUG_OUT(0, "# N	n1	n2	Dks	alpha");

    for(int i=0; i<200000; ++i) {
        double r = drand48();
        if(rand()%2==0) {
            vec1.push_back(r);
        } else {
            vec2.push_back(0.98*r + 0.02*pow(r,2));
        }
        if(i%1000==0) {
            sort(vec1.begin(), vec1.end());
            sort(vec2.begin(), vec2.end());
            double ks_stat = KolmogorovSmirnovTest::k_s_statistic(vec1,vec2,true);
            DEBUG_OUT(0,
                      i << "	" <<
                      vec1.size() << "	" <<
                      vec2.size() << "	" <<
                      ks_stat << "	" <<
                      KolmogorovSmirnovTest::k_s_test(ks_stat, vec1.size(), vec2.size())
                );
        }
    }

    return 0;
}
