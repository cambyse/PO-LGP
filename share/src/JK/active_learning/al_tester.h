#ifndef _TESTER_H_
#define _TESTER_H_

#include <MT/array.h>
#include <iostream>

class ClassifyMaster;
class ClassificatorV;
template<class S> class Sampler;
class Tester {
  public:
    Tester(const int testNumber = 5000, const char* filename = "classification.data");
    virtual ~Tester();
    const double test(ClassificatorV* l);

    ClassifyMaster* m;
    Sampler<MT::Array<arr> >* sampler;

    std::ofstream outfile;
};

#endif
