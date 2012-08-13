#ifndef _TESTER_H_
#define _TESTER_H_

#include <biros/biros.h>
#include <biros/logging.h>
#include <JK/utils/sampler.h>
#include <MT/array.h>
#include <iostream>

SET_LOG(classify, DEBUG)

class ClassifyMaster;
class ClassificatorV;
class ClassifyData : public Variable {
  public:
    FIELD(int, numOfResults);
    FIELD(int, numOfJobs);
    FIELD(int, numOfJobsToStart);
    FIELD(int, numOfWorkingJobs);
    FIELD(double, sumOfCorrect);
    FIELD(double, result);
    ClassifyData() : Variable("Classify Data") {
      reg_numOfResults(); reg_numOfJobs(); reg_numOfWorkingJobs(); reg_sumOfCorrect(); reg_result(); 
    }
    ~ClassifyData() {};
};
template<class S> class Sampler;
class Tester {
  public:
    Tester(const int testNumber = 5000, const char* filename = "classification.data", int numOfWorkers=5, Sampler<MT::Array<arr> >* sampler = new BlocksWorldSampler);
    virtual ~Tester();
    const double test(ClassificatorV* l);

    ClassifyMaster* m;

    std::ofstream outfile;
};

#endif
