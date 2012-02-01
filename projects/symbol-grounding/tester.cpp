#include "tester.h"

#include "masterWorker.h"
#include "activeLearner.h"
#include "oracle.h"
#include "sampler.h"
#include "activeLearningProcess.h"


#include <relational/blocksWorld.h>

#include <JK/util.h>

class ClassifyMaster : public Master<MT::Array<arr>, double> {
  private:
    int numOfResults;
    int numOfJobs;
    int numOfWorkingJobs;
    double sumOfCorrect;


    BlocksWorldSampler sampler;
  public:
    double _result;
    int testNumber;

    ClassifyMaster(int numOfJobs, ClassificatorV* cl);
    virtual MT::Array<arr> createJob();  
    virtual int hasNextJob();
    virtual int hasWorkingJob();
    virtual void integrateResult(const double& result);

    void reset(int jobs) { numOfJobs = jobs; numOfWorkingJobs = jobs; numOfResults = 0; sumOfCorrect = 0; }

    ClassificatorV* classificator;
};

class ClassifyWorker : public Worker<MT::Array<arr>, double> {
  public:
    ClassificatorV* classificator;
    ClassifyWorker(ClassificatorV*);
    virtual void doWork(double& result, const MT::Array<arr>& job);
};

class ClassifyWorkerFactory : public WorkerFactory<MT::Array<arr>, double> {
  public:
    ClassifyWorkerFactory(ClassificatorV* cl);
    ClassificatorV* classificator;
    virtual Worker<MT::Array<arr>, double>* createWorker();
};

ClassifyWorkerFactory::ClassifyWorkerFactory(ClassificatorV* cl) :
  classificator(cl) {}

ClassifyMaster::ClassifyMaster(int numOfJobs, ClassificatorV* cl) : 
  Master<MT::Array<arr>, double>("Classify Master Process", (WorkerFactory<MT::Array<arr>, double>*) new ClassifyWorkerFactory(cl)) ,
  numOfJobs(numOfJobs),
  numOfWorkingJobs(numOfJobs),
  classificator(cl)
{}

MT::Array<arr> ClassifyMaster::createJob() {
  MT::Array<arr> sample;
  sampler.sample(sample);
  return sample;
}

int ClassifyMaster::hasNextJob() {
  return numOfJobs--;  
}

int ClassifyMaster::hasWorkingJob() {
  return numOfWorkingJobs;  
}

void ClassifyMaster::integrateResult(const double& result) {
  numOfResults++;
  numOfWorkingJobs--;
  sumOfCorrect += result;
  _result = sumOfCorrect/numOfResults;
}

ClassifyWorker::ClassifyWorker(ClassificatorV* cl) :
  Worker<MT::Array<arr>, double>("Classify Worker"),
  classificator(cl)
{}

void ClassifyWorker::doWork(double& result, const MT::Array<arr>& job) {
  classificator->readAccess(this);
  if (classificator->classificator->classify(job) == classificator->oracle->classify(job)) result = 1;
  else result = 0;
  classificator->deAccess(this);
}

Worker<MT::Array<arr>, double>* ClassifyWorkerFactory::createWorker() {
  return new ClassifyWorker(classificator);
}

Tester::Tester(const int testNumber) : m(NULL) {
  m = new ClassifyMaster(testNumber, new ClassificatorV);
  m->testNumber = testNumber;
  outfile.open("classification.data");
  m->threadOpen();
}

const double Tester::test(ClassificatorV* l) {
  m->classificator->writeAccess(NULL);
  m->classificator->classificator = l->classificator;
  m->classificator->oracle = l->oracle;
  m->classificator->deAccess(NULL);
  m->reset(m->testNumber);
  m->restart();
  while(m->hasWorkingJob()) sleep(1);
  m->pause();
  outfile << m->_result << std::endl;
  return m->_result;
}

Tester::~Tester() {
  outfile.close();
  m->threadStop();
  m->threadClose();
  delete m;
}
