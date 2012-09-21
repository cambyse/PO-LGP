#include "al_tester.h"
#include "al.h"
#include "al_process.h"

#define LOG_STRING (level == ERROR ? std::cerr : std::cout) << "[@" << file << ":" << line << " | " << name << " | " << msg << " ]" << std::endl; 

#include <JK/utils/masterWorker.h>
#include <JK/utils/oracle.h>
#include <JK/utils/sampler.h>
#include <biros/logging.h>

#include <relational/robotManipulationSimulator.h>
#if 0
class ClassifyMaster : public Master<MT::Array<arr>, double> {
  public:
    int testNumber;

    ClassifyMaster(int numOfJobs, ClassificatorV* cl, int numOfWorkers);
    virtual MT::Array<arr> createJob();  
    virtual int hasNextJob();
    virtual int hasWorkingJob();

    ClassificatorV* classificator;
    ClassifyData *data;
};

class ClassifyIntegrator : public Integrator<double> {
  public:
    ClassifyIntegrator(int num_of_jobs) : Integrator<double>("Classify Integrator"){
      birosInfo().getVariable<ClassifyData>(data, "Classify Data", this);
    }
    virtual void integrateResult(const double& r);
    virtual void restart() { data->writeAccess(this); data->numOfJobs=data->numOfJobsToStart; data->numOfWorkingJobs = data->numOfJobsToStart; data->numOfResults = 0; data->sumOfCorrect = 0; data->deAccess(this); };
    ClassifyData *data;
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

void ClassifyIntegrator::integrateResult(const double& result) {
  data->writeAccess(this);
  data->numOfResults++;
  data->numOfWorkingJobs--;
  data->sumOfCorrect += result;
  data->result = data->sumOfCorrect/data->numOfResults;
  data->deAccess(this);
}

ClassifyWorkerFactory::ClassifyWorkerFactory(ClassificatorV* cl) :
  classificator(cl) {}

ClassifyMaster::ClassifyMaster(int numOfJobs, ClassificatorV* cl, int numOfWorkers) : 
  Master<MT::Array<arr>, double>((WorkerFactory<MT::Array<arr>, double>*) new ClassifyWorkerFactory(cl), (Integrator<double> *) new ClassifyIntegrator(numOfJobs), numOfWorkers) ,
  classificator(cl)
{
  birosInfo().getVariable<ClassifyData>(data, "Classify Data", NULL);
  data->writeAccess(NULL);
  data->numOfJobs = numOfJobs;
  data->numOfWorkingJobs = numOfJobs;
  data->numOfJobsToStart= numOfJobs;
  data->deAccess(NULL);
}

MT::Array<arr> ClassifyMaster::createJob() {
  MT::Array<arr> sample;
  classificator->classificator->problem.sampler->sample(sample);
  return sample;
}

int ClassifyMaster::hasNextJob() {
  int n = data->get_numOfJobs(NULL);
  data->set_numOfJobs(n-1, NULL);
  return n;  
}

int ClassifyMaster::hasWorkingJob() {
  return data->get_numOfWorkingJobs(NULL);  
}

ClassifyWorker::ClassifyWorker(ClassificatorV* cl) :
  Worker<MT::Array<arr>, double>("Classify Worker"),
  classificator(cl)
{}

void ClassifyWorker::doWork(double& result, const MT::Array<arr>& job) {
  classificator->readAccess(this);
  if (classificator->classificator->classify(job) == classificator->classificator->problem.oracle->classify(job)) result = 1;
  else result = 0;
  classificator->deAccess(this);
}

Worker<MT::Array<arr>, double>* ClassifyWorkerFactory::createWorker() {
  return new ClassifyWorker(classificator);
}

Tester::Tester(const int testNumber, const char* filename, int numOfWorkers, Sampler<MT::Array<arr> >* sampler) : m(NULL) {
  m = new ClassifyMaster(testNumber, new ClassificatorV, numOfWorkers);
  ClassifyData *d = new ClassifyData();
  m->testNumber = testNumber;
  outfile.open(filename);
}

const double Tester::test(ClassificatorV* l) {
  m->classificator->writeAccess(NULL);
  m->classificator->classificator = l->classificator;
  m->classificator->deAccess(NULL);
  m->restart(NULL);
  while(m->hasWorkingJob()) sleep(1);
  m->pause();
  ClassifyData *d = m->data;
  d->readAccess(NULL);
  outfile << d->result << std::endl;
	cout << d->result << std::endl;
  double r = d->result;
  d->deAccess(NULL);
  return r;
}

Tester::~Tester() {
  outfile.close();
  ClassifyData *d;
  birosInfo().getVariable<ClassifyData>(d, "Classify Data", NULL);
  delete d;
  m->pause();
  delete m;
}
#endif
