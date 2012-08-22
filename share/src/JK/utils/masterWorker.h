#ifndef _MASTERWORKER_H_
#define _MASTERWORKER_H_

#include <biros/biros.h>
#include <devTools/logging.h>
#include <MT/array.h>
#include <queue>

template<class Job, class Result> class sMaster;
template<class Job, class Result> class WorkerFactory;
template<class Result> class Integrator;

SET_LOG(masterworker, WARN);

template<class T> class Pool : public Variable {
  public:
    Pool<T>();
    std::queue<T> data; // Not a FIELD, since then we need operator<< for every T
};

template<class Job, class Result> class Worker : public Process {
  public:
    Worker<Job, Result>(const char* name);
    virtual ~Worker<Job, Result>() {};
    virtual void doWork(Result& r, const Job& j) = 0;

    virtual void open();
    virtual void step();
    virtual void close();

    Pool<Job>* jobs;
    Pool<Result>* results;
};

template<class Job, class Result> class Master  {
  protected:
    sMaster<Job, Result>* s;
  public:
    Master<Job, Result>(WorkerFactory<Job, Result>* factory, Integrator<Result> *i, const int numOfWorkers = 5);
    virtual ~Master<Job, Result>();

    //override if you want to use restart(Process*) instead of
    //restart(std::queue<Job>&, Process*)
    virtual int hasNextJob() { WARN(masterworker, "Don't use restart() without overriding hasNextJob()!"); return 0; }
    virtual Job createJob() { } ;

    //override if you want to see if all jobs are done
    virtual int hasWorkingJob() { WARN(masterworker, "hasWorkingJob() is not implemted. Override it!"); return 0; }

    virtual void pause();
    virtual void restart(Process* p);
    virtual void restart(std::queue<Job> &jobs, Process* p);

    Pool<Job>* jobs;
    Pool<Result>* results;
};

template<class Result> class Integrator : public Process {
  public:
    Integrator(const char *name) : Process(name) {};
    virtual void integrateResult(const Result& r) = 0;

    virtual void open();
    void step();
    virtual void close();
    virtual void restart();

    Pool<Result>* results;
};

template<class Job, class Result> class WorkerFactory {
  public:
    virtual ~WorkerFactory<Job, Result>() {};
    virtual Worker<Job, Result>* createWorker() = 0;  
};

#ifdef MT_IMPLEMENT_TEMPLATES
#include "masterWorker.cpp"
#endif

#endif
