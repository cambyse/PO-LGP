#ifndef _MASTERWORKER_H_
#define _MASTERWORKER_H_

#include <biros/biros.h>
#include <MT/array.h>
#include <queue>

template<class Job, class Result> class sMaster;
template<class Job, class Result> class WorkerFactory;
template<class Result> class Integrator;

template<class T> class Pool : public Variable {
  public:
    Pool<T>();
    FIELD(std::queue<T>, data); 
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
    virtual int hasNextJob();
    virtual Job createJob();

    //override if you want to see if all jobs are done
    virtual int hasWorkingJob();

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

    void open();
    void step();
    void close();

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
