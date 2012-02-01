#ifndef _MASTERWORKER_H_
#define _MASTERWORKER_H_

#include <MT/process.h>
#include <MT/array.h>
#include <queue>

template<class Job, class Result> class sMaster;
template<class Job, class Result> class WorkerFactory;

template<class T> class Pool : public Variable {
  public:
    Pool<T>();
    std::queue<T> data; 
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

template<class Job, class Result> class Master : public Process {
  private:
    sMaster<Job, Result>* s;
  public:
    Master<Job, Result>(const char* name, WorkerFactory<Job, Result>* factory, const int numOfWorkers = 5);
    virtual ~Master<Job, Result>();
    virtual int hasNextJob() = 0;
    virtual int hasWorkingJob() = 0;
    virtual Job createJob() = 0;
    virtual void integrateResult(const Result& r) = 0;

    virtual void open();
    virtual void step();
    virtual void close();

    virtual void pause();
    virtual void restart();

    Pool<Job>* jobs;
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
