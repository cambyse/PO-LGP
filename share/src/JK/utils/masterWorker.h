#ifndef _MASTERWORKER_H_
#define _MASTERWORKER_H_

#include <biros/biros.h>
#include <devTools/logging.h>
#include <MT/array.h>
#include <queue>

template<class Job, class Result> class sMaster;
template<class Job, class Result> class WorkerFactory;
template<class Job, class Result> class Integrator;

SET_LOG(masterworker, WARN);


template<class Job, class Result> class Workspace : public Variable {
  public:
    Workspace<Job, Result>(const char* name) : Variable(name) { reg_working_jobs(); reg_jobs(); reg_results(); working_jobs = 0; };
    Workspace<Job, Result>();
    FIELD(MT::Array<Job>, jobs);
    FIELD(MT::Array<Result>, results);
    FIELD(int, working_jobs);
};

template<class Job, class Result> class Worker : public Process {
  public:
    Worker<Job, Result>(const char* name);
    virtual ~Worker<Job, Result>() {};
    virtual void doWork(Result& r, const Job& j) = 0;

    virtual void open();
    virtual void step();
    virtual void close();
   
    Workspace<Job, Result> *workspace;
};

template<class Job, class Result> class Master  {
  protected:
    sMaster<Job, Result>* s;
  public:
    Master<Job, Result>(WorkerFactory<Job, Result>* factory, Integrator<Job, Result> *i, const int numOfWorkers = 5);
    virtual ~Master<Job, Result>();

    //override if you want to use restart(Process*) instead of
    //restart(std::queue<Job>&, Process*)
    virtual int hasNextJob() { return workspace->get_jobs_to_do(NULL); }
    virtual Job createJob() { } ;

    //override if you want to see if all jobs are done
    virtual int hasWorkingJob() {  return workspace->get_working_jobs(NULL); }

    virtual void pause();
    virtual void restart(Process* p);
    virtual void restart(std::queue<Job> &jobs, Process* p);

    Workspace<Job, Result> *workspace;
};

template<class Job, class Result> class Integrator : public Process {
  public:
    Integrator(const char *name) : Process(name) {};
    virtual void integrateResult(const Result& r) = 0;

    virtual void open();
    void step();
    virtual void close();
    virtual void restart();

    Workspace<Job, Result>* workspace;
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
