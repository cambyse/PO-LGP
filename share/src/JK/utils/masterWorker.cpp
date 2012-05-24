#ifndef _MASTERWORKER_CPP_
#define _MASTERWORKER_CPP_

#include "masterWorker.h"

#include <queue>
#include <JK/utils/util.h>


template<class Job, class Result> class sMaster {
  public:
    std::vector<Worker<Job, Result>* > workers;
    WorkerFactory<Job, Result>* workerFactory;
    uint numOfWorkers;
    Integrator<Result> *integrator;

    void restartWorker();
    bool paused;
};

template<class Job, class Result> Worker<Job, Result>::Worker(const char* name) : Process(name) {}


template<class Job, class Result> void Worker<Job, Result>::open() {
}


template<class Job, class Result> void Worker<Job, Result>::step() {
  Job j;
  jobs->writeAccess(this);
  if (!jobs->data.empty()) {
    j = jobs->data.front();
    jobs->data.pop();
    jobs->deAccess(this);
    Result r;
    doWork(r, j);
    results->writeAccess(this);
    results->data.push(r);
    results->deAccess(this);
  }
  else {
    jobs->deAccess(this);  
  }
}

template<class Job, class Result> void Worker<Job, Result>::close() {
    
}

template<class Job, class Result> Master<Job, Result>::Master(WorkerFactory<Job, Result>* fac, Integrator<Result> *i, const int numOfWorkers) {
  s = new sMaster<Job, Result>;
  s->workerFactory = fac;
  s->numOfWorkers = numOfWorkers; 
  s->integrator = i;

  jobs = new Pool<Job>();
  results = new Pool<Result>();

  s->paused = true;
  
  for (uint i = 0; i < s->numOfWorkers; ++i) {
    Worker<Job, Result>* w = s->workerFactory->createWorker();
    s->workers.push_back(w);
    w->jobs = jobs;
    w->results = results;
    w->threadOpen();
  }
  s->integrator->results = results;
  s->integrator->threadOpen();

}

template<class Job, class Result> void Master<Job, Result>::pause() {
  if (s->paused) return;
  s->paused = true;
  typename std::vector<Worker<Job, Result>* >::iterator w;
  for (w = s->workers.begin(); w != s->workers.end(); w++) {
    (*w)->threadStop();
  }
  s->integrator->threadStop();
}

template<class Job, class Result> void Master<Job, Result>::restart(Process *p) {
  if (!s->paused) return;
  s->paused = false;
  while (hasNextJob()) {
    Job j = createJob();
    jobs->writeAccess(p);
    jobs->data.push(j);
    jobs->deAccess(p);
  }
  s->restartWorker();
  s->integrator->restart();
  s->integrator->threadLoop();

}
template<class Job, class Result> void Master<Job, Result>::restart(std::queue<Job> &jobs, Process *p) {
  if (!s->paused) return;
  s->paused = false;
  this->jobs->writeAccess(p);
  this->jobs->data = jobs;
  this->jobs->deAccess(p);
  s->restartWorker();
  s->integrator->restart();
  s->integrator->threadLoop();
}
  

template<class Job, class Result> void sMaster<Job, Result>::restartWorker() {
  typename std::vector<Worker<Job, Result>* >::iterator w;
  for (w = workers.begin(); w != workers.end(); w++) {
    (*w)->threadLoop();
  }
}

template<class Result> void Integrator<Result>::open() {
}

template<class Result> void Integrator<Result>::restart() {
}

template<class Result> void Integrator<Result>::step() {
  results->writeAccess(this);
  if (!results->data.empty()) {
    integrateResult(results->data.front());
    results->data.pop();
  }
  results->deAccess(this);
}

template<class Result> void Integrator<Result>::close() {
}

template<class Job, class Result> Master<Job, Result>::~Master() {
  for (uint i = 0; i < s->numOfWorkers; ++i) {
     Worker<Job, Result>* w = s->workers.back();
     w->threadStop();
     w->threadClose();
     s->workers.pop_back();
  }
}

template<class T> Pool<T>::Pool() : Variable("Pool Variable") { }

#endif
