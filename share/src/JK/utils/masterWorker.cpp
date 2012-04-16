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

template<class Job, class Result> Master<Job, Result>::Master(const char* name, WorkerFactory<Job, Result>* fac, const int numOfWorkers) : Process(name) {
  s = new sMaster<Job, Result>;
  s->workerFactory = fac;
  s->numOfWorkers = numOfWorkers; 

  jobs = new Pool<Job>();
  results = new Pool<Result>();
  
  for (uint i = 0; i < s->numOfWorkers; ++i) {
    Worker<Job, Result>* w = s->workerFactory->createWorker();
    s->workers.push_back(w);
    w->jobs = jobs;
    w->results = results;
    w->threadOpen();
  }
}

template<class Job, class Result> void Master<Job, Result>::pause() {
  if (s->paused) return;
  s->paused = true;
  typename std::vector<Worker<Job, Result>* >::iterator w;
  for (w = s->workers.begin(); w != s->workers.end(); w++) {
    (*w)->threadStop();
  }
  this->threadStop();
}

template<class Job, class Result> void Master<Job, Result>::restart() {
  if (!s->paused) return;
  s->paused = false;
  while (hasNextJob()) {
    Job j = createJob();
    jobs->writeAccess(this);
    jobs->data.push(j);
    jobs->deAccess(this);
  }
  s->restartWorker();
  this->threadLoop();

}

template<class Job, class Result> void sMaster<Job, Result>::restartWorker() {
  typename std::vector<Worker<Job, Result>* >::iterator w;
  for (w = workers.begin(); w != workers.end(); w++) {
    (*w)->threadLoop();
  }
}

template<class Job, class Result> void Master<Job, Result>::open() {
  s->paused = true;
}

template<class Job, class Result> void Master<Job, Result>::step() {
  results->writeAccess(this);
  if (!results->data.empty()) {
    integrateResult(results->data.front());
    results->data.pop();
  }
  results->deAccess(this);
}

template<class Job, class Result> void Master<Job, Result>::close() {
}

template<class Job, class Result> Master<Job, Result>::~Master() {
  for (uint i = 0; i < s->numOfWorkers; ++i) {
     Worker<Job, Result>* w = s->workers.back();
     w->threadStop();
     w->threadClose();
     s->workers.pop_back();
  }
}

template<class T> Pool<T>::Pool() : Variable("Pool Variable") {}

#endif
