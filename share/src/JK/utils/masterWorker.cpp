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
    Integrator<Job, Result> *integrator;

    void restartWorker();
    bool paused;
};

template<class Job, class Result> Worker<Job, Result>::Worker(const char* name) : Process(name) {}


template<class Job, class Result> void Worker<Job, Result>::open() {
}


template<class Job, class Result> void Worker<Job, Result>::step() {
  Job j;
  for(;;){
    workspace->readAccess(this);
    uint jobs = workspace->jobs.size();
    workspace->deAccess(this);
    
    if(!jobs) break;
    
    workspace->writeAccess(this);
    j = workspace->jobs.front();
    workspace->jobs.pop();
    workspace->working_jobs++;
    workspace->jobs_to_do--;
    workspace->deAccess(this);
    
    Result r;
    doWork(r, j);
    
    workspace->writeAccess(this);
    workspace->results.append(r);
    workspace->working_jobs--;
    workspace->deAccess(this);
  }
}

template<class Job, class Result> void Worker<Job, Result>::close() {
    
}

template<class Job, class Result> Master<Job, Result>::Master(WorkerFactory<Job, Result>* fac, Integrator<Job, Result> *i, const int numOfWorkers) {
  s = new sMaster<Job, Result>;
  s->workerFactory = fac;
  s->numOfWorkers = numOfWorkers; 
  s->integrator = i;


  workspace = new Workspace<Job, Result>();

  s->paused = true;
  
  for (uint i = 0; i < s->numOfWorkers; ++i) {
    Worker<Job, Result>* w = s->workerFactory->createWorker();
    s->workers.push_back(w);
    w->workspace = workspace;
    w->threadOpen();
  }
  s->integrator->workspace = workspace;
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
    workspace->writeAccess(p);
    workspace->jobs.push(j);
    workspace->deAccess(p);
  }
  s->restartWorker();
  s->integrator->restart();
  s->integrator->threadLoop();

}
template<class Job, class Result> void Master<Job, Result>::restart(std::queue<Job> &jobs, Process *p) {
  if (!s->paused) return;
  s->paused = false;
  workspace->writeAccess(p);
  workspace->jobs = jobs;
  workspace->deAccess(p);
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

template<class Job, class Result> void Integrator<Job, Result>::open() {
}

template<class Job, class Result> void Integrator<Job, Result>::restart() {
}

template<class Job, class Result> void Integrator<Job, Result>::step() {
  workspace->writeAccess(this);
  if (!workspace->results.empty()) {
    integrateResult(workspace->results.front());
    workspace->results.pop();
  }
  workspace->deAccess(this);
}

template<class Job, class Result> void Integrator<Job, Result>::close() {
}

template<class Job, class Result> Master<Job, Result>::~Master() {
  for (uint i = 0; i < s->numOfWorkers; ++i) {
     Worker<Job, Result>* w = s->workers.back();
     w->threadStop();
     w->threadClose();
     s->workers.pop_back();
  }
}

template<class Job, class Result> Workspace<Job, Result>::Workspace() : Variable("Master-Worker Workspace Variable") { } 
#endif
