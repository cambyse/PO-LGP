#include "act_PathOpt.h"
#include "roopi.h"

#include <Core/thread.h>
#include <Motion/komo.h>
#include <Kin/kinViewer.h>

struct sAct_PathOpt : Thread{
  Access_typed<arr> x;
  KOMO *komo;
  ConditionVariable *status;
  OrsPathViewer *viewer=NULL;

  Conv_KOMO_ConstrainedProblem *CP=NULL;
  OptConstrained *opt = NULL;
  sAct_PathOpt(KOMO *komo, ConditionVariable *status)
    : Thread("Act_PathOpt", 0.), x(this, "PathOpt_x"), komo(komo), status(status){}
  ~sAct_PathOpt(){ threadClose(); }
  virtual void open();
  virtual void step();
  virtual void close();
};


Act_PathOpt::Act_PathOpt(Roopi* r)
  : Act(r), s(NULL),  komo(NULL) {
  komo = new KOMO;
  komo->setModel(roopi.getK());
  s = new sAct_PathOpt(komo, this);
}

Act_PathOpt::~Act_PathOpt(){
  if(s->viewer) delete s->viewer;
  delete s;
  delete komo;
}

void Act_PathOpt::start(){ s->threadLoop(); }

void Act_PathOpt::stop(){ s->threadStop(); }

void sAct_PathOpt::open(){
  komo->reset();

  if(CP) delete CP;
  if(opt) delete opt;

  CP = new Conv_KOMO_ConstrainedProblem(komo->MP->komo_problem);
  opt = new OptConstrained(komo->x, komo->dual, *CP);
  opt->earlyPhase = true;
}

void sAct_PathOpt::step(){
  bool stop = opt->step();
  x.set() = komo->x;
  if(stop){
    status->setStatus(AS_converged);

    cout <<"KOMO PathOpt done:\n" <<komo->getReport() <<endl;
    if(!viewer){
      viewer = new OrsPathViewer("PathOpt_x");
      viewer->threadLoop();
    }
    viewer->stepMutex.lock();
    viewer->setConfigurations(komo->MP->configurations);
    viewer->stepMutex.unlock();

    threadStop();
  }
}

void sAct_PathOpt::close(){
  if(CP) delete CP;   CP=NULL;
  if(opt) delete opt;  opt=NULL;
}

