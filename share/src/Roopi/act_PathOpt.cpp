#include "act_PathOpt.h"
#include "roopi.h"

Act_PathOpt::Act_PathOpt(Roopi* r)
  : Thread("Act_PathOpt", 0.), roopi(r), x(this, "PathOpt_x") {
  komo = new KOMO;
  komo->setModel(roopi->getKinematics());
}

Act_PathOpt::~Act_PathOpt(){
  delete komo;
}

void Act_PathOpt::open(){
  komo->reset();

  if(CP) delete CP;
  if(opt) delete opt;

  CP = new Conv_KOMO_ConstrainedProblem(komo->MP->komo_problem);
  opt = new OptConstrained(komo->x, komo->dual, *CP);
  opt->earlyPhase = true;
}

void Act_PathOpt::step(){
  bool stop = opt->step();
  x.set() = komo->x;
  if(stop){
    status.setValue(AS_converged);
    threadStop();
  }
}

void Act_PathOpt::close(){
  if(CP) delete CP;   CP=NULL;
  if(opt) delete opt;  opt=NULL;
  cout <<"KOMO PathOpt done:\n" <<komo->getReport() <<endl;
}

