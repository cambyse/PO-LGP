#include "act.h"
#include "roopi.h"
#include <Core/util.h>

Act::Act(Roopi *r)
  : roopi(*r), startTime(mlr::realTime()) {
  roopi.registerAct(this);
  setValue(AS_create);
}

Act::~Act(){
  roopi.deregisterAct(this);
}

double Act::time(){ return mlr::realTime()-startTime; }

RUN_ON_INIT_BEGIN(roopi_act)
ActL::memMove=true;
RUN_ON_INIT_END(roopi_act)
