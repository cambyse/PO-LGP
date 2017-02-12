#include "act.h"
#include "roopi.h"
#include <Core/util.h>

template<> const char* mlr::Enum<ActStatus>::names []={
  /*"AS_create", */"AS_running", "AS_done", "AS_stalled", "AS_converged", NULL
};

Act::Act(Roopi *r)
  : roopi(*r), startTime(mlr::realTime()) {
  registryNode = registry().newNode<Act*>({"Act", "bla"}, {}, this);
  roopi.registerAct(this);
  setStatus(AS_create);
}

Act::~Act(){
  roopi.deregisterAct(this);
  delete registryNode;
}

double Act::time(){ return mlr::realTime()-startTime; }

RUN_ON_INIT_BEGIN(roopi_act)
ActL::memMove=true;
RUN_ON_INIT_END(roopi_act)
