#include "act.h"
#include "roopi.h"
#include <Core/util.h>

template<> const char* mlr::Enum<ActStatus>::names []={
  /*"AS_create", */"AS_running", "AS_done", "AS_stalled", "AS_converged", NULL
};

Act::Act(Roopi *r)
  : roopi(*r), startTime(mlr::realTime()) {
  registryNode = registry().newNode<Act*>({"Act", typeid(*this).name()}, {}, this);
  roopi.acts.set()->append(this);
  setStatus(AS_create);
}

//Act::Act(Act&& a)
//  : roopi(a.roopi), ConditionVariable(a.status){
//  registryNode = a.registryNode;
//  registryNode->get<Act*>() = this;
//  a.registryNode = NULL;
//}

Act::~Act(){
  roopi.acts.set()->removeValue(this);
  if(registryNode) delete registryNode;
}

double Act::time(){ return mlr::realTime()-startTime; }

RUN_ON_INIT_BEGIN(roopi_act)
ActL::memMove=true;
RUN_ON_INIT_END(roopi_act)
