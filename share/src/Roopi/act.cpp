#include "act.h"
#include "roopi.h"
#include <Core/util.h>

template<> const char* mlr::Enum<ActStatus>::names []={
  /*"AS_init", */"AS_running", "AS_done", "AS_converged", "AS_stalled", "AS_true", "AS_false", "AS_kill", NULL
};

Act::Act(Roopi *r)
  : roopi(*r), startTime(mlr::realTime()) {
  registryNode = registry()->newNode<Act*>({"Act", typeid(*this).name()}, {}, this);
  roopi.acts.set()->append(this);
  setStatus(AS_init);
}

//Act::Act(Act&& a)
//  : roopi(a.roopi), Signaler(a.status){
//  registryNode = a.registryNode;
//  registryNode->get<Act*>() = this;
//  a.registryNode = NULL;
//}

Act::~Act(){
  roopi.acts.set()->removeValue(this);
  if(registryNode) registry()->delNode(registryNode);
}

double Act::time(){ return mlr::realTime()-startTime; }

void Act::write(ostream& os){ os <<'<' <<std::setw(14) <<NAME(typeid(*this)) <<"> @" <<std::setw(12) <<mlr::Enum<ActStatus>((ActStatus)getStatus()) <<std::setw(5) <<std::setprecision(3)<<time() <<"s -- "; }


RUN_ON_INIT_BEGIN(roopi_act)
ActL::memMove=true;
RUN_ON_INIT_END(roopi_act)
