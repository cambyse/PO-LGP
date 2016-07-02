#include "taskMap_FixAttachedObjects.h"
#include "taskMap_qItself.h"
#include "taskMap_default.h"

uint FixAttachedObjectsTaskMap::dim_phi(const WorldL& G, int t){
  mlr::Array<ors::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  return switchedJoints.d0*7;
}


void FixAttachedObjectsTaskMap::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  CHECK(order==1,"");
  CHECK(G.N==2,"");

  arr yi, Ji;
  uint M=7;
  mlr::Array<ors::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  y.resize(M*switchedJoints.d0);
  if(&J) J.resize(M*switchedJoints.d0, G.elem(-2)->q.N+G.elem(-1)->q.N);
  for(uint i=0;i<switchedJoints.d0;i++){
    ors::Joint *j0 = switchedJoints(i,0);    CHECK(&j0->world==G.elem(-2),"");
    ors::Joint *j1 = switchedJoints(i,1);    CHECK(&j1->world==G.elem(-1),"");
    CHECK(j0->to->index == j1->to->index,"");

    DefaultTaskMap pos(posDiffTMT, j0->to->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y.refRange(M*i,M*i+2)(), (&J?J.refRange(M*i,M*i+2)():NoArr), G, tau, t);

    DefaultTaskMap quat(quatDiffTMT, j0->to->shapes.first()->index);
    quat.order=1;
    quat.TaskMap::phi(y.refRange(M*i+3,M*i+6)(), (&J?J.refRange(M*i+3,M*i+6)():NoArr), G, tau, t);
  }
}


