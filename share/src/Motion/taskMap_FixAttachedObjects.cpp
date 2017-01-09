/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


#include "taskMap_FixAttachedObjects.h"
#include "taskMap_qItself.h"
#include "taskMap_default.h"

uint TaskMap_FixSwichedObjects::dim_phi(const WorldL& G, int t){
  mlr::Array<mlr::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  return switchedJoints.d0*7;
}


void TaskMap_FixSwichedObjects::phi(arr& y, arr& J, const WorldL& G, double tau, int t){
  //TODO: so far this only fixes switched objects to zero pose vel
  //better: constrain to zero relative velocity with BOTH, pre-attached and post-attached
  CHECK(order==1,"");

  arr yi, Ji;
  uint M=7;
  mlr::Array<mlr::Joint*> switchedJoints = getSwitchedJoints(*G.elem(-2), *G.elem(-1));
  y.resize(M*switchedJoints.d0);
  if(&J){
    uint xbarDim=0;
    for(auto& W:G) xbarDim+=W->q.N;
    J.resize(M*switchedJoints.d0, xbarDim).setZero();
  }
  for(uint i=0;i<switchedJoints.d0;i++){
    mlr::Joint *j0 = switchedJoints(i,0);    CHECK(&j0->world==G.elem(-2),"");
    mlr::Joint *j1 = switchedJoints(i,1);    CHECK(&j1->world==G.elem(-1),"");
    CHECK(j0->to->index == j1->to->index,"");

    if(j1->to->name.startsWith("slider")) continue;

#if 1 //absolute velocities
    TaskMap_Default pos(posDiffTMT, j0->to->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index);
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#else //relative velocities
    TaskMap_Default pos(posDiffTMT, j0->to->shapes.first()->index, NoVector, j0->from->shapes.first()->index);
    pos.order=1;
    pos.TaskMap::phi(y({M*i,M*i+2})(), (&J?J({M*i,M*i+2})():NoArr), G, tau, t);

    TaskMap_Default quat(quatDiffTMT, j0->to->shapes.first()->index/*, NoVector, j0->from->shapes.first()->index*/);
    quat.order=1;
    quat.TaskMap::phi(y({M*i+3,M*i+6})(), (&J?J({M*i+3,M*i+6})():NoArr), G, tau, t);
#endif
  }
}


