#include "miscTaskVariables.h"

zOpposeTaskVariable::zOpposeTaskVariable(const char* _name,
                                         ors::Graph& _ors,
                                         ShapeList& _refs){
  refs=_refs;
  set(_name, _ors, userTVT, -1, ors::Transformation(), -1, ors::Transformation(), ARR());
}

void zOpposeTaskVariable::userUpdate(){
    //compute sum of z-vectors for n shapes
    //return sunOfSqr of this sum (is a scalar task variable)
  uint i;
  ors::Shape *s;
  arr zi,Ji,sum_z,sum_J;
  sum_J.resize(refs.N,ors->getJointStateDimension()); sum_J.setZero();
  sum_z.resize(3); sum_z.setZero();
  for_list(i,s,refs){
    ors->kinematicsZ(zi,s->body->index,&s->rel);
    sum_z += zi/norm(zi);
    ors->jacobianZ  (Ji,s->body->index,&s->rel);
    sum_J += Ji;
  }
  y.resize(1);
  y(0)=sumOfSqr(sum_z);
  J = 2.*sum_z*sum_J;
  J.reshape(1,ors->getJointStateDimension());
  transpose(Jt,J);
}

zFocusTargetTaskVariable::zFocusTargetTaskVariable(const char* _name,
    ors::Graph& _ors,
    ShapeList& _refs){
  refs=_refs;
  set(_name, _ors, userTVT, -1, ors::Transformation(), -1, ors::Transformation(), ARR());
}

void zFocusTargetTaskVariable::userUpdate(){
    //for all n shapes:
    //diff=target-shape->X.p;
    //z = shape->X.getZ();
    //offset = diff - <diff,z>*z;
    //return |offset|^2;
}

