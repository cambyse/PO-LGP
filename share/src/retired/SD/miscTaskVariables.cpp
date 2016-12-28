#include "miscTaskVariables.h"

zOpposeTaskVariable::zOpposeTaskVariable(const char* _name,
                                         mlr::KinematicWorld& _ors,
                                         ShapeList& _refs){
  refs=_refs;
  set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());
}

void zOpposeTaskVariable::userUpdate(const mlr::KinematicWorld& ors){
    //compute sum of z-vectors for n shapes
    //return sunOfSqr of this sum (is a scalar task variable)
  uint i;
  mlr::Shape *s;
  mlr::Vector tmp;
  arr zi,Ji,sum_z,sum_J;
  sum_J.resize(refs.N,ors.getJointStateDimension()); sum_J.setZero();
  sum_z.resize(3); sum_z.setZero();
  for_list(Type, s, refs){
    ors.kinematicsVec(zi,s->body->index,&s->rel.rot.getZ(tmp));
    sum_z += zi/length(zi);
    ors.jacobianVec  (Ji,s->body->index,&s->rel.rot.getZ(tmp));
    sum_J += Ji;
  }
  y.resize(1);
  y(0)=sumOfSqr(sum_z);
  J = 2.*~sum_z*sum_J;
  J.reshape(1,ors.getJointStateDimension());
  transpose(Jt,J);
}

zFocusTargetTaskVariable::zFocusTargetTaskVariable(const char* _name,
    mlr::KinematicWorld& _ors,
    ShapeList& _refs){
  refs=_refs;
  set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());
}

void zFocusTargetTaskVariable::userUpdate(const mlr::KinematicWorld& ors){
    //for all n shapes:
    //diff=target-shape->X.p;
    //z = shape->X.getZ();
    //offset = diff - <diff,z>*z;
    //return |offset|^2;
}

