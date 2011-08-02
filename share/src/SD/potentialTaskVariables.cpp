#include "potentialTaskVariables.h"
#include "graspObjects.h"

PotentialValuesTaskVariable::PotentialValuesTaskVariable(const char* _name,
                              ors::Graph& _ors,
                              ShapeList& _refs,
                              PotentialField& _f){
  refs=_refs;
  f=&_f;
  set(_name, _ors, userTVT, -1, ors::Transformation(), -1, ors::Transformation(), ARR());
}

void PotentialValuesTaskVariable::userUpdate(){
  uint i;
  ors::Shape *s;
  arr xi,Ji,grad;
  y.resize(refs.N);
  J.resize(refs.N,ors->getJointStateDimension());
  for_list(i,s,refs){
    ors->kinematics(xi,s->body->index,&s->rel);
    ors->jacobian  (Ji,s->body->index,&s->rel);
    y(i) = f->psi(&grad,NULL,xi);
    J[i]() = grad*Ji;
  }
  transpose(Jt,J);
}

PotentialFieldAlignTaskVariable::PotentialFieldAlignTaskVariable(const char* _name,
    ors::Graph& _ors,
    ShapeList& _refs,
    PotentialField& _f){
      refs=_refs;
      f=&_f;
      set(_name, _ors, userTVT, -1, ors::Transformation(), -1, ors::Transformation(), ARR());
}

void PotentialFieldAlignTaskVariable::userUpdate(){
  uint i;
  ors::Shape *s;
  arr xi,zi,Ji,grad,hess;
  y.resize(refs.N);
  J.resize(refs.N,ors->getJointStateDimension());
  for_list(i,s,refs){
    ors->kinematics (xi,s->body->index,&s->rel);
    ors->kinematicsZ(zi,s->body->index,&s->rel);
    ors->jacobianZ  (Ji,s->body->index,&s->rel);
    f->psi(&grad,NULL,xi);
    //f->hessian(&hess,xi);
    grad /= norm(grad);
    y(i) = scalarProduct(zi,grad);
    J[i]() = ~grad * Ji ; //+ ~zi * hess;
  }
  transpose(Jt,J);
}
