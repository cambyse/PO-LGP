#include "potentialTaskVariables.h"
#include "graspObjects.h"

PotentialValuesTaskVariable::PotentialValuesTaskVariable(const char* _name,
                              ors::KinematicWorld& _ors,
                              const ShapeList& _refs,
                              PotentialField& _f){
  refs=_refs;
  f=&_f;
  set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());
}

void PotentialValuesTaskVariable::userUpdate(const ors::KinematicWorld& ors){
  uint i;
  ors::Shape *s;
  arr xi,Ji,grad;
  y.resize(refs.N);
  J.resize(refs.N,ors.getJointStateDimension());
  for_list(Type, s, refs){
    ors.kinematicsPos(xi,s->body->index,&s->rel.pos);
    ors.jacobian  (Ji,s->body->index,&s->rel.pos);
    y(i) = f->psi(&grad,NULL,xi);
    J[i]() = ~grad*Ji;
  }
  transpose(Jt,J);
}

PotentialFieldAlignTaskVariable::PotentialFieldAlignTaskVariable(const char* _name,
    ors::KinematicWorld& _ors,
    const ShapeList& _refs,
    PotentialField& _f){
  refs=_refs;
  f=&_f;
  set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());
}

/** Compute current value and jacobian of the TV.
 * (compare  to mlr/stanio/notes/Jacobian_of_field_align_TV.tex)
 */
void PotentialFieldAlignTaskVariable::userUpdate(const ors::KinematicWorld& ors){
  uint i;
  ors::Shape *s;
  ors::Vector tmp;
  arr xi,zi,Jzi,Jxi,grad,hess;
  y.resize(refs.N);
  J.resize(refs.N,ors.getJointStateDimension());
  for_list(Type, s, refs){
    ors.kinematics   (xi,s->body->index,&s->rel.pos);
    ors.jacobian     (Jxi,s->body->index,&s->rel.pos);
    ors.kinematicsVec(zi,s->body->index,&s->rel.rot.getZ(tmp));
    ors.jacobianVec  (Jzi,s->body->index,&s->rel.rot.getZ(tmp));
    f->psi(&grad,&hess,xi);
    grad /= length(grad);
    y(i) = scalarProduct(grad,zi);
    J[i]() = (~Jxi * hess) * zi  + ~grad * Jzi ;
  }
  transpose(Jt,J);
}

GPVarianceTaskVariable::GPVarianceTaskVariable(const char* _name,
                              ors::KinematicWorld& _ors,
                              const ShapeList& _refs,
                              GraspObject_GP& _f){
  refs=_refs;
  f=&_f;
  set(_name, _ors, userTVT, -1, Transformation_Id, -1, Transformation_Id, ARR());
}

/** $ \dfdx{\vec y_i}{\vec q} =  2 (\vec{G^{-1}}\vec\kappa) \vec\kappa'\vec Ji $
 * need: inverse Gram, kappa, and derivative of kappa
 */
void GPVarianceTaskVariable::userUpdate(const ors::KinematicWorld& ors){
  uint i;
  ors::Shape *s;
  arr xi,Ji,ki,dki,*Ginv;

  y.resize(refs.N);
  J.resize(refs.N,ors.getJointStateDimension());
  Ginv = &f->isf_gp.gp.Ginv;
  for_list(Type, s, refs){
    ors.kinematicsPos(xi,s->body->index,&s->rel.pos);
    ors.jacobian  (Ji,s->body->index,&s->rel.pos);
    f->isf_gp.gp.k_star(xi,ki);
    f->isf_gp.gp.dk_star(xi,dki);
    f->phi(NULL,NULL,&y(i),xi);
    J[i]() = 2. * ( ( (*Ginv) * ki ) * dki) * Ji ;/* TODO */ 
  }
  transpose(Jt,J);
}

