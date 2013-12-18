#include "feedbackControl.h"

//===========================================================================

void PDtask::setTarget(const arr& yref, const arr& vref){
  y_ref = yref;
  if(&vref) v_ref=vref; else v_ref.resizeAs(y_ref).setZero();
}

void PDtask::setGains(double pgain, double dgain) {
  active=true;
  Pgain=pgain;
  Dgain=dgain;
  if(!prec) prec=100.;
}

void PDtask::setGainsAsNatural(double decayTime, double dampingRatio) {
  active=true;
  Pgain = MT::sqr(1./decayTime);
  Dgain = 2.*dampingRatio/decayTime;
  if(!prec) prec=100.;
}

arr PDtask::getDesiredAcceleration(const arr& y, const arr& ydot){
  if(!y_ref.N) y_ref.resizeAs(y).setZero();
  if(!v_ref.N) v_ref.resizeAs(ydot).setZero();
  Perr = y_ref-y;
  Derr = v_ref-ydot;
  return Pgain*Perr + Dgain*Derr;
}

//===========================================================================

FeedbackMotionControl::FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift)
  : MotionProblem(_world, useSwift), nullSpacePD(NULL) {
  loadTransitionParameters();
  nullSpacePD.setGainsAsNatural(1.,1.);
  nullSpacePD.prec=1.;
}

PDtask* FeedbackMotionControl::addTask(const char* name, TaskMap *m){
  PDtask *t = new PDtask(m);
  t->name=name;
  tasks.append(t);
  return t;
}

PDtask* FeedbackMotionControl::addPDTask(const char* name,
                                         double decayTime, double dampingRatio,
                                         DefaultTaskMapType type,
                                         const char* iShapeName, const ors::Vector& ivec,
                                         const char* jShapeName, const ors::Vector& jvec,
                                         const arr& params){
  PDtask *t = addTask(name, new DefaultTaskMap(type, world, iShapeName, ivec, jShapeName, jvec, params));
  t->setGainsAsNatural(decayTime, dampingRatio);
  return t;
}

void FeedbackMotionControl::getTaskCosts(arr& phi, arr& J, arr& a){
  phi.clear();
  if(&J) J.clear();
  arr y, J_y, a_des;
  for(PDtask* t: tasks){
    if(t->active) {
      t->map.phi(y, J_y, world);
      a_des = t->getDesiredAcceleration(y, J_y*world.qdot);
      phi.append(::sqrt(t->prec)*(J_y*a - a_des));
      if(&J) J.append(::sqrt(t->prec)*J_y);
    }
  }
  if(&J) J.reshape(phi.N, a.N);
}

arr FeedbackMotionControl::operationalSpaceControl(){
  arr phi, J, a;
  a.resizeAs(world.q).setZero();
  getTaskCosts(phi, J, a);
  if(!phi.N) return a;
  arr H, Jinv;
  H.setDiag(1./H_rate_diag);
  pseudoInverse(Jinv, J, H, 1e-6);
  arr Null = eye(a.N) - Jinv * J;
  a = - Jinv * phi + Null * nullSpacePD.getDesiredAcceleration(world.q, world.qdot);
  return a;
}

