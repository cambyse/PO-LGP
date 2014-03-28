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
  double lambda = -decayTime*dampingRatio/log(.1);
  Pgain = MT::sqr(1./lambda);
  Dgain = 2.*dampingRatio/lambda;
  if(!prec) prec=100.;
}

arr PDtask::getDesiredAcceleration(const arr& y, const arr& ydot){
  if(!y_ref.N) y_ref.resizeAs(y).setZero();
  if(!v_ref.N) v_ref.resizeAs(ydot).setZero();
  this->y = y;
  this->v = ydot;
//  cout <<" TASK " <<name <<":  \tPterm=(" <<Pgain <<'*' <<length(y_ref-y) <<")  \tDterm=(" <<Dgain <<'*' <<length(v_ref-ydot) <<')' <<endl;
  return Pgain*(y_ref-y) + Dgain*(v_ref-ydot);
}

//===========================================================================

void ConstraintForceTask::updateConstraintControl(const arr& _g, const double& lambda_desired){
  CHECK(_g.N==1, "can handle only 1D constraints so far");
  double g=_g(0);
  CHECK(lambda_desired>=0., "lambda must be positive or zero");

  if(g<0 && lambda_desired>0.){ //steer towards constraint
    desiredApproach.y_ref=ARR(.05); //set goal to overshoot!
    desiredApproach.setGainsAsNatural(.1, 1.);
    desiredApproach.prec=1e4;
  }

  if(g>-1e-2 && lambda_desired>0.){ //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.02, .7);
    desiredApproach.prec=1e6;
  }

  if(g>-0.02 && lambda_desired==0.){ //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.02);
    desiredApproach.setGainsAsNatural(.1, 1.);
    desiredApproach.prec=1e4;
  }

  if(g<=-0.02 && lambda_desired==0.){ //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

FeedbackMotionControl::FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift)
  : MotionProblem(_world, useSwift), qitselfPD(NULL) {
  loadTransitionParameters();
  qitselfPD.name="nullSpacePD";
  qitselfPD.setGains(1.,10.);
//  qitselfPD.setGainsAsNatural(1.,1.);
  qitselfPD.prec=1.;
}

PDtask* FeedbackMotionControl::addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map){
  PDtask *t = new PDtask(map);
  t->name=name;
  tasks.append(t);
  t->setGainsAsNatural(decayTime, dampingRatio);
  return t;
}

ConstraintForceTask* FeedbackMotionControl::addConstraintForceTask(const char* name, TaskMap *map){
  ConstraintForceTask *t = new ConstraintForceTask(map);
  t->name=name;
  t->desiredApproach.name=STRING(name <<"_PD");
  t->desiredApproach.active=false;
  forceTasks.append(t);
  tasks.append(&t->desiredApproach);
  return t;
}

PDtask* FeedbackMotionControl::addPDTask(const char* name,
                                         double decayTime, double dampingRatio,
                                         DefaultTaskMapType type,
                                         const char* iShapeName, const ors::Vector& ivec,
                                         const char* jShapeName, const ors::Vector& jvec,
                                         const arr& params){
  PDtask *t = addPDTask(name, decayTime, dampingRatio, new DefaultTaskMap(type, world, iShapeName, ivec, jShapeName, jvec, params));
  return t;
}

void FeedbackMotionControl::getTaskCosts(arr& phi, arr& J, arr& q_ddot){
  phi.clear();
  if(&J) J.clear();
  arr y, J_y, a_des;
  for(PDtask* t: tasks) {
    if(t->active) {
      t->map.phi(y, J_y, world);
      a_des = t->getDesiredAcceleration(y, J_y*world.qdot);
      phi.append(::sqrt(t->prec)*(J_y*q_ddot - a_des));
      if(&J) J.append(::sqrt(t->prec)*J_y);
    }
  }
  if(&J) J.reshape(phi.N, q_ddot.N);
}

void FeedbackMotionControl::updateConstraintControllers(){
  arr y;
  for(ConstraintForceTask* t: forceTasks){
    if(t->active){
      t->map.phi(y, NoArr, world);
      t->updateConstraintControl(y, t->desiredForce);
    }
  }
}

arr FeedbackMotionControl::getDesiredConstraintForces(){
  arr Jl(world.q.N, 1);
  Jl.setZero();
  arr y, J_y;
  for(ConstraintForceTask* t: forceTasks){
    if(t->active) {
      t->map.phi(y, J_y, world);
      CHECK(y.N==1," can only handle 1D constraints for now");
      Jl += ~J_y * t->desiredForce;
    }
  }
  Jl.reshape(Jl.N);
  return Jl;
}

arr FeedbackMotionControl::operationalSpaceControl(){
  arr phi, J, q_ddot;
  q_ddot.resizeAs(world.q).setZero();
  getTaskCosts(phi, J, q_ddot);
  if(!phi.N && !qitselfPD.active) return q_ddot;
  arr H = diag(H_rate_diag);
  arr A = H;
  arr a(H.d0); a.setZero();
  if(qitselfPD.active){
    a += qitselfPD.prec * (H_rate_diag % qitselfPD.getDesiredAcceleration(world.q, world.qdot));
  }
  if(phi.N){
    A += comp_At_A(J);
    a -= comp_At_x(J, phi);
  }
//  if(qitselfPD.active){
//    A += qitselfPD.prec * eye(H.d0);
//    a -= qitselfPD.prec * (q_ddot - qitselfPD.getDesiredAcceleration(world.q, world.qdot));
//  }
  q_ddot = inverse_SymPosDef(A) * a;

//  if(nullSpacePD.active && nullSpacePD.prec){
//    arr Null = eye(a.N) - Ainv * A;
//    q_ddot += Null * nullSpacePD.getDesiredAcceleration(world.q, world.qdot);
//  }

  return q_ddot;
}
