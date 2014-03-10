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

void ConstraintForceTask::updateConstraintControl(const arr& _g, const double& lambda_desired){
  CHECK(_g.N==1, "can handle only 1D constraints so far");
  double g=_g(0);
  CHECK(lambda_desired>=0., "lambda must be positive or zero");

  if(g<0 && lambda_desired>0.){ //steer towards constraint
    desiredApproach.y_ref=ARR(.05); //set goal to overshoot!
    desiredApproach.setGainsAsNatural(.2, 1.);
    desiredApproach.prec=1e4;
  }

  if(g>-1e-2 && lambda_desired>0.){ //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.01, .7);
    desiredApproach.prec=1e6;
  }

  if(g>-0.02 && lambda_desired==0.){ //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.02);
    desiredApproach.setGainsAsNatural(.2, 1.);
    desiredApproach.prec=100.;
  }

  if(g<=-0.02 && lambda_desired==0.){ //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

FeedbackMotionControl::FeedbackMotionControl(ors::KinematicWorld& _world, bool useSwift)
  : MotionProblem(_world, useSwift), nullSpacePD(NULL) {
  loadTransitionParameters();
  nullSpacePD.setGainsAsNatural(1.,1.);
  nullSpacePD.prec=1.;
}

PDtask* FeedbackMotionControl::addTask(const char* name, TaskMap *map){
  PDtask *t = new PDtask(map);
  t->name=name;
  tasks.append(t);
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
  PDtask *t = addTask(name, new DefaultTaskMap(type, world, iShapeName, ivec, jShapeName, jvec, params));
  t->setGainsAsNatural(decayTime, dampingRatio);
  return t;
}

void FeedbackMotionControl::getTaskCosts(arr& phi, arr& J, arr& q_ddot){
  phi.clear();
  if(&J) J.clear();
  arr y, J_y, a_des;
  for(PDtask* t: tasks){
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
  if(!phi.N) return q_ddot;
  arr H = diag(1./H_rate_diag);
  arr A = H + comp_At_A(J);
  arr a = -comp_At_x(J, phi);
  if(nullSpacePD.active) a += H * nullSpacePD.prec * nullSpacePD.getDesiredAcceleration(world.q, world.qdot);
  q_ddot = inverse_SymPosDef(A) * a;
  return q_ddot;
}
