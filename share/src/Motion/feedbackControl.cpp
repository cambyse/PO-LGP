#include "feedbackControl.h"

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
  Pgain = MT::sqr(MT_PI/decayTime);
  Dgain = 4.*dampingRatio*sqrt(Pgain);
  if(!prec) prec=100.;
}

arr PDtask::getDesiredAcceleration(const arr& y, const arr& ydot){
  return Pgain*(y_ref-y) + Dgain*(v_ref-ydot);
}

FeedbackMotionProblem::FeedbackMotionProblem(ors::Graph *_ors, SwiftInterface *_swift, bool useSwift) {
  if(_ors)   ors   = _ors;   else { ors=new ors::Graph;        ors  ->init(MT::getParameter<MT::String>("orsFile")); } // orLinkTree(); }
  if(useSwift){
    if(_swift) swift = _swift; else { swift=new SwiftInterface;  swift->init(*ors, 2.*MT::getParameter<double>("swiftCutoff", 0.11)); }
  }else swift=NULL;
  ors->getJointState(x_current, v_current);
}

PDtask* FeedbackMotionProblem::addTask(const char* name, TaskMap *m){
  PDtask *t = new PDtask(m);
  t->name=name;
  tasks.append(t);
  return t;
}

void FeedbackMotionProblem::setState(const arr& q, const arr& v) {
  if(&v) v_current = v;
  x_current = q;
  ors->setJointState(q);
  ors->calcBodyFramesFromJoints();
  if(swift) swift->computeProxies(*ors, false);
}

void FeedbackMotionProblem::loadTransitionParameters() {
  //transition type
//  transitionType = (TransitionType)MT::getParameter<int>("transitionType");

  //time and steps
  tau = 0.01;

  //transition cost metric
  H_rate_diag.resize(x_current.N)  = MT::getParameter<double>("Hrate");
}

void FeedbackMotionProblem::getTaskCosts(arr& phi, arr& J, arr& a){
  phi.clear();
  if(&J) J.clear();
  arr y, J_y, a_des;
  for(PDtask* t: tasks){
    if(t->active) {
      t->map.phi(y, J_y, *ors);
      a_des = t->getDesiredAcceleration(y, J_y*v_current);
      phi.append(::sqrt(t->prec)*(J_y*a - a_des));
      if(&J) J.append(::sqrt(t->prec)*J_y);
    }
  }
  if(&J) J.reshape(phi.N, a.N);
}

arr FeedbackMotionProblem::operationalSpaceControl(){
  arr phi, J, a;
  a.resizeAs(x_current).setZero();
  getTaskCosts(phi, J, a);
  arr H, Jinv;
  H.setDiag(H_rate_diag);
  pseudoInverse(Jinv, J, H, 1e-6);
  a = - Jinv * phi;
  return a;
}

//virtual void FeedbackMotionProblem::fv(arr& phi, arr& J, const arr& a){
