/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */

#include "feedbackControl.h"
#include <Ors/ors_swift.h>

//===========================================================================

CtrlTask::CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : map(*map), name(name), active(true), prec(0.), Pgain(0.), Dgain(0.), maxVel(maxVel), maxAcc(maxAcc), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  setGainsAsNatural(decayTime, dampingRatio);
}

CtrlTask::CtrlTask(const char* name, TaskMap& map, Graph& params)
  : map(map), name(name), active(true), prec(0.), Pgain(0.), Dgain(0.), maxVel(1.), maxAcc(10.), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  Node *it;
  if((it=params["PD"])){
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  } else {
    setGainsAsNatural(3., .7);
  }
  if((it=params["prec"])) prec = it->get<double>();
  if((it=params["target"])) y_ref = it->get<arr>();
}

//CtrlTask::CtrlTask(const char* name, double decayTime, double dampingRatio,
//               DefaultTaskMapType type, const ors::KinematicWorld& G,
//               const char* iShapeName, const ors::Vector& ivec,
//               const char* jShapeName, const ors::Vector& jvec,
//               const arr& params)
//  : map(*new DefaultTaskMap(type, G, iShapeName, ivec, jShapeName, jvec, params)), name(name), active(true), prec(0.), Pgain(0.), Dgain(0.), flipTargetScalarProduct(false){
//  setGainsAsNatural(decayTime, dampingRatio);
//}

void CtrlTask::setTarget(const arr& yref, const arr& vref){
  y_ref = yref;
  if(&vref) v_ref=vref; else v_ref.resizeAs(y_ref).setZero();
}

void CtrlTask::setGains(double pgain, double dgain) {
  active=true;
  this->Pgain=pgain;
  this->Dgain=dgain;
  if(!prec) prec=100.;
}

void CtrlTask::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  active=true;
  double lambda = -decayTime*dampingRatio/log(.1);
  this->Pgain = mlr::sqr(1./lambda);
  this->Dgain = 2.*dampingRatio/lambda;
  if(!prec) prec=100.;
}

arr CtrlTask::getDesiredAcceleration(const arr& y, const arr& ydot){
  if(!y_ref.N) y_ref.resizeAs(y).setZero();
  if(!v_ref.N) v_ref.resizeAs(ydot).setZero();
  this->y = y;
  this->v = ydot;
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y, y_ref) < 0)
    y_ref = -y_ref;
  if(makeTargetModulo2PI) for(uint i=0;i<y.N;i++){
      while(y_ref(i) < y(i)-MLR_PI) y_ref(i)+=MLR_2PI;
      while(y_ref(i) > y(i)+MLR_PI) y_ref(i)-=MLR_2PI;
  }
  //compute diffs
  arr y_diff(y);
  if(y_ref.N==1) {
    y_diff -= y_ref.scalar();
  }else if(y_ref.N==y_diff.N) {
    y_diff -= y_ref;
  }
  arr ydot_diff(ydot);
  if(v_ref.N==1) {
    ydot_diff -= v_ref.scalar();
  }else if(v_ref.N==ydot_diff.N) {
    ydot_diff -= v_ref;
  }

  arr a = - Pgain*y_diff - Dgain*ydot_diff;
  //check limits
  double accNorm = length(a);
  if(accNorm<1e-4) return a;
  if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
  if(!maxVel) return a;
  double velRatio = scalarProduct(ydot, a/accNorm)/maxVel;
  if(velRatio>1.) a.setZero();
  else if(velRatio>.9) a *= 1.-10.*(velRatio-.9);
  return a;
}

void CtrlTask::getForceControlCoeffs(arr& f_des, arr& u_bias, arr& K_I, arr& J_ft_inv, const ors::KinematicWorld& world){
  //-- get necessary Jacobians
  DefaultTaskMap *m = dynamic_cast<DefaultTaskMap*>(&map);
  CHECK(m,"this only works for the default position task map");
  CHECK(m->type==posTMT,"this only works for the default positioni task map");
  CHECK(m->i>=0,"this only works for the default position task map");
  ors::Body *body = world.shapes(m->i)->body;
  ors::Vector vec = world.shapes(m->i)->rel*m->ivec;
  ors::Shape* l_ft_sensor = world.getShapeByName("l_ft_sensor");
  arr J_ft, J;
  world.kinematicsPos         (NoArr, J,   body, vec);
  world.kinematicsPos_wrtFrame(NoArr, J_ft,body, vec, l_ft_sensor);

  //-- compute the control coefficients
  u_bias = ~J*f_ref;
  f_des = f_ref;
  J_ft_inv = inverse_SymPosDef(J_ft*~J_ft)*J_ft;
  K_I = f_Igain*~J;
}

void CtrlTask::reportState(ostream& os){
  os <<"  CtrlTask " <<name;
  if(active) {
    if(y_ref.N==y.N && v_ref.N==v.N){
      os <<":  y_ref=" <<y_ref <<" \ty=" <<y
           <<"  Pterm=(" <<Pgain <<'*' <<length(y_ref-y)
           <<")   Dterm=(" <<Dgain <<'*' <<length(v_ref-v) <<')'
           <<endl;
    }else{
      os <<" -- y_ref.N!=y.N or v_ref.N!=v.N -- not initialized? -- "
           <<" Pgain=" <<Pgain
           <<" Dgain=" <<Dgain <<endl;
    }
  }else{
    os <<" -- inactive" <<endl;
  }
}

//===========================================================================

void ConstraintForceTask::updateConstraintControl(const arr& _g, const double& lambda_desired){
  CHECK_EQ(_g.N,1, "can handle only 1D constraints so far");
  double g=_g(0);
  CHECK(lambda_desired>=0., "lambda must be positive or zero");

  if(g<0 && lambda_desired>0.){ //steer towards constraint
    desiredApproach.y_ref=ARR(.05); //set goal to overshoot!
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=1e4;
  }

  if(g>-1e-2 && lambda_desired>0.){ //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.05, .7);
    desiredApproach.prec=1e6;
  }

  if(g>-0.02 && lambda_desired==0.){ //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.04);
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=1e4;
  }

  if(g<=-0.02 && lambda_desired==0.){ //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

FeedbackMotionControl::FeedbackMotionControl(ors::KinematicWorld& _world, bool _useSwift)
  : world(_world), qitselfPD(NULL), useSwift(_useSwift) {
  computeMeshNormals(world.shapes);
  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*mlr::getParameter<double>("swiftCutoff", 0.11));
  }
  H_rate_diag = getH_rate_diag(world);
  qitselfPD.name="qitselfPD";
  qitselfPD.setGains(0.,100.);
  qitselfPD.prec=1.;
}

CtrlTask* FeedbackMotionControl::addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map){
  return tasks.append(new CtrlTask(name, map, decayTime, dampingRatio, 1., 1.));
}

CtrlTask* FeedbackMotionControl::addPDTask(const char* name,
                                         double decayTime, double dampingRatio,
                                         DefaultTaskMapType type,
                                         const char* iShapeName, const ors::Vector& ivec,
                                         const char* jShapeName, const ors::Vector& jvec){
  return tasks.append(new CtrlTask(name, new DefaultTaskMap(type, world, iShapeName, ivec, jShapeName, jvec),
                                   decayTime, dampingRatio, 1., 1.));
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

void FeedbackMotionControl::getCostCoeffs(arr& c, arr& J){
  c.clear();
  if(&J) J.clear();
  arr y, J_y, yddot_des;
  for(CtrlTask* t: tasks) {
    if(t->active && !t->f_ref.N) {
      t->map.phi(y, J_y, world);
      yddot_des = t->getDesiredAcceleration(y, J_y*world.qdot);
      c.append(::sqrt(t->prec)*(yddot_des /*-Jdot*qdot*/));
      if(&J) J.append(::sqrt(t->prec)*J_y);
    }
  }
  if(&J) J.reshape(c.N, world.q.N);
}

void FeedbackMotionControl::reportCurrentState(){
  for(CtrlTask* t: tasks) t->reportState(cout);
}

void FeedbackMotionControl::setState(const arr& q, const arr& qdot){
  world.setJointState(q, qdot);
  if(useSwift) world.stepSwift();
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
      CHECK_EQ(y.N,1," can only handle 1D constraints for now");
      Jl += ~J_y * t->desiredForce;
    }
  }
  Jl.reshape(Jl.N);
  return Jl;
}

arr FeedbackMotionControl::operationalSpaceControl(){
  arr c, J;
  getCostCoeffs(c, J); //this corresponds to $J_\phi$ and $c$ in the reference (they include C^{1/2})
  if(!c.N && !qitselfPD.active) return zeros(world.q.N,1).reshape(world.q.N);
  arr A = diag(H_rate_diag);
  arr a(A.d0); a.setZero();
  if(qitselfPD.active){
    a += H_rate_diag % qitselfPD.getDesiredAcceleration(world.q, world.qdot);
  }
  if(c.N){
    A += comp_At_A(J);
    a += comp_At_x(J, c);
  }
  arr q_ddot = inverse_SymPosDef(A) * a;
  return q_ddot;
}

RUN_ON_INIT_BEGIN(CtrlTask)
mlr::Array<CtrlTask*>::memMove=true;
RUN_ON_INIT_END(CtrlTask)
