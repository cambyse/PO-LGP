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
  : map(*map), name(name), active(true), prec(0.), maxVel(maxVel), maxAcc(maxAcc), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  setGainsAsNatural(decayTime, dampingRatio);
}

CtrlTask::CtrlTask(const char* name, TaskMap& map, Graph& params)
  : map(map), name(name), active(true), prec(0.), maxVel(1.), maxAcc(10.), flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  Node *it;
  if((it=params["PD"])){
    arr pd=it->V<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  } else {
    setGainsAsNatural(3., .7);
  }
  if((it=params["prec"])) prec = it->V<double>();
  if((it=params["target"])) y_ref = it->V<arr>();
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

void CtrlTask::setGains(const arr& _Kp, const arr& _Kd) {
  active=true;
  Kp = _Kp;
  Kd = _Kd;
  if(!prec) prec=100.;
}

void CtrlTask::setGains(double pgain, double dgain) {
  active=true;
  Kp = ARR(pgain);
  Kd = ARR(dgain);
  if(!prec) prec=100.;
}

void CtrlTask::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  double lambda = -decayTime*dampingRatio/log(.1);
  setGains(mlr::sqr(1./lambda), 2.*dampingRatio/lambda);
}

void makeRefsVectors(arr& y_ref, arr& yd_ref, uint n){
  if(!y_ref.N) y_ref = zeros(n);
  if(!yd_ref.N==1) yd_ref = zeros(n);
  if(y_ref.N==1) y_ref.setUni(y_ref.scalar(), n);
  if(yd_ref.N==1) yd_ref.setUni(yd_ref.scalar(), n);
  CHECK(y_ref.nd==1 && y_ref.d0==n,"");
  CHECK(yd_ref.nd==1 && yd_ref.d0==n,"");
}

void makeGainsMatrices(arr& Kp, arr& Kd, uint n){
  if(Kp.N==1) Kp = diag(Kp.scalar(), n);
  if(Kd.N==1) Kd = diag(Kd.scalar(), n);
  CHECK(Kp.nd==2 && Kp.d0==n && Kp.d1==n,"");
  CHECK(Kd.nd==2 && Kd.d0==n && Kd.d1==n,"");
}

arr CtrlTask::get_y_ref(const arr& y){
  this->y = y;
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y, y_ref) < 0)
    y_ref = -y_ref;
  if(makeTargetModulo2PI) for(uint i=0;i<y.N;i++){
    while(y_ref(i) < y(i)-MLR_PI) y_ref(i)+=MLR_2PI;
    while(y_ref(i) > y(i)+MLR_PI) y_ref(i)-=MLR_2PI;
  }
  return y_ref;
}

arr CtrlTask::get_ydot_ref(const arr& ydot){
  this->v = ydot;
  return v_ref;
}

arr CtrlTask::getDesiredAcceleration(const arr& y, const arr& ydot){
  makeRefsVectors(y_ref, v_ref, y.N);
  makeGainsMatrices(Kp, Kd, y.N);
  arr a = Kp*(get_y_ref(y)-y) + Kd*(get_ydot_ref(ydot)-ydot);

  //check vel/acc limits
  double accNorm = length(a);
  if(accNorm<1e-4) return a;
  if(maxAcc>0. && accNorm>maxAcc) a *= maxAcc/accNorm;
  if(!maxVel) return a;
  double velRatio = scalarProduct(ydot, a/accNorm)/maxVel;
  if(velRatio>1.) a.setZero();
  else if(velRatio>.9) a *= 1.-10.*(velRatio-.9);
  return a;
}

void CtrlTask::getDesiredLinAccLaw(arr& Kp_y, arr& Kd_y, arr& a0_y, const arr& y, const arr& ydot){
  makeRefsVectors(y_ref, v_ref, y.N);
  makeGainsMatrices(Kp, Kd, y.N);
  a0_y = Kp*get_y_ref(y) + Kd*get_ydot_ref(ydot);
  Kp_y = -Kp;
  Kd_y = -Kd;
  arr a = a0_y + Kp_y*y + Kd_y*ydot; //linear law
  double accNorm = length(a);

  //check vel limit -> change a0, no change in gains
  if(maxVel){
    double velRatio = scalarProduct(ydot, a/accNorm)/maxVel;
    if(velRatio>1.) a0_y -= a; //a becomes zero
    else if(velRatio>.9) a0_y -= a*(10.*(velRatio-.9));
  }

  //check acc limits -> change all
  if(maxAcc>1e-4 && accNorm>maxAcc){
    double scale = maxAcc/accNorm;
    a0_y *= scale;
    Kp_y *= scale;
    Kd_y *= scale;
  }
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
  K_I = f_alpha*~J;
}

void CtrlTask::reportState(ostream& os){
  os <<"  CtrlTask " <<name;
  if(active) {
    if(y_ref.N==y.N && v_ref.N==v.N){
      os <<":  y_ref=" <<y_ref <<" \ty=" <<y
           <<"  Pterm=(" <<Kp <<'*' <<length(y_ref-y)
           <<")   Dterm=(" <<Kd <<'*' <<length(v_ref-v) <<')'
           <<endl;
    }else{
      os <<" -- y_ref.N!=y.N or v_ref.N!=v.N -- not initialized? -- "
           <<" Pgain=" <<Kp
           <<" Dgain=" <<Kd <<endl;
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

void FeedbackMotionControl::calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0) {
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr q0, q, qDot;
  world.getJointState(q,qDot);

  //arr H = /*diag(world.getHmetric());//*/0.1*eye(world.getJointStateDimension());
  //M = H;
  //F = zeros(world.getJointStateDimension());

  arr H = inverse(M); //TODO: Other metrics (have significant influence)

  arr A = ~M*H*M; //TODO: The M matrix is symmetric, isn't it? And also symmetric? Furthermore, if H = M^{-1}, this should be calculated more efficiently
  arr a = zeros(q.N); //M*eye(world.getJointStateDimension())*5.0*(-qDot);// //TODO: other a possible
  u0 = ~M*H*(a-F);
  arr y, J_y, Kp_y, Kd_y, a0_y;
  arr tempKp, tempKd;

  q0 = q;
  Kp = zeros(q.N, q.N);
  Kd = zeros(q.N, q.N);
  for(CtrlTask* law : tasks) if(law->active){
    law->map.phi(y, J_y, world);
    A += ~J_y*law->prec*J_y;
    law->getDesiredLinAccLaw(Kp_y, Kd_y, a0_y, y, J_y*world.qdot);
//    makeRefsVectors(law->y_ref, law->v_ref, y.N);
//    makeGainsMatrices(law->Kp, law->Kd, y.N);
    tempKp = ~J_y*law->prec*Kp_y;
    tempKd = ~J_y*law->prec*Kd_y;
    u0 += ~J_y*law->prec*a0_y;
//    u0 += tempKp*(law->get_y_ref(y) - y + J_y*q0);
//    u0 += tempKd*law->get_ydot_ref(world.qdot);
//    u0 += ~J*law->getC()*law->getDDotRef(); //TODO: add ydd_ref
    Kp += tempKp*J_y;
    Kd += tempKd*J_y;
  }
  arr invA = inverse(A); //TODO: SymPosDef?
  Kp = M*invA*Kp;
  Kd = M*invA*Kd;
  u0 = M*invA*u0 + F;
}

void FeedbackMotionControl::fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0){
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr u = u0 + Kp*world.q + Kd*world.qdot;
  arr qdd;
  world.fwdDynamics(qdd, world.qdot, u);

  for(uint tt=0;tt<1;tt++){
    world.q += .001*world.qdot;
    world.qdot += .001*qdd;
  }

  setState(world.q, world.qdot);
}

void FeedbackMotionControl::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma) {
  uint nForceTasks=0;
  for(CtrlTask* law : this->tasks) if(law->active && law->f_ref.N){
    nForceTasks++;
    DefaultTaskMap& map = dynamic_cast<DefaultTaskMap&>(law->map);
    ors::Body* body = world.shapes(map.i)->body;
    ors::Vector vec = world.shapes(map.i)->rel.pos;
    ors::Shape* lFtSensor = world.getShapeByName("l_ft_sensor");
    arr y, J, J_ft;
    law->map.phi(y, J, world);
    world.kinematicsPos_wrtFrame(NoArr, J_ft, body, vec, lFtSensor);
    J_ft_inv = -~conv_vec2arr(map.ivec)*inverse_SymPosDef(J_ft*~J_ft)*J_ft;
    K_ft = -~J*law->f_alpha;
    fRef = law->f_ref;
    gamma = law->f_gamma;
  }

  CHECK(nForceTasks<=1, "Multiple force laws not allowed at the moment");
  if(!nForceTasks){
    K_ft = zeros(world.getJointStateDimension());
    fRef = ARR(0.0);
    J_ft_inv = zeros(1,6);
    gamma = 0.0;
  }

}

RUN_ON_INIT_BEGIN(CtrlTask)
mlr::Array<CtrlTask*>::memMove=true;
RUN_ON_INIT_END(CtrlTask)
