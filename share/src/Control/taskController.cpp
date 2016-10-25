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
#include "taskController.h"
#include <Ors/ors_swift.h>
#include <Motion/motion.h>
#include <Motion/taskMaps.h>

//===========================================================================

CtrlTask::CtrlTask(const char* name, TaskMap* map)
  : map(*map), name(name), active(true), prec(ARR(100.)), maxVel(0.), maxAcc(0.), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
}

CtrlTask::CtrlTask(const char* name, TaskMap* map, double decayTime, double dampingRatio, double maxVel, double maxAcc)
  : map(*map), name(name), active(true), prec(ARR(100.)), maxVel(maxVel), maxAcc(maxAcc), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  setGainsAsNatural(decayTime, dampingRatio);
}

CtrlTask::CtrlTask(const char* name, TaskMap& map, const Graph& params)
  : map(map), name(name), active(true), prec(ARR(100.)), maxVel(0.), maxAcc(0.), f_alpha(0.), f_gamma(0.),
    flipTargetSignOnNegScalarProduct(false), makeTargetModulo2PI(false){
  if(!params["PD"]) setGainsAsNatural(3., .7);
  set(params);
}

void CtrlTask::set(const Graph& params){
  Node *it;
  if((it=params["PD"])){
    arr pd=it->get<arr>();
    setGainsAsNatural(pd(0), pd(1));
    maxVel = pd(2);
    maxAcc = pd(3);
  }
  if((it=params["prec"])) prec = it->get<arr>();
  if((it=params["target"])) y_ref = it->get<arr>();
}

void CtrlTask::setTarget(const arr& yref, const arr& vref){
  y_ref = yref;
  if(&vref) v_ref=vref; else v_ref.resizeAs(y_ref).setZero();
}

void CtrlTask::setGains(const arr& _Kp, const arr& _Kd) {
  //active=true; //TODO
  Kp = _Kp;
  Kd = _Kd;
  if(!prec.N) prec=ARR(100.);
}

void CtrlTask::setGains(double pgain, double dgain) {
  //active=true; //TODO
  Kp = ARR(pgain);
  Kd = ARR(dgain);
  if(!prec.N) prec=ARR(100.);
}

void CtrlTask::setGainsAsNatural(double decayTime, double dampingRatio) {
  CHECK(decayTime>0. && dampingRatio>0., "this does not define proper gains!");
  double lambda = -decayTime*dampingRatio/log(.1);
  setGains(mlr::sqr(1./lambda), 2.*dampingRatio/lambda);
}

void CtrlTask::setC(const arr& C) {
  prec = C;
}


void makeGainsMatrices(arr& Kp, arr& Kd, uint n){
  if(Kp.N==1) Kp = diag(Kp.scalar(), n);
  if(Kd.N==1) Kd = diag(Kd.scalar(), n);
  if(Kp.nd==1) Kp = diag(Kp);
  if(Kd.nd==1) Kd = diag(Kd);
  CHECK(Kp.nd==2 && Kp.d0==n && Kp.d1==n,"");
  CHECK(Kd.nd==2 && Kd.d0==n && Kd.d1==n,"");
}

arr CtrlTask::get_y_ref(const arr& _y){
  if(&_y) y = _y;
  if(y_ref.N!=y.N){
    if(!y_ref.N) y_ref = zeros(y.N); //by convention: no-references = zero references
    if(y_ref.N==1) y_ref = consts<double>(y_ref.scalar(), y.N); //by convention: scalar references = const vector references
  }
  if(flipTargetSignOnNegScalarProduct && scalarProduct(y, y_ref) < 0)
    y_ref = -y_ref;
  if(makeTargetModulo2PI) for(uint i=0;i<y.N;i++){
    while(y_ref(i) < y(i)-MLR_PI) y_ref(i)+=MLR_2PI;
    while(y_ref(i) > y(i)+MLR_PI) y_ref(i)-=MLR_2PI;
  }
  return y_ref;
}

arr CtrlTask::get_ydot_ref(const arr& _ydot){
  if(&_ydot) this->v = _ydot;
  if(v_ref.N!=v.N){
    if(!v_ref.N) v_ref = zeros(v.N);
    if(v_ref.N==1) v_ref = consts<double>(v_ref.scalar(), v.N);
  }
  return v_ref;
}

arr CtrlTask::getC(){
  uint n=y_ref.N;
  if(prec.N==1) return diag(prec.scalar(), n);
  if(prec.nd==1) return diag(prec);
  return prec;
}

arr CtrlTask::getDesiredAcceleration(const arr& y, const arr& ydot){
  arr Kp_y = Kp;
  arr Kd_y = Kd;
  makeGainsMatrices(Kp_y, Kd_y, y.N);
  arr a = Kp_y*(get_y_ref(y)-y) + Kd_y*(get_ydot_ref(ydot)-ydot);

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
  Kp_y = Kp;
  Kd_y = Kd;
  makeGainsMatrices(Kp_y, Kd_y, y.N);

  arr y_delta = get_y_ref(y) - y;
  double y_delta_length = length(y_delta);
  if(maxVel && y_delta_length>maxVel)
    y_delta *= maxVel/y_delta_length;

  a0_y = Kp_y*(y+y_delta) + Kd_y*get_ydot_ref(ydot);
  arr a = a0_y - Kp_y*y - Kd_y*ydot; //linear law
  double accNorm = length(a);

  return;

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
  TaskMap_Default *m = dynamic_cast<TaskMap_Default*>(&map);
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

double CtrlTask::error(){
  if(!(y.N && y.N==y_ref.N && v.N==v_ref.N)) return -1.;
  return maxDiff(y, y_ref) + maxDiff(v, v_ref);
}

bool CtrlTask::isConverged(double tolerance){
  return (y.N && y.N==y_ref.N && v.N==v_ref.N
          && maxDiff(y, y_ref)<tolerance
          && maxDiff(v, v_ref)<tolerance);
}

void CtrlTask::reportState(ostream& os){
  if(active) {
    os <<"  CtrlTask " <<name;
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
    desiredApproach.prec=ARR(1e4);
  }

  if(g>-1e-2 && lambda_desired>0.){ //stay in constraint -> constrain dynamics
    desiredApproach.y_ref=ARR(0.);
    desiredApproach.setGainsAsNatural(.05, .7);
    desiredApproach.prec=ARR(1e6);
  }

  if(g>-0.02 && lambda_desired==0.){ //release constraint -> softly push out
    desiredApproach.y_ref=ARR(-0.04);
    desiredApproach.setGainsAsNatural(.3, 1.);
    desiredApproach.prec=ARR(1e4);
  }

  if(g<=-0.02 && lambda_desired==0.){ //stay out of contact -> constrain dynamics
    desiredApproach.active=false;
  }
}

//===========================================================================

TaskController::TaskController(ors::KinematicWorld& _world, bool _useSwift)
  : world(_world), qNullCostRef(NULL, NULL), useSwift(_useSwift) {
  computeMeshNormals(world.shapes);
  if(useSwift) {
    makeConvexHulls(world.shapes);
    world.swift().setCutoff(2.*mlr::getParameter<double>("swiftCutoff", 0.11));
  }
  qNullCostRef.name="qitselfPD";
  qNullCostRef.setGains(0.,100.);
  qNullCostRef.prec = getH_rate_diag(world);
  qNullCostRef.setTarget( world.q );
}

CtrlTask* TaskController::addPDTask(const char* name, double decayTime, double dampingRatio, TaskMap *map){
  return tasks.append(new CtrlTask(name, map, decayTime, dampingRatio, 1., 1.));
}

CtrlTask* TaskController::addPDTask(const char* name,
                                         double decayTime, double dampingRatio,
                                         TaskMap_DefaultType type,
                                         const char* iShapeName, const ors::Vector& ivec,
                                         const char* jShapeName, const ors::Vector& jvec){
  return tasks.append(new CtrlTask(name, new TaskMap_Default(type, world, iShapeName, ivec, jShapeName, jvec),
                                   decayTime, dampingRatio, 1., 1.));
}

ConstraintForceTask* TaskController::addConstraintForceTask(const char* name, TaskMap *map){
  ConstraintForceTask *t = new ConstraintForceTask(map);
  t->name=name;
  t->desiredApproach.name=STRING(name <<"_PD");
  t->desiredApproach.active=false;
  forceTasks.append(t);
  tasks.append(&t->desiredApproach);
  return t;
}

void TaskController::getTaskCoeffs(arr& c, arr& J){
  c.clear();
  if(&J) J.clear();
  arr y, J_y, yddot_des;
  for(CtrlTask* t: tasks) {
    if(t->active && !t->f_ref.N) {
      t->map.phi(y, J_y, world);
      yddot_des = t->getDesiredAcceleration(y, J_y*world.qdot);
      c.append(::sqrt(t->prec)%(yddot_des /*-Jdot*qdot*/));
      if(&J) J.append(::sqrt(t->prec)%J_y);
    }
  }
  if(&J) J.reshape(c.N, world.q.N);
}

void TaskController::reportCurrentState(){
  for(CtrlTask* t: tasks) t->reportState(cout);
}

void TaskController::setState(const arr& q, const arr& qdot){
  world.setJointState(q, qdot);
  if(useSwift) world.stepSwift();
}

void TaskController::updateConstraintControllers(){
  arr y;
  for(ConstraintForceTask* t: forceTasks){
    if(t->active){
      t->map.phi(y, NoArr, world);
      t->updateConstraintControl(y, t->desiredForce);
    }
  }
}

arr TaskController::getDesiredConstraintForces(){
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

arr TaskController::operationalSpaceControl(){
  arr c, J;
  getTaskCoeffs(c, J); //this corresponds to $J_\phi$ and $c$ in the reference (they include C^{1/2})
  if(!c.N && !qNullCostRef.active) return zeros(world.q.N,1).reshape(world.q.N);
  arr A = qNullCostRef.getC();
  arr a = zeros(A.d0);
  if(qNullCostRef.active){
    a += qNullCostRef.getC() * qNullCostRef.getDesiredAcceleration(world.q, world.qdot);
  }
  if(c.N){
    A += comp_At_A(J);
    a += comp_At_x(J, c);
  }
  arr q_ddot = inverse_SymPosDef(A) * a;
  return q_ddot;
}

arr TaskController::getDesiredLinAccLaw(arr &Kp, arr &Kd, arr &k) {
  arr Kp_y, Kd_y, k_y, C_y;
  qNullCostRef.getDesiredLinAccLaw(Kp_y, Kd_y, k_y, world.q, world.qdot);
  arr H = qNullCostRef.getC();

  Kp = H * Kp_y;
  Kd = H * Kd_y;
  k  = H * k_y;

  arr JCJ = zeros(world.q.N, world.q.N);

  for(CtrlTask* task : tasks) if(task->active){
    arr y, J_y;
    task->map.phi(y, J_y, world);
    task->getDesiredLinAccLaw(Kp_y, Kd_y, k_y, y, J_y*world.qdot);
    C_y = task->getC();

    arr JtC_y = ~J_y*C_y;

    JCJ += JtC_y*J_y;

    Kp += JtC_y*Kp_y*J_y;
    Kd += JtC_y*Kd_y*J_y;
    k  += JtC_y*(k_y + Kp_y*(J_y*world.q - y));
  }
  arr invA = inverse_SymPosDef(H + JCJ);

  arr E = zeros(4,world.q.N);
  E(0,0) = 1; //Fix Base
  E(1,1) = 1; //Fix Base
  E(2,2) = 1; //Fix Base

  E(3,3) = 1; //Fix Torso

  invA = invA*(eye(world.q.N)-~E*inverse_SymPosDef(E*invA*~E)*E*invA);
  Kp = invA*Kp;
  Kd = invA*Kd;
  k  = invA*k;

  return k - Kp*world.q - Kd*world.qdot;
}


arr TaskController::calcOptimalControlProjected(arr &Kp, arr &Kd, arr &u0, const arr& M, const arr& F) {
  uint n=F.N;

//  arr q0, q, qDot;
//  world.getJointState(q,qDot);

  arr H = inverse(M); //TODO: Other metrics (have significant influence)

  arr A = ~M*H*M; //TODO: The M matrix is symmetric, isn't it? And also symmetric? Furthermore, if H = M^{-1}, this should be calculated more efficiently
  arr a = zeros(n); //TODO M*eye(world.getJointStateDimension())*5.0*(-qDot);// //TODO: other a possible
  u0 = ~M*H*(a-F);
  arr y, J_y, Kp_y, Kd_y, a0_y;
  arr tempJPrec, tempKp;

//  q0 = q;
  Kp = zeros(n, n);
  Kd = zeros(n, n);
  //TODO: add qitselfPD!!
//  if(qitselfPD.active){
//    a += H_rate_diag % qitselfPD.getDesiredAcceleration(world.q, world.qdot);
//  }
  for(CtrlTask* law : tasks) if(law->active){
    law->map.phi(y, J_y, world);
    tempJPrec = ~J_y*law->getC();
    A += tempJPrec*J_y;

    law->getDesiredLinAccLaw(Kp_y, Kd_y, a0_y, y, J_y*world.qdot);

    u0 += tempJPrec*a0_y;

    tempKp = tempJPrec*Kp_y;

    u0 += tempKp*(-y + J_y*world.q);

    //u0 += ~J*law->getC()*law->getDDotRef(); //TODO: add ydd_ref

    Kp += tempKp*J_y;
    Kd += tempJPrec*Kd_y*J_y;
  }
  arr invA = inverse(A); //TODO: SymPosDef?
  Kp = M*invA*Kp;
  Kd = M*invA*Kd;
  u0 = M*invA*u0 + F;

  return u0 + Kp*world.q + Kd*world.qdot;
}

void TaskController::fwdSimulateControlLaw(arr& Kp, arr& Kd, arr& u0){
  arr M, F;
  world.equationOfMotion(M, F, false);

  arr u = u0 - Kp*world.q - Kd*world.qdot;
  arr qdd;
  world.fwdDynamics(qdd, world.qdot, u);

  for(uint tt=0;tt<10;tt++){
    world.qdot += .001*qdd;
    world.q += .001*world.qdot;
    setState(world.q, world.qdot);
  }
}

void TaskController::calcForceControl(arr& K_ft, arr& J_ft_inv, arr& fRef, double& gamma) {
  uint nForceTasks=0;
  for(CtrlTask* law : this->tasks) if(law->active && law->f_ref.N){
    nForceTasks++;
    TaskMap_Default& map = dynamic_cast<TaskMap_Default&>(law->map);
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
