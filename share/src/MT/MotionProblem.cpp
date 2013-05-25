#include "MotionProblem.h"
#include "opengl.h"

struct DefaultTaskMap:TaskMap{
  DefaultTaskMapType type;
  int i, j;             ///< which body(-ies) does it refer to?
  ors::Transformation irel, jrel; ///< relative position to the body
  arr params;           ///< parameters of the variable (e.g., liner coefficients, limits, etc)

  virtual void phi(arr& y, arr& J, const ors::Graph& G);
  virtual uint phiDim(const ors::Graph& G);
};

void DefaultTaskMap::phi(arr& y, arr& J, const ors::Graph& G){
  arr q;
  ors::Vector pi, pj, c;
  arr zi, zj, Ji, Jj, JRj;
  ors::Transformation f, fi, fj;
  ors::Vector vi, vj, r, jk;
  uint k,l;

  //get state
  switch(type) {
    case posTMT:
      if(j==-1) {
        G.kinematicsPos(y, i, &irel.pos);
        if(&J) G.jacobianPos(J, i, &irel.pos);
        break;
      }
      pi = G.bodies(i)->X.pos + G.bodies(i)->X.rot * irel.pos;
      pj = G.bodies(j)->X.pos + G.bodies(j)->X.rot * jrel.pos;
      c = G.bodies(j)->X.rot / (pi-pj);
      y.resize(3); y = ARRAY(c);
      G.jacobianPos(Ji, i, &irel.pos);
      G.jacobianPos(Jj, j, &jrel.pos);
      G.jacobianR(JRj, j);
      if(&J){
        J.resize(3, Jj.d1);
        for(k=0; k<Jj.d1; k++) {
          vi.set(Ji(0, k), Ji(1, k), Ji(2, k));
          vj.set(Jj(0, k), Jj(1, k), Jj(2, k));
          r .set(JRj(0, k), JRj(1, k), JRj(2, k));
          jk =  G.bodies(j)->X.rot / (vi - vj);
          jk -= G.bodies(j)->X.rot / (r ^(pi - pj));
          J(0, k)=jk.x; J(1, k)=jk.y; J(2, k)=jk.z;
        }
      }
      break;
    case zoriTMT:
      if(j==-1) {
        G.kinematicsVec(y, i, &irel.rot.getZ(vi));
        if(&J) G.jacobianVec(J, i, &irel.rot.getZ(vi));
        break;
      }
      //relative
      MT_MSG("warning - don't have a correct Jacobian for this TMType yet");
      fi = G.bodies(i)->X; fi.appendTransformation(irel);
      fj = G.bodies(j)->X; fj.appendTransformation(jrel);
      f.setDifference(fi, fj);
      f.rot.getZ(c);
      y = ARRAY(c);
      NIY; //TODO: Jacobian?
      break;
    case qItselfTMT:   G.getJointState(q);    y = q;   if(&J) J.setId(q.N);  break;
    case qLinearTMT:   G.getJointState(q);    y = params * q;   if(&J) J=params;  break;
    case qSquaredTMT:
      G.getJointState(q);
      y.resize(1);  y(0) = scalarProduct(params, q, q);
      if(&J){
        J = params * q;
        J *= (double)2.;
        J.reshape(1, q.N);
      }
      break;
    case qSingleTMT:
      G.getJointState(q);
      y.resize(1);  y(0)=q(-i);
      if(&J) {
        J.resize(1, G.getJointStateDimension());
        J.setZero();
        J(0, -i) = 1.;
      }
      break;
    case qLimitsTMT:   G.getLimitsMeasure(y, params);  if(&J) G.getLimitsGradient(J, params);   break;
    case comTMT:       G.getCenterOfMass(y);     y.resizeCopy(2); if(&J){ G.getComGradient(J);  J.resizeCopy(2, J.d1); }  break;
    case collTMT:      G.phiCollision(y, J, params(0));  break;
    case colConTMT:    G.getContactConstraints(y);  if(&J) G.getContactConstraintsGradient(J); break;
    case skinTMT:
      y.resize(params.N);
      y.setZero();
      if(&J){
        J.clear();
        for(k=0; k<params.N; k++) {
          l=(uint)params(k);
          G.jacobianPos(Ji, l, NULL);
          G.bodies(l)->X.rot.getY(vi);
          vi *= -1.;
          zi = ARRAY(vi);
          J.append(~zi*Ji);
        }
        J.reshape(params.N, J.N/params.N);
      }
      break;
    case zalignTMT:
      G.kinematicsVec(zi, i, &irel.rot.getZ(vi));
      if(&J) G.jacobianVec(Ji, i, &irel.rot.getZ(vi));
      if(j==-1) {
        ors::Vector world_z;
        if(params.N==3) world_z.set(params.p); else world_z=Vector_z;
        zj = ARRAY((jrel*world_z));
        if(&J){ Jj.resizeAs(Ji); Jj.setZero(); }
      } else {
        G.kinematicsVec(zj, j, &jrel.rot.getZ(vj));
        if(&J) G.jacobianVec(Jj, j, &jrel.rot.getZ(vj));
      }
      y.resize(1);
      y(0) = scalarProduct(zi, zj);
      if(&J){
        J = ~zj * Ji + ~zi * Jj;
        J.reshape(1, G.getJointStateDimension());
      }
      break;
    default:  HALT("no such TVT");
  }
}

uint DefaultTaskMap::phiDim(const ors::Graph& G){
  //get state
  switch(type) {
  case posTMT: return 3;
  case zoriTMT: return 3;
  case qItselfTMT: return G.getJointStateDimension();
  case qLinearTMT: return params.d0;
  case qSquaredTMT: return 1;
  case qSingleTMT: return 1;
  case qLimitsTMT: return 1;
  case comTMT: return 2;
  case collTMT: return 1;
  case colConTMT: return 1;
  case skinTMT: return params.N;
  case zalignTMT: return 1;
  default:  HALT("no such TVT");
  }
}

//===========================================================================

MotionProblem::MotionProblem(ors::Graph *_ors, SwiftInterface *_swift){
  if(_ors)   ors   = _ors;   else { ors=new ors::Graph;        ors  ->init(MT::getParameter<MT::String>("orsFile")); } // ormakeLinkTree(); }
  if(_swift) swift = _swift; else { swift=new SwiftInterface;  swift->init(*ors, 2.*MT::getParameter<double>("swiftCutoff", 0.11)); }
  ors->getJointState(x0, v0);
  x_current = x0;
  v_current = v0;
}

void MotionProblem::loadTransitionParameters(){
  //transition type
  transitionType = (TransitionType)MT::getParameter<int>("transitionType");

  //time and steps
  double duration = MT::getParameter<double>("duration");
  T = MT::getParameter<uint>("timeSteps");
  tau = duration/T;

  //transition cost metric
  arr W_diag;
  if(MT::checkParameter<arr>("Wdiag")){
    W_diag = MT::getParameter<arr>("Wdiag");
  }else{
    W_diag = ors->naturalQmetric();
  }
  H_rate_diag = MT::getParameter<double>("Hrate")*W_diag;
}

void MotionProblem::setx0(const arr& x){
  x0=x;
}

void MotionProblem::setx0v0(const arr& x, const arr& v){
  x0=x; v0=v;
}

TaskCost* MotionProblem::addDefaultTaskMap(
    const char* name,
    DefaultTaskMapType type,
    int iBody, const ors::Transformation& irel,
    int jBody, const ors::Transformation& jrel,
    const arr& params){
  DefaultTaskMap *m = new DefaultTaskMap();
  m->type=type;
  m->i=iBody;  m->irel=irel;
  m->j=jBody;  m->jrel=jrel;
  if(&params) m->params=params;
  TaskCost *t = new TaskCost(m);
  t->name=name;
  taskCosts.append(t);
  return t;
}

TaskCost* MotionProblem::addDefaultTaskMap(
    const char* name,
    DefaultTaskMapType type,
    const char *iBodyName, const char *iframe,
    const char *jBodyName, const char *jframe,
    const arr& params){
  ors::Body *a = iBodyName ? ors->getBodyByName(iBodyName):NULL;
  ors::Body *b = jBodyName ? ors->getBodyByName(jBodyName):NULL;
  return addDefaultTaskMap(
        name, type,
        a  ? (int)a->index : -1,
        iframe ? ors::Transformation().setText(iframe) : Transformation_Id,
        b  ? (int)b->index : -1,
        jframe ? ors::Transformation().setText(jframe) : Transformation_Id,
        params);
}

TaskCost* MotionProblem::addDefaultTaskMap(
    const char* name,
    DefaultTaskMapType type,
    const char *iShapeName,
    const char *jShapeName,
    const arr& params){
  ors::Shape *a = iShapeName ? ors->getShapeByName(iShapeName):NULL;
  ors::Shape *b = jShapeName ? ors->getShapeByName(jShapeName):NULL;
  return addDefaultTaskMap(
        name, type,
        a ? (int)a->body->index : -1,
        a ? a->rel : Transformation_Id,
        b ? (int)b->body->index : -1,
        b ? b->rel : Transformation_Id,
        params);
}

void MotionProblem::setInterpolatingCosts(
    TaskCost *c,
    TaskCostInterpolationType inType,
    const arr& y_finalTarget, double y_finalPrec, const arr& y_midTarget, double y_midPrec){
  uint m=c->map.phiDim(*ors);
  setState(x0,v0);
  arr y0;
  c->map.phi(y0, NoArr, *ors);
  //TODO: cleaner, next 3 lines
  arr midTarget(m),finTarget(y_finalTarget);
  if(&y_midTarget && y_midTarget.N==1) midTarget = y_midTarget(0);
  if(y_finalTarget.N==1){ finTarget.resize(m);  finTarget = y_finalTarget(0); }
  switch(inType){
  case constFinalMid:{
    c->y_target.resize(T+1, m);
    c->y_target[T]() = finTarget;
    for(uint t=0; t<T; t++) c->y_target[t]() = (&y_midTarget) ? midTarget : finTarget;
    c->y_prec.resize(T+1);
    c->y_prec = y_midPrec<0. ? y_finalPrec : y_midPrec;
    c->y_prec(T) = y_finalPrec;
  } break;
  case linearInterpolation:{
    c->y_target.resize(T+1, m);
    for(uint t=0; t<=T; t++) {
      double a = (double)t/T;
      c->y_target[t]() = ((double)1.-a)*y0 + a*finTarget;
    }
    c->y_prec.resize(T+1);
    c->y_prec = y_midPrec<0. ? y_finalPrec : y_midPrec;
    c->y_prec(T) = y_finalPrec;
  } break;
  }
}

void MotionProblem::setInterpolatingVelCosts(
    TaskCost *c,
    TaskCostInterpolationType inType,
    const arr& v_finalTarget, double v_finalPrec, const arr& v_midTarget, double v_midPrec){
  uint m=c->map.phiDim(*ors);
  setState(x0,v0);
  arr y0,yv0,J;
  c->map.phi(y0, J, *ors);
  yv0 = J * v0;
  arr midTarget(m), finTarget(m);
  if(&v_finalTarget && v_finalTarget.N==1) finTarget = v_finalTarget(0); else finTarget=v_finalTarget;
  if(&v_midTarget && v_midTarget.N==1) midTarget = v_midTarget(0); else midTarget=v_midTarget;
  switch(inType){
  case constFinalMid:{
    c->v_target.resize(T+1, m);
    c->v_target[T]() = finTarget;
    for(uint t=0; t<T; t++) c->v_target[t]() = (&v_midTarget) ? midTarget : finTarget;
    c->v_prec.resize(T+1);
    c->v_prec = v_midPrec<0. ? v_finalPrec : v_midPrec;
    c->v_prec(T) = v_finalPrec;
  } break;
  case linearInterpolation:{
    c->v_target.resize(T+1, m);
    for(uint t=0; t<=T; t++) {
      double a = (double)t/T;
      c->v_target[t]() = ((double)1.-a)*yv0 + a*finTarget;
    }
    c->v_prec.resize(T+1);
    c->v_prec = v_midPrec<0. ? v_finalPrec : v_midPrec;
    c->v_prec(T) = v_finalPrec;
  } break;
  }
}

void MotionProblem::setState(const arr& q, const arr& v){
  v_current = v;
  x_current = q;
  ors->setJointState(q);
//  if(q_external.N)
//    ors->setExternalState(q_external[0]);
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
  if(transitionType == realDynamic){
    NIY;
    //requires computation of the real dynamics, i.e. of M and F
  }
}


uint MotionProblem::get_phiDim(uint t){
  uint m=0;
  for(uint i=0; i<taskCosts.N; i++){
    TaskCost *c = taskCosts(i);
    if(c->active){
      if(c->y_target.N) m += c->map.phiDim(*ors);
      if(transitionType!=kinematic && c->v_target.N)  m += c->map.phiDim(*ors);
    }
  }
  return m;
}

void MotionProblem::getTaskCosts(arr& phi, arr& J_x, arr& J_v, uint t){
  phi.clear();
  if(&J_x) J_x.clear();
  if(&J_v) J_v.clear();
  arr y,J;
  for(uint i=0; i<taskCosts.N; i++){
    TaskCost *c = taskCosts(i);
    if(c->active){
      c->map.phi(y, J, *ors);
      if(c->y_target.N){ //pose costs
        phi.append(sqrt(c->y_prec(t))*(y - c->y_target[t]));
        if(&J_x) J_x.append(sqrt(c->y_prec(t))*J);
        if(&J_v) J_v.append(0.*J);
      }
      if(transitionType!=kinematic && c->v_target.N){ //velocity costs
        phi.append(sqrt(c->v_prec(t))*(J*v_current - c->v_target[t]));
        if(&J_x) J_x.append(0.*J);
        if(&J_v) J_v.append(sqrt(c->v_prec(t))*J);
      }
    }
  }
  if(&J_x) J_x.reshape(phi.N, x_current.N);
  if(&J_v) J_v.reshape(phi.N, x_current.N);
}

uint MotionProblem::get_psiDim(){
  return x0.N;
}

void MotionProblem::costReport(){
  CHECK(costMatrix.d1 == get_psiDim() + get_phiDim(0),"");
  cout <<"*** MotionProblem -- CostReport" <<endl;

  double transC=0., taskC=0., tc;
  cout <<" * transition costs:" <<endl;
  transC=sumOfSqr(costMatrix.sub(0,-1,0,get_psiDim()-1));
  cout <<"\t total=" <<transC <<endl;

  uint m=get_psiDim();

  cout <<" * task costs:" <<endl;
  for(uint i=0;i<taskCosts.N;i++){
    TaskCost *c = taskCosts(i);
    uint d=c->map.phiDim(*ors);

    cout <<"\t '" <<c->name <<"' [" <<d <<"] ";

    if(c->y_target.N){
      taskC+=tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      cout <<"\t state=" <<tc;
      m += d;
    }
    if(transitionType!=kinematic && c->v_target.N){
      taskC+=tc=sumOfSqr(costMatrix.sub(0,-1,m,m+d-1));
      cout <<"\t vel=" <<tc;
      m += d;
    }
    cout <<endl;
  }
  cout <<"\t total=" <<taskC <<endl;

  cout <<" * task+transition=" <<taskC+transC <<endl;

  CHECK(m == costMatrix.d1, "");
}

uint MotionProblemFunction::get_T(){ return P.T; }
uint MotionProblemFunction::get_k(){ if(P.transitionType==MotionProblem::kinematic) return 1;  return 2; }
uint MotionProblemFunction::get_n(){ return P.x0.N; }
uint MotionProblemFunction::get_m(uint t){
  uint nq = get_n();
  return nq + P.get_phiDim(t+1);
}

arr MotionProblemFunction::get_prefix(){
  arr x_pre(get_k(), get_n());
  for(uint i=0;i<x_pre.d0;i++) x_pre[i]() = P.x0;
  return x_pre;
}

void MotionProblemFunction::phi_t(arr& phi, arr& J, uint t, const arr& x_bar){
  uint n=get_n();
  CHECK(x_bar.d0==3 && x_bar.d1==n,"");
  arr q_2(x_bar,0); //this is q(t-2)
  arr q_1(x_bar,1);
  arr q_0(x_bar,2);
  double tau=P.tau;
  double _tau2=1./(tau*tau);

  //dynamics
  arr h = sqrt(P.H_rate_diag)*sqrt(tau);
  phi = h % (_tau2*(q_0-2.*q_1+q_2)); //penalize acceleration
  if(&J){ //we todoalso need to return the Jacobian
    J.resize(n,3,n);
    J.setZero();
    for(uint i=0;i<n;i++){  J(i,2,i) = 1.;  J(i,1,i) = -2.;  J(i,0,i) = 1.; }
    J *= _tau2;
    J.reshape(n,3*n);
    for(uint i=0;i<n;i++) J[i]() *= h(i);
  }

  if(&J) CHECK(J.d0==phi.N,"");

  //task phi w.r.t. q1
  arr _phi, J_x, J_v;
  P.setState(q_0, (q_0-q_1)/tau);
  P.getTaskCosts(_phi, J_x, J_v, t);
  phi.append(_phi);
  if(&J && _phi.N) {
    arr Japp(_phi.N,3*n);
    Japp.setZero();
    Japp.setMatrixBlock(J_x + ( 1./tau)*J_v, 0, 2*n); //w.r.t. q_0
    Japp.setMatrixBlock((-1./tau)*J_v, 0, n); //w.r.t. q_1
    J.append(Japp);
  }

  if(&J) CHECK(J.d0==phi.N,"");

  //store in CostMatrix
  if(!P.costMatrix.N){
    P.costMatrix.resize(get_T()+1,phi.N);
    P.costMatrix.setZero();
  }

  CHECK(P.costMatrix.d1==phi.N,"");
  P.costMatrix[t]() = phi;
}

