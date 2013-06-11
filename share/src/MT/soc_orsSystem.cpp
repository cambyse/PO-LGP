/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

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

/**
 * @file
 * @ingroup group_soc
 */
/**
 * @addtogroup group_soc
 * @{
 */



#include "soc_orsSystem.h"
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>

//===========================================================================
//
// OrsSystem
//

struct sOrsSystem {
  ors::Graph *ors;
  SwiftInterface *swift;
  MT::Array<TaskVariable*> vars;
  arr x, x0;
  arr v_act, H_rate, Q_rate;
  arr q_external;

  uint T;
  double tau;
  bool pseudoDynamic;
  bool newedOrs;
  bool dynamic;

  sOrsSystem():ors(NULL), swift(NULL) {}

  void getCurrentStateFromOrs();
  void setq(const arr& q);
  void setqv(const arr& q, const arr& qd);
  void getMF(arr& M, arr& F, arr& Q, uint t);
  void getMinvF(arr& Minv, arr& F, arr& Q, uint t);
  void getTarget(arr& y_i, double& prec, uint i, uint t);
  void getTargetV(arr& v_i, double& prec, uint i, uint t);
  bool isConditioned(uint i, uint t);
  void getPhi(arr& phiq_i, uint i);
  void getJJt(arr& J_i, arr& Jt_i, uint i);
  void getJqd(arr& Jqd_i, uint i);
};

OrsSystem::OrsSystem(){
  s = new sOrsSystem;
  s->pseudoDynamic=false;
  s->newedOrs=false;
}

OrsSystem::~OrsSystem(){
  if(s->newedOrs){
    listDelete(s->vars);
    delete s->swift;
    delete gl;
    delete s->ors;
  }
  delete s;
}

OrsSystem* OrsSystem::newClone(bool deep) const {
  OrsSystem *sys=new OrsSystem();
  sys->os = os;

  if(!deep){
    sys->s->ors   = s->ors;
    sys->s->swift = s->swift;
    sys->gl    = gl;
    sys->s->newedOrs = false;
  }else{
    sys->s->ors  = s->ors->newClone();
    sys->s->swift= s->swift->newClone(*sys->s->ors);
    if(gl){
      sys->gl =gl->newClone();
      sys->gl->remove(ors::glDrawGraph, s->ors);
      sys->gl->add(ors::glDrawGraph, sys->s->ors);
    }
    sys->s->newedOrs = true;
  }

  listClone(sys->s->vars, s->vars); //deep copy the task variables!

  sys->s->H_rate = s->H_rate;
  sys->s->Q_rate = s->Q_rate;
  sys->s->T = s->T;
  sys->s->tau = s->tau;
  sys->s->pseudoDynamic = s->pseudoDynamic;
  return sys;
}

void OrsSystem::initBasics(ors::Graph *_ors, SwiftInterface *_swift, OpenGL *_gl,
                                    uint trajectory_steps, double trajectory_duration, bool _dynamic, arr *W){
  if(_ors)   s->ors   = _ors;   else { s->ors=new ors::Graph;        s->ors  ->init(MT::getParameter<MT::String>("orsFile")); } // ors->makeLinkTree(); }
  if(_swift) s->swift = _swift; else { s->swift=new SwiftInterface;  s->swift->init(*s->ors, 2.*MT::getParameter<double>("swiftCutoff", 0.11)); }
  gl    = _gl;
  if(gl && !_ors){
    gl->add(glStandardScene);
    gl->add(ors::glDrawGraph, s->ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
  }
  s->dynamic = _dynamic;
  setTimeInterval(trajectory_duration, trajectory_steps);
  s->getCurrentStateFromOrs();
  s->x0 = s->x;
  //swift->computeProxies(*ors, false); if(gl) gl->watch();
  arr W_rate;
  if(W){
    if(W->nd==1){
      if(W->N > get_qDim()){ W->resizeCopy(get_qDim()); MT_MSG("truncating W diagonal..."); }
      CHECK(W->N==get_qDim(), "");
      W_rate.setDiag(*W);
    } else NIY;
  }else{
    s->ors->computeNaturalQmetric(W_rate);
    //cout <<"automatic W initialization =" <<s->W <<endl;
    //graphWriteDirected(cout, ors->bodies, ors->joints);
  }
  static MT::Parameter<double> hr("Hrate");
  static MT::Parameter<double> qr("Qrate", 1e-10);
  s->H_rate = hr()*W_rate;     //u-metric for torque control
  s->Q_rate.setDiag(qr, get_xDim());  //covariance \dot q-update
  if(s->dynamic) s->pseudoDynamic=true;
}

void OrsSystem::initStandardReachProblem(uint rand_seed, uint T, bool _dynamic){
  if(!T) T = MT::getParameter<uint>("trajectoryLength");

  initBasics(NULL, NULL, NULL, T, 3., _dynamic, NULL);
  os=&std::cout;

  if(MT::getParameter<bool>("standOnFoot")){
    s->ors->reconfigureRoot(s->ors->getBodyByName("rfoot"));
    s->ors->calcBodyFramesFromJoints();
  }

  if(rand_seed>0){
    rnd.seed(rand_seed);
    ors::Body &t=*s->ors->getBodyByName("target");
    t.X.pos.x += .05*rnd.gauss();
    t.X.pos.y += .05*rnd.gauss();
    t.X.pos.z += .05*rnd.gauss();
  }

  //standard task variables and problem definition

  MT::String endeffShapeName= MT::getParameter<MT::String>("endeffShapeName");
  double endPrec=MT::getParameter<double>("endPrec");
  double midPrec=MT::getParameter<double>("midPrec");
  double colPrec=MT::getParameter<double>("colPrec");
  double balPrec=MT::getParameter<double>("balPrec");
  double margin =MT::getParameter<double>("margin");
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position" , *s->ors, posTVT, endeffShapeName, 0, ARR());
  TaskVariable *col;
  if(!MT::getParameter<bool>("useTruncation"))
    col = new DefaultTaskVariable("collision", *s->ors, collTVT, 0, 0, 0, 0, ARR(margin));
  else col = new DefaultTaskVariable("collision", *s->ors, colConTVT, 0, 0, 0, 0, ARR(margin));
  TaskVariable *com = new DefaultTaskVariable("balance", *s->ors, comTVT, 0, 0, 0, 0, ARR());
  setTaskVariables(ARRAY(pos, col, com));

  pos->y_target = ARRAY(s->ors->getShapeByName("target")->X.pos);
  pos->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 10*endPrec);
  if(col->type==collTVT){
    col->y        = ARR(0.);
    col->y_target = ARR(0.);
    col->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  } else col->active=true;
  if(balPrec){
    com->y_target = com->y;
    com->setInterpolatedTargetsConstPrecisions(T, balPrec, 0.);
  } else com->active=false;
}

void OrsSystem::initStandardBenchmark(uint rand_seed){
  uint K = MT::getParameter<uint>("segments");
  uint T = MT::getParameter<uint>("trajectoryLength");
  s->dynamic = MT::getParameter<bool>("isDynamic");
  double margin = MT::getParameter<double>("margin");
  bool useTruncation = MT::getParameter<bool>("useTruncation");

  //generate the configuration
  ors::Body *b, *target, *endeff;  ors::Shape *sh;  ors::Joint *j;
  MT::String str;
  s->ors=new ors::Graph;
  //the links
  for(uint k=0; k<=K; k++){
    b=new ors::Body(*s->ors);
    b->name = STRING("body" <<k);
    if(!k) b->type=ors::staticBT;
    sh=new ors::Shape(*s->ors, b);
    sh->type=ors::cappedCylinderST;
    sh->size[0]=.0; sh->size[1]=.0; sh->size[2]=1./K; sh->size[3]=.2/K;
    sh->rel.setText(STRING("<t(0 0 " <<.5/K <<")>"));
    if(k&1){ sh->color[0]=.5; sh->color[1]=.2; sh->color[2]=.2; } else   { sh->color[0]=.2; sh->color[1]=.2; sh->color[2]=.2; }
    sh->cont=true;
    if(k){
      j=new ors::Joint(*s->ors, s->ors->bodies(k-1), s->ors->bodies(k));
      j->type = ors::JT_hingeX;
      j->Q.setText("<d(45 1 0 0)>");
      if(k&1){ //odd -> rotation around z
        j->A.setText(STRING("<t(0 0 " <<1./K <<") d(-90 0 1 0)>"));
        j->B.setText("<d(90 0 1 0)>");
      }else{ //even -> rotation around x
        j->A.setText(STRING("<t(0 0 " <<1./K <<")>"));
      }
    }
  }
  endeff=b;
  //the target
  b=new ors::Body(*s->ors);
  b->name = "target";
  b->X.setText("<t(.2 0 0)>");
  sh=new ors::Shape(*s->ors, b);
  sh->read(STREAM("type=1 size=[.0 .0 .1 .02] color=[0 0 1]"));
  sh=new ors::Shape(*s->ors, b);
  sh->read(STREAM("type=0 rel=<t(0 -.1 0)> size=[.2 .01 .3 .0] color=[0 0 0] contact"));
  sh=new ors::Shape(*s->ors, b);
  sh->read(STREAM("type=0 rel=<t(0 .1 0)> size=[.2 .01 .3 .0] color=[0 0 0] contact"));
  sh=new ors::Shape(*s->ors, b);
  sh->read(STREAM("type=0 rel=<t(.1 0 0)> size=[.01 .2 .3 .0] color=[0 0 0] contact"));
  graphMakeLists(s->ors->bodies, s->ors->joints);
  s->ors->calcBodyFramesFromJoints();
  target=b;

  if(rand_seed>0){
    rnd.seed(rand_seed);
    target->X.pos.x += .05*rnd.gauss();
    target->X.pos.y += .05*rnd.gauss();
    //target->X.p(2) += .05*rnd.gauss();
  }

  initBasics(s->ors, NULL, NULL, T, 3., MT::getParameter<bool>("isDynamic"), NULL);
  os=&std::cout;

  double endPrec=MT::getParameter<double>("endPrec");
  double midPrec=MT::getParameter<double>("midPrec");
  double colPrec=MT::getParameter<double>("colPrec");
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position" , *s->ors, posTVT, endeff->name, STRING("<t(0 0 " <<.5/K <<")>"), 0, 0, ARR());
  TaskVariable *col;
  if(!useTruncation) col = new DefaultTaskVariable("collision", *s->ors, collTVT, 0, 0, 0, 0, ARR(margin));
  else               col = new DefaultTaskVariable("collision", *s->ors, colConTVT, 0, 0, 0, 0, ARR(margin));
  setTaskVariables(ARRAY(pos, col));

  pos->y_target = ARRAY(s->ors->getBodyByName("target")->X.pos);
  pos->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 10*endPrec);
  if(col->type==collTVT){
    col->y        = ARR(0.);
    col->y_target = ARR(0.);
    col->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  } else col->active=true;
}

void OrsSystem::setTau(double tau){
  s->tau = tau;
}

void OrsSystem::setx0ToCurrent(){
  s->x0 = s->x;
}

void OrsSystem::setTox0(){
  setx(s->x0);
}

void OrsSystem::setx0(const arr& x0){
  s->x0 = x0;
}

void OrsSystem::setTimeInterval(double trajectory_duration,  uint trajectory_steps){
  s->T=trajectory_steps;
  s->tau=trajectory_duration/trajectory_steps;
}

void OrsSystem::setTaskVariables(const TaskVariableList& _CVlist){
  s->vars=_CVlist;
  uint i; TaskVariable *v;
  for_list(i, v, s->vars) v->updateState(*s->ors);
}

//overload the display method to include variances
void OrsSystem::displayCurrentState(const char *title, bool pause, bool reportVariables){
  if(title) gl->text.clear() <<title;
  gl->update();
  if(pause)  gl->watch();
  if(reportVariables) reportAll(s->vars, cout);
}


bool OrsSystem::isKinematic(){  return !s->dynamic; }
uint OrsSystem::get_T(){  return s->T; }
double OrsSystem::get_tau(){  return s->tau; }
uint OrsSystem::get_xDim(){ return s->x.N; }
//uint OrsSystem::nTasks(){ return vars.N; }
uint OrsSystem::get_qDim(){ if(isKinematic()) return get_xDim(); return get_xDim()/2; }
uint OrsSystem::get_uDim(){ if(isKinematic()) return get_xDim(); return get_xDim()/2; }
//uint OrsSystem::yDim(uint i){ return vars(i)->y.N; }

void OrsSystem::get_x0(arr& x){ x=s->x0; }

void OrsSystem::getControlCosts(arr& _H, arr& Hinv, uint t){
  arr H;
  if(!s->dynamic){
    H = s->H_rate;
  }else{
    H=s->H_rate*get_tau(); // control cost rate
  }
  if(&_H) _H = H;
  if(&Hinv) inverse_SymPosDef(Hinv, H);
}

void sOrsSystem::setq(const arr& q){
  v_act.resizeAs(q);
  v_act.setZero();
  ors->setJointState(q);
  if(q_external.N)
    ors->setExternalState(q_external[0]);
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
  uint i;
  TaskVariable *v;
  for_list(i, v, vars)  if(v->active)  v->updateState(*ors);
}

void sOrsSystem::setqv(const arr& q, const arr& qd){
  v_act = qd;
  ors->setJointState(q, qd);
  if(q_external.N)
    ors->setExternalState(q_external[0]);
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
  uint i;
  TaskVariable *v;
  for_list(i, v, vars)  if(v->active)  v->updateState(*ors);
}

void OrsSystem::getDynamics(arr& A, arr& a, arr& B, arr& Q, uint t){
  uint n=get_qDim();
  if(isKinematic()){
    A.setId(n);
    B.setId(n);
    a.resize(n);
    a.setZero();
    if(&Q) Q.setDiag(1e-10,n);
  }else{
    double tau=get_tau();
    arr I, Z, Minv, F;
    I.setId(n);
    Z.resize(n, n); Z.setZero();

    s->getMinvF(Minv, F, Q, t);

    A.setBlockMatrix(I, tau*I, Z, I);
    //double alpha = .1; //with fricion
    //A.setBlockMatrix(I, tau*I-tau*alpha*Minv, Z, I-tau*alpha*Minv);

    B.resize(2*n, n);
    B.setZero();
    B.setMatrixBlock(.5*tau*tau*Minv, 0, 0);
    B.setMatrixBlock(tau*Minv, n, 0);

    a.resize(2*n);
    a.setZero();
    a.setVectorBlock(.5*tau*tau*Minv*F, 0);
    a.setVectorBlock(tau*Minv*F, n);
  }
}

void OrsSystem::getDynamics(arr& A, arr& tA, arr& Ainv, arr& invtA, arr& a, arr& B, arr& tB, arr& Q, uint t){
  getDynamics(A, a, B, Q, t);
  //A^{-1} is A transpose and the lower-left matrix negative.. BLOCKMATRIX(Id, -2^scale*tau*Id, 0, Id)
  Ainv=A;
  if(!isKinematic()){
    uint n=get_qDim();
    for(uint i=0; i<n; i++) Ainv(i, n+i) *= -1.;
  }
  transpose(tA, A);
  transpose(tB, B);
  transpose(invtA, Ainv);
}

void OrsSystem::getTaskCosts(arr& phiBar, arr& JBar, uint t){
  uint i, m=s->vars.N;
  phiBar.clear();
  if(&JBar) JBar.clear();
  arr phi_q, phi_v, Jac, JacT, y, v;
  double prec, precv;
  for(i=0; i<m; i++) if(s->isConditioned(i, t)){
    s->getPhi(phi_q, i);
    s->getTarget(y, prec, i, t);
    s->getJJt(Jac, JacT, i);
    phiBar.append(sqrt(prec)*(phi_q - y));
    if(s->dynamic){
      s->getJqd(phi_v, i);
      s->getTargetV(v, precv, i, t);
      //CHECK(xt.N==2*Jac.d1, ""); //x is a dynamic state
      //phi_v = Jac * xt.sub(Jac.d1, -1); //task velocity is J*q_vel;
      phiBar.append(sqrt(precv)*(phi_v - v));

      if(&JBar){
        arr tmp;
        tmp.resize(2*y.N, 2*Jac.d1);  tmp.setZero();
        tmp.setMatrixBlock(sqrt(prec)*Jac, 0, 0);
        tmp.setMatrixBlock(sqrt(precv)*Jac, y.N, Jac.d1);
        JBar.append(tmp);
      }
    }else{
      if(&JBar) JBar.append(sqrt(prec)*Jac);
    }
  }
  if(&JBar) JBar.reshape(phiBar.N, get_xDim());

  //cout <<"t=" <<t <<"phi=" <<phiBar <<"J=" <<JBar <<endl;
  //if(checkGrad && rnd.uni()<checkGrad) testGradientsInCurrentState(xt,t);
}

arr& OrsSystem::getx(){ return s->x; }

void OrsSystem::setx(const arr& x){
  s->x = x;
  if(!s->dynamic){
    s->setq(x);
  }else{
    uint n=x.N/2;
    CHECK(x.N==2*n, "");
    arr q, v;
    q.referToSubRange(x, 0, n-1);
    v.referToSubRange(x, n, 2*n-1);
    s->setqv(q, v);
  }
}

void sOrsSystem::getCurrentStateFromOrs(){
  arr q0,v0;
  ors->getJointState(q0, v0);
  v0.setZero(); //MT_MSG("evil speed v0=0 hack"); //TODO
  x.setBlockVector(q0, v0);
}

void sOrsSystem::getMF(arr& M, arr& F, arr& Q, uint t){
  if(!pseudoDynamic){
    ors->clearForces();
    ors->gravityToForces();
    //ors->frictionToForces(1.1);
    ors->equationOfMotion(M, F, v_act);
    //M.setId();  F = .1;
    //Minv *= .2;//1e-1;
  }else{
    uint n=x.d0/2;
    M.setId(n);
    F.resize(n); F.setZero();
  }
  if(&Q) Q = Q_rate*tau;
  //CHECK(!stepScale(t), "NIY"); //this is taken care of in getProcess!!
}

void sOrsSystem::getMinvF(arr& Minv, arr& F, arr& Q, uint t){
  arr M;
  getMF(M, F, Q, t);
  inverse(Minv, M);
}

bool sOrsSystem::isConditioned(uint i, uint t){
  if(vars(i)->type==colConTVT) return false;
  return vars(i)->active;
}

/* bool OrsSystem::isConstrained(uint i, uint t){
  return vars(i)->active && vars(i)->type==colConTVT;
}

const char* OrsSystem::taskName(uint i){
}
*/

uint OrsSystem::get_phiDim(uint t){
  uint m=0;
  for(uint i=0; i<s->vars.N; i++) if(s->isConditioned(i, t)){
    m+=s->vars(i)->y.N;
  }
  if(s->dynamic) return 2*m;
  return m;
}

void sOrsSystem::getPhi(arr& phiq_i, uint i){
  phiq_i=vars(i)->y;
}

void sOrsSystem::getJqd(arr& jqd_i, uint i){
  arr q, qd;
  ors->getJointState(q, qd);
  jqd_i = vars(i)->J * qd;
}

void sOrsSystem::getJJt(arr& J_i, arr& Jt_i, uint i){
  J_i=vars(i)->J;
  Jt_i=vars(i)->Jt;
}

/*void OrsSystem::getHessian(arr& H_i, uint i){
  vars(i)->getHessian(H_i);
}*/

void sOrsSystem::getTarget(arr& y_target, double& y_prec, uint i, uint t){
  TaskVariable *v=vars(i);
  //cout <<"getting y_target for TV " <<v->name <<endl;
  if(!t && v->targetType!=trajectoryTT){
    v->updateChange(-1, tau);
    y_target = v->y_ref;
    y_prec   = v->y_prec;
    return;
  }
  CHECK(t<v->y_trajectory.d0, "task target trajectory for variable '" <<v->name <<"' not specified");
  y_target = v->y_trajectory[t];
  y_prec   = v->y_prec_trajectory(t);
}

void sOrsSystem::getTargetV(arr& v_target, double& v_prec, uint i, uint t){
  TaskVariable *v=vars(i);
  //cout <<"getting v_target for TV " <<v->name <<endl;
  if(!t && v->targetType!=trajectoryTT){
    v->updateChange(-1, tau);
    v_target = v->v_ref;
    v_prec   = v->v_prec;
    return;
  }
  v_target = v->v_trajectory[t];
  v_prec   = v->v_prec_trajectory(t);
}

void OrsSystem::getTaskCostInfos(uintA& dims, MT::Array<MT::String>& names, uint t){
  uint i, m=s->vars.N;
  dims.clear();
  names.clear();
  for(i=0; i<m; i++) if(s->isConditioned(i, t)){
    names.append(s->vars(i)->name);
    dims.append(s->vars(i)->y.N);
  }
}

ors::Graph& OrsSystem::getOrs(){ return *s->ors; }

SwiftInterface& OrsSystem::getSwift(){ return *s->swift; }

MT::Array<TaskVariable*>& OrsSystem::vars(){ return s->vars; }

/** @} */
