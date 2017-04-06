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
//===========================================================================
#include <Kin/kin.h>
#include <Gui/opengl.h>
#include "socSystem_ors.h"
#include <Plot/plot.h>

//===========================================================================
//
// SocSystem_Ors
//

struct soc::sSocSystem_Ors {
  arr q0, v0, H_rate, Q_rate, v_act;
  arr q_external;

  uint T;
  double tau;
  bool pseudoDynamic;
  bool newedOrs;
};

soc::SocSystem_Ors::SocSystem_Ors():SocSystemAbstraction::SocSystemAbstraction(){
  s = new sSocSystem_Ors;
  ors   = NULL;
  swift = NULL;
  vars = NULL;
  s->pseudoDynamic=false;
  s->newedOrs=false;
}

soc::SocSystem_Ors::~SocSystem_Ors(){
  if(s->newedOrs){
    listDelete(vars);
    delete swift;
    delete gl;
    delete ors;
  }
  delete s;
}

soc::SocSystem_Ors* soc::SocSystem_Ors::newClone(bool deep) const {
  SocSystem_Ors *sys=new SocSystem_Ors();
  sys->os = os;
  sys->dynamic = dynamic;
  sys->stepScale = stepScale;

  if(!deep){
    sys->ors   = ors;
    sys->swift = swift;
    sys->gl    = gl;
    sys->s->newedOrs = false;
  }else{
    sys->ors  =ors->newClone();
    sys->swift=swift->newClone(*sys->ors);
    if(gl){
      sys->gl =gl->newClone();
      sys->gl->remove(mlr::glDrawGraph, ors);
      sys->gl->add(mlr::glDrawGraph, sys->ors);
    }
    sys->s->newedOrs = true;
  }

  listClone(sys->vars, vars); //deep copy the task variables!

  sys->s->q0 = s->q0;
  sys->s->v0 = s->v0;
  sys->s->H_rate = s->H_rate;
  sys->s->Q_rate = s->Q_rate;
  sys->s->v_act = s->v_act;
  sys->s->T = s->T;
  sys->s->tau = s->tau;
  sys->s->pseudoDynamic = s->pseudoDynamic;
  return sys;
}

void soc::SocSystem_Ors::initBasics(mlr::KinematicWorld *_ors, SwiftInterface *_swift, OpenGL *_gl,
                                    uint trajectory_steps, double trajectory_duration, bool _dynamic, arr *W){
  if(_ors)   ors   = _ors;   else { ors=new mlr::KinematicWorld;        ors  ->init(mlr::getParameter<mlr::String>("orsFile")); } // ors->makeLinkTree(); }
  if(_swift) swift = _swift; else { swift=new SwiftInterface;  swift->init(*ors, 2.*mlr::getParameter<double>("swiftCutoff", 0.11)); }
  gl    = _gl;
  if(gl && !_ors){
    gl->add(glStandardScene);
    gl->add(mlr::glDrawGraph, ors);
    gl->camera.setPosition(5, -10, 10);
    gl->camera.focus(0, 0, 1);
    gl->camera.upright();
  }
  setTimeInterval(trajectory_duration, trajectory_steps);
  setx0ToCurrent();
  //swift->computeProxies(*ors, false); if(gl) gl->watch();
  arr W_rate;
  if(W){
    if(W->nd==1){
      if(W->N > s->q0.N){ W->resizeCopy(s->q0.N); MLR_MSG("truncating W diagonal..."); }
      CHECK_EQ(W->N,s->q0.N, "");
      W_rate.setDiag(*W);
    } else NIY;
  }else{
    ors->computeNaturalQmetric(W_rate);
    //cout <<"automatic W initialization =" <<s->W <<endl;
    //graphWriteDirected(cout, ors->bodies, ors->joints);
  }
  static mlr::Parameter<double> hr("Hrate");
  static mlr::Parameter<double> qr("Qrate", 1e-10);
  s->H_rate = hr()*W_rate;     //u-metric for torque control
  if(!_dynamic){
    dynamic=false;
    s->Q_rate.setDiag(qr, s->q0.N);  //covariance \dot q-update
  }else{
    dynamic=true;
    s->pseudoDynamic=true;
    s->Q_rate.setDiag(qr, 2*s->q0.N);  //covariance \dot q-update
  }
  stepScale.resize(s->T+1);  stepScale.setZero();
}

void soc::SocSystem_Ors::initStandardReachProblem(uint rand_seed, uint T, bool _dynamic){
  if(!T) T = mlr::getParameter<uint>("trajectoryLength");

  initBasics(NULL, NULL, NULL, T, 3., _dynamic, NULL);
  os=&std::cout;

  if(mlr::getParameter<bool>("standOnFoot")){
    ors->reconfigureRoot(ors->getBodyByName("rfoot"));
    ors->calcBodyFramesFromJoints();
  }

  if(rand_seed>0){
    rnd.seed(rand_seed);
    mlr::Body &t=*ors->getBodyByName("target");
    t.X.pos.x += .05*rnd.gauss();
    t.X.pos.y += .05*rnd.gauss();
    t.X.pos.z += .05*rnd.gauss();
  }

  //standard task variables and problem definition

  mlr::String endeffShapeName= mlr::getParameter<mlr::String>("endeffShapeName");
  double endPrec=mlr::getParameter<double>("endPrec");
  double midPrec=mlr::getParameter<double>("midPrec");
  double colPrec=mlr::getParameter<double>("colPrec");
  double balPrec=mlr::getParameter<double>("balPrec");
  double margin =mlr::getParameter<double>("margin");
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position" , *ors, posTVT, endeffShapeName, 0, arr());
  TaskVariable *col;
  if(!mlr::getParameter<bool>("useTruncation"))
    col = new DefaultTaskVariable("collision", *ors, collTVT, 0, 0, 0, 0, ARR(margin));
  else col = new DefaultTaskVariable("collision", *ors, colConTVT, 0, 0, 0, 0, ARR(margin));
  TaskVariable *com = new DefaultTaskVariable("balance", *ors, comTVT, 0, 0, 0, 0, arr());
  setTaskVariables({pos, col, com});

  pos->y_target = conv_vec2arr(ors->getShapeByName("target")->X.pos);
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

void soc::SocSystem_Ors::initStandardBenchmark(uint rand_seed){
  uint K = mlr::getParameter<uint>("segments");
  uint T = mlr::getParameter<uint>("trajectoryLength");
  dynamic = mlr::getParameter<bool>("isDynamic");
  double margin = mlr::getParameter<double>("margin");
  bool useTruncation = mlr::getParameter<bool>("useTruncation");

  //generate the configuration
  mlr::Body *b, *target, *endeff;  mlr::Shape *s;  mlr::Joint *j;
  mlr::String str;
  ors=new mlr::KinematicWorld;
  //the links
  for(uint k=0; k<=K; k++){
    b=new mlr::Body(*ors);
    b->name = STRING("body" <<k);
    if(!k) b->type=mlr::BT_static;
    s=new mlr::Shape(*ors, b);
    s->type=mlr::ST_capsule;
    s->size(0)=.0; s->size(1)=.0; s->size(2)=1./K; s->size(3)=.2/K;
    s->rel.setText(STRING("<t(0 0 " <<.5/K <<")>"));
    if(k&1){ s->color[0]=.5; s->color[1]=.2; s->color[2]=.2; } else   { s->color[0]=.2; s->color[1]=.2; s->color[2]=.2; }
    s->cont=true;
    if(k){
      j=new mlr::Joint(*ors, ors->bodies(k-1), ors->bodies(k));
      j->type = mlr::JT_hingeX;
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
  b=new mlr::Body(*ors);
  b->name = "target";
  b->X.setText("<t(.2 0 0)>");
  s=new mlr::Shape(*ors, b);
  s->read(STREAM("type=1 size=[.0 .0 .1 .02] color=[0 0 1]"));
  s=new mlr::Shape(*ors, b);
  s->read(STREAM("type=0 rel=<t(0 -.1 0)> size=[.2 .01 .3 .0] color=[0 0 0] contact"));
  s=new mlr::Shape(*ors, b);
  s->read(STREAM("type=0 rel=<t(0 .1 0)> size=[.2 .01 .3 .0] color=[0 0 0] contact"));
  s=new mlr::Shape(*ors, b);
  s->read(STREAM("type=0 rel=<t(.1 0 0)> size=[.01 .2 .3 .0] color=[0 0 0] contact"));
  graphMakeLists(ors->bodies, ors->joints);
  ors->calcBodyFramesFromJoints();
  target=b;

  if(rand_seed>0){
    rnd.seed(rand_seed);
    target->X.pos.x += .05*rnd.gauss();
    target->X.pos.y += .05*rnd.gauss();
    //target->X.p(2) += .05*rnd.gauss();
  }

  initBasics(ors, NULL, NULL, T, 3., mlr::getParameter<bool>("isDynamic"), NULL);
  os=&std::cout;

  double endPrec=mlr::getParameter<double>("endPrec");
  double midPrec=mlr::getParameter<double>("midPrec");
  double colPrec=mlr::getParameter<double>("colPrec");
  //-- setup the control variables (problem definition)
  TaskVariable *pos = new DefaultTaskVariable("position" , *ors, posTVT, endeff->name, STRING("<t(0 0 " <<.5/K <<")>"), 0, 0, arr());
  TaskVariable *col;
  if(!useTruncation) col = new DefaultTaskVariable("collision", *ors, collTVT, 0, 0, 0, 0, ARR(margin));
  else               col = new DefaultTaskVariable("collision", *ors, colConTVT, 0, 0, 0, 0, ARR(margin));
  setTaskVariables({pos, col});

  pos->y_target = conv_vec2arr(ors->getBodyByName("target")->X.pos);
  pos->setInterpolatedTargetsEndPrecisions(T, midPrec, endPrec, 0., 10*endPrec);
  if(col->type==collTVT){
    col->y        = ARR(0.);
    col->y_target = ARR(0.);
    col->setInterpolatedTargetsConstPrecisions(T, colPrec, 0.);
  } else col->active=true;
}

/* OLD VERSION!!
void soc::createEndeffectorReachProblem(SocSystem_Ors &sys,
                                        const char *ors_file,
                                        uint trajectory_length,
                                        int rand_seed)
{

  //setup the workspace
  *sys.ors <<FILE(ors_file);
  if(rand_seed>0){
    rnd.seed(rand_seed);
    mlr::Body &t=*sys.ors->getBodyByName("target");
    t.X.p(0) += .05*rnd.gauss();
    t.X.p(1) += .05*rnd.gauss();
    t.X.p(2) += .05*rnd.gauss();
  }
  sys.ors->calcNodeFramesFromEdges();
  sys.ors->reconfigureRoot(sys.ors->getBodyByName("rfoot"));
  sys.ors->getJointState(sys.s->q0, sys.s->v0);
  sys.swift->init(*sys.ors);


  sys.s->T=trajectory_length;
  sys.s->tau=.01;
  arr Wdiag(sys.s->q0.N);
  Wdiag=1.;
  MLR_MSG("Warning - need to change this");
  Wdiag  <<"[20 20 20 10 10 10 10 1 1 1 1 10 10 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 20 20 10 10 10 10 10 10 ]";
  //Wdiag  <<"[20 20 20 10 10 10 10 1 1 1 1 10 10 1 1 1 20 20 10 10 10 10 10 10 ]";
  CHECK_EQ(Wdiag.N,sys.s->q0.N, "wrong W matrix!");
  sys.s->W.setDiag(Wdiag);
  sys.s->H_rate = sys.s->W;

  //set task variables
  TaskVariable *x0, *x1, *x2;
  x0 = new TaskVariable("finger-tip", *sys.ors, posTVT , "effector", 0, 0, 0, 0);
  x1 = new TaskVariable("COM",       *sys.ors, comTVT , 0, 0, 0, 0, 0);
  x2 = new TaskVariable("collision", *sys.ors, collTVT, 0, 0, 0, 0, ARR(.02));
  sys.vars = {x0, x1, x2};

  updateState(globalSpace);
  //reportAll(globalSpace, cout);


  double midPrec, endPrec, balPrec, colPrec;
  mlr::getParameter(midPrec, "midPrec");
  mlr::getParameter(endPrec, "endPrec");
  mlr::getParameter(balPrec, "balPrec");
  mlr::getParameter(colPrec, "colPrec");

  x0->y_target.setCarray(sys.ors->getBodyByName("target")->X.p.v, 3);
  x0->setInterpolatedTargetTrajectory(sys.s->T);
  x0->setPrecisionTrajectoryFinal(sys.s->T, midPrec, endPrec);

  x1->y_target(0)=0.;
  x1->y_target(1)=-.02;
  x1->setInterpolatedTargetTrajectory(sys.s->T);
  x1->setPrecisionTrajectoryConstant(sys.s->T, balPrec);
  if(!balPrec) x1->active=false;

  x2->y_target = 0.;
  x2->setInterpolatedTargetTrajectory(sys.s->T);
  x2->setPrecisionTrajectoryConstant(sys.s->T, colPrec);
  if(!colPrec) x2->active=false;

  sys.dynamic=false;
}
*/
void soc::SocSystem_Ors::setTau(double tau){
  s->tau = tau;
}

void soc::SocSystem_Ors::setTimeInterval(double trajectory_duration,  uint trajectory_steps){
  s->T=trajectory_steps;
  s->tau=trajectory_duration/trajectory_steps;
  stepScale.resize(s->T+1);  stepScale.setZero();
}

void soc::SocSystem_Ors::setTaskVariables(const TaskVariableList& _CVlist){
  vars=_CVlist;
  uint i; TaskVariable *v;
  for_list(Type,  v,  vars) v->updateState(*ors);
}

/// report on some
void soc::SocSystem_Ors::reportOnState(ostream& os){
  os <<"OrsSocImplementat - state report:\n";
  os <<"** control variables:" <<endl;
  reportAll(vars, os);
  os <<"** proxies:" <<endl;
  ors->reportProxies(&os);
}

/// DZ: write trajectory of task variable into the file
void soc::SocSystem_Ors::recordTrajectory(const arr& q, const char *variable, const char *file){
  uint i, k, m, T=get_T();
  int ind = -1;
  uint num=T;
  m=nTasks();
  for(i=0; i<m; i++)
    if(strcmp(vars(i)->name, variable)==0){ ind=i; break; }

  if(ind<0) return;
  arr y_traj;
  y_traj.resize(T,vars(ind)->y.N);
  for(k=0; k<(uint)num; k++){
      setq(q[k]());
      vars(ind)->updateState(*ors);
      y_traj[k]() = vars(ind)->y;
   }
   ofstream out(file); y_traj.writeRaw(out); out.close();
}

//overload the display method to include variances
void soc::SocSystem_Ors::displayState(const arr *x, const arr *Qinv, const char *text, bool reportVariables){
  if(x){ if(x->N==qDim()) setq(*x); else setx(*x); }
  if(text) gl->text.clear() <<text;
  if(Qinv){
    arr Q;
    inverse_SymPosDef(Q, *Qinv);
    if(gl->drawers.last().call!=glDrawPlot) gl->add(glDrawPlot, &plotModule);
    plotClear();
    TaskVariable *v;
    uint i;
    for_list(Type,  v,  vars){
      if(v->type==posTVT){
        arr X(3, 3);
        X = v->J * Q * v->Jt;
        plotCovariance(v->y, X);
        break;
      }
    }
  }
  gl->update();
  //gl->watch();
  //gl->timedupdate(getTau()*(T-1)/(display-1));
  //if(Qinv) gl->drawers.popLast();
  if(reportVariables){
    reportAll(vars, cout);
  }
}


uint soc::SocSystem_Ors::get_T(){  return s->T>>scalePower; }
uint soc::SocSystem_Ors::get_xDim(){ if(dynamic) return 2.*qDim(); return qDim(); }
uint soc::SocSystem_Ors::nTasks(){ return vars.N; }
uint soc::SocSystem_Ors::qDim(){   return s->q0.N; }
uint soc::SocSystem_Ors::uDim(){   return qDim(); }
uint soc::SocSystem_Ors::yDim(uint i){ return vars(i)->y.N; }
double soc::SocSystem_Ors::getTau(bool scaled){
  double tau=s->tau;
  if(scaled) for(uint i=0; i<scalePower; i++) tau *=2.;
  return tau;
}

void soc::SocSystem_Ors::getq0(arr& q){ q=s->q0; }
void soc::SocSystem_Ors::setq0(const arr& q){ s->q0=q; }
void soc::SocSystem_Ors::getv0(arr& v){ v=s->v0; }
void soc::SocSystem_Ors::getx0(arr& x){
  if(!dynamic){
    x=s->q0;
  }else{
    x.setBlockVector(s->q0, s->v0);
  }
}
void soc::SocSystem_Ors::setx0(const arr& x0){ setx(x0); setx0ToCurrent(); }
void soc::SocSystem_Ors::getqv0(arr& q, arr& qd){ q=s->q0; qd=s->v0; }
void soc::SocSystem_Ors::getControlCosts(arr& _H, arr& Hinv, uint t){
  arr H;
  if(!dynamic){
    H = s->H_rate;
  }else{
    H=s->H_rate*getTau(); // control cost rate
  }
  if(stepScale(t)) H *= double(1 <<stepScale(t));
  if(&_H) _H = H;
  if(&Hinv) inverse_SymPosDef(Hinv, H);
}

void soc::SocSystem_Ors::setq(const arr& q, uint t){
  ors->setJointState(q);
  if(s->q_external.N)
    ors->setExternalState(s->q_external[t]);
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
  s->v_act.resizeAs(q);
  s->v_act.setZero();
  uint i;
  TaskVariable *v;
  for_list(Type,  v,  vars)  if(v->active)  v->updateState(*ors);
}

void soc::SocSystem_Ors::setqv(const arr& q, const arr& qd, uint t){
  ors->setJointState(q, qd);
  if(s->q_external.N)
    ors->setExternalState(s->q_external[t]);
  ors->calcBodyFramesFromJoints();
  swift->computeProxies(*ors, false);
  s->v_act=qd;
  uint i;
  TaskVariable *v;
  for_list(Type,  v,  vars)  if(v->active)  v->updateState(*ors);
}

void soc::SocSystem_Ors::setx(const arr& x, uint t){
  if(!dynamic){
    setq(x);
  }else{
    uint n=x.N/2;
    CHECK_EQ(x.N,2*n, "");
    arr q, v;
    q.referToRange(x, 0, n-1);
    v.referToRange(x, n, 2*n-1);
    setqv(q, v);
  }
}

void soc::SocSystem_Ors::setx0ToCurrent(){
  ors->getJointState(s->q0, s->v0);
  s->v0.setZero(); MLR_MSG("evil speed v0=0 hack"); //TODO
}

void soc::SocSystem_Ors::getMF(arr& M, arr& F, arr& Q, uint t){
  if(!s->pseudoDynamic){
    ors->clearForces();
    ors->gravityToForces();
    //ors->frictionToForces(1.1);
    ors->equationOfMotion(M, F, s->v_act);
    //M.setId();  F = .1;
    //Minv *= .2;//1e-1;
  }else{
    uint n=qDim();
    M.setId(n);
    F.resize(n); F.setZero();
  }
  if(&Q) Q = s->Q_rate*getTau();
  //CHECK(!stepScale(t), "NIY"); //this is taken care of in getProcess!!
}

void soc::SocSystem_Ors::getMinvF(arr& Minv, arr& F, arr& Q, uint t){
  arr M;
  getMF(M, F, Q, t);
  inverse(Minv, M);
}

bool soc::SocSystem_Ors::isConditioned(uint i, uint t){
  if(vars(i)->type==colConTVT) return false;
  return vars(i)->active;
}

bool soc::SocSystem_Ors::isConstrained(uint i, uint t){
  return vars(i)->active && vars(i)->type==colConTVT;
}

const char* soc::SocSystem_Ors::taskName(uint i){
  return vars(i)->name.p;
}

uint soc::SocSystem_Ors::taskDim(uint i){
  return vars(i)->y.N;
}

void soc::SocSystem_Ors::getPhi(arr& phiq_i, uint i){
  phiq_i=vars(i)->y;
}

void soc::SocSystem_Ors::getJqd(arr& jqd_i, uint i){
  arr q, qd;
  ors->getJointState(q, qd);
  jqd_i = vars(i)->J * qd;
}

void soc::SocSystem_Ors::getJJt(arr& J_i, arr& Jt_i, uint i){
  J_i=vars(i)->J;
  Jt_i=vars(i)->Jt;
}

void soc::SocSystem_Ors::getHessian(arr& H_i, uint i){
  vars(i)->getHessian(*ors, H_i);
}

void soc::SocSystem_Ors::getTarget(arr& y_target, double& y_prec, uint i, uint t){
  TaskVariable *v=vars(i);
  //cout <<"getting y_target for TV " <<v->name <<endl;
  if(!t && v->targetType!=trajectoryTT){
    v->updateChange(-1, s->tau);
    y_target = v->y_ref;
    y_prec   = v->y_prec;
    return;
  }
  CHECK(t<v->y_trajectory.d0, "task target trajectory for variable '" <<v->name <<"' not specified");
  y_target = v->y_trajectory[t];
  y_prec   = v->y_prec_trajectory(t);
}

void soc::SocSystem_Ors::getTargetV(arr& v_target, double& v_prec, uint i, uint t){
  TaskVariable *v=vars(i);
  //cout <<"getting v_target for TV " <<v->name <<endl;
  if(!t && v->targetType!=trajectoryTT){
    v->updateChange(-1, s->tau);
    v_target = v->v_ref;
    v_prec   = v->v_prec;
    return;
  }
  v_target = v->v_trajectory[t];
  v_prec   = v->v_prec_trajectory(t);
}


//===========================================================================
//
// problem implementations
//

void drawOrsSocEnv(void*){
  glStandardLight(NULL);
  //glDrawFloor(1., .4, .4, .4);
  //DrawAxes(1.);
}

/*void soc::setupOpenGL(SocSystem_Ors &sys){
  if(!sys.gl) sys.gl=new OpenGL();
  sys.gl->add(drawOrsSocEnv, 0);
  sys.gl->add(mlr::glDrawGraph, sys.ors);
  //sys.gl->add(plotDrawOpenGL, &plotData);
  sys.gl->camera.focus(0, 0, .8);
}*/


/*void soc::createDynamicProblem(SocSystem_Ors &sys,
                          const char *ors_file,
                          double trajectory_time,
                          uint trajectory_steps){

  //setup the workspace
  *sys.ors <<FILE(ors_file);
  sys.ors->calcBodyFramesFromJoints();
  sys.ors->getJointState(sys.s->q0, sys.s->v0);
  sys.swift->init(*sys.ors);

  uint n=sys.s->q0.N;
  sys.s->T=trajectory_steps;
  sys.s->tau=trajectory_time/trajectory_steps;
  sys.s->W.setDiag(1e-6, n);  //q-metric for inverse kinematics (initialization)
  static mlr::Parameter<double> hc("Hcost");
  static mlr::Parameter<double> qn("Qnoise");
  sys.s->H.setDiag(hc, n);  //u-metric for torque control
  sys.s->Q.setDiag(qn, 2*n);  //covariance \dot q-update

  //set task variables
  TaskVariable *x0;
  x0 = new TaskVariable("finger-tip", *sys.ors, posTVT , "eff", "t(0 0 .15)", 0, 0, 0);
  sys.vars = {x0};

  updateState(sys.vars);
  //reportAll(globalSpace, cout);

  double midPrec, endPrec;
  mlr::getParameter(midPrec, "midPrec");
  mlr::getParameter(endPrec, "endPrec");

  x0->y_target = conv_vec2arr(sys.ors->getBodyByName("target")->X.pos);
  x0->v_target <<"[2 0 0]";
  x0->setInterpolatedTargetTrajectory(sys.s->T);
  x0->setPrecisionTrajectoryFinal (sys.s->T, midPrec, endPrec);
  x0->setPrecisionVTrajectoryFinal(sys.s->T, 0., endPrec);


  sys.dynamic=true;
}

void createNikolayReachProblem(soc::SocSystem_Ors& sys,
                                   mlr::KinematicWorld& _ors,
                                   SwiftInterface& _swift,
                                   uint trajectory_length,
                                   const arr& endeffector_target,
                                   const char* endeffector_name,
                                   const arr& W){
  static soc::sSocSystem_Ors s;

  //setup the workspace
  //s.vars = globalSpace;
  sys.ors=&_ors;
  sys.ors->getJointState(s.q0, s.v0);
  s.T=trajectory_length;
  s.W=W;
  sys.swift=&_swift;

  //set task variables
  TaskVariable *x0, *x1;
  x0 = new TaskVariable("finger-tip", *sys.ors, posTVT , endeffector_name, "", 0, 0, 0);
  x1 = new TaskVariable("collision", *sys.ors, collTVT, 0, 0, 0, 0, 0);
  sys.vars = {x0, x1};

  updateState(sys.vars);
  //reportAll(globalSpace, cout);

  x0->y_prec=1e3;  x0->setGainsAsAttractor(30.);  x0->y_target = endeffector_target;
  x0->setTrajectory(s.T, 0, 1e10);
  x0->setPrecisionTrajectoryFinal(s.T, 1e1, 1e4);

  x1->y_prec=1e6;  x1->setGainsAsAttractor(30.);  x1->y_target = 0.;
  x1->setTrajectory(s.T, 0);
  x1->setPrecisionTrajectoryConstant(s.T, 1e5);

  sys.s=&s;
  sys.dynamic=false;
}*/

#if 0
//old code to give system relative to a linear transform
//now realized a level deeper within mlr::KinematicWorld
void transformSystemMatricesWithQlin(){
  //clone operator operator
  sys->Qlin = Qlin;
  sys->Qoff = Qoff;
  sys->Qinv = Qinv;

  //get q & v
  q=Qinv*(s->q0-Qoff);
  v=Qinv*s->v0;

  //set q
  q = Qlin*_q + Qoff;  qd = Qlin*_qd;

  //getJJt
  J_i=vars(i)->J*Qlin;

  //get W & H
  W = ~Qlin*s->W*Qlin;
  H = ~Qlin*s->H*Qlin;

  //get Q (noise matrix)
  arr Qbig;
  Qbig.resize(2*Qlin.d0, 2*Qlin.d1); Qbig.setZero();
  Qbig.setMatrixBlock(Qlin, 0, 0); Qbig.setMatrixBlock(Qlin, Qlin.d0, Qlin.d1);
  //cout <<Qbig <<endl;
  Q = ~Qbig*s->Q*Qbig;
}
#endif

/** @} */
