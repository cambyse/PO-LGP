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



#include "motion.h"
#include "taskMaps.h"
#include <Gui/opengl.h>
#include <Ors/ors_swift.h>
#include <climits>
#include <iomanip>


//===========================================================================
//#define T T //(T+1)
//#define t<T (t<T) //(t<=T)
//===========================================================================

void Task::setCostSpecs(int fromTime,
                        uint toTime,
                        const arr& _target,
                        double _prec){
  if(&_target) target = _target; else target = {0.};
  if(fromTime<0) fromTime=0;
  CHECK((int)toTime>=fromTime,"");
  prec.resize(toTime+1).setZero();
  for(uint t=fromTime;t<=toTime;t++) prec(t) = _prec;
}


//===========================================================================

Task* Task::newTask(const Node* specs, const ors::KinematicWorld& world, uint Tinterval, uint Tzero){
  if(specs->parents.N<2) return NULL; //these are not task specs

  //-- check the term type first
  TermType termType;
  mlr::String& tt=specs->parents(0)->keys.last();
  if(tt=="MinSumOfSqr") termType=sumOfSqrTT;
  else if(tt=="LowerEqualZero") termType=ineqTT;
  else if(tt=="EqualZero") termType=eqTT;
  else return NULL;

  //-- try to crate a map
  TaskMap *map = TaskMap::newTaskMap(specs, world);
  if(!map) return NULL;

  //-- create a task
  Task *task = new Task(map, termType);

  //-- check for additional continuous parameters
  if(specs->isGraph()){
    const Graph& params = specs->graph();
    arr time = params.get<arr>("time",{0.,1.});
    task->setCostSpecs(Tzero + time(0)*Tinterval, Tzero + time(1)*Tinterval, params.get<arr>("target", {}), params.get<double>("scale", {1.}));
  }else{
    task->setCostSpecs(Tzero, Tzero+Tinterval, {}, 1.);
  }
  return task;
}


//===========================================================================

MotionProblem::MotionProblem(ors::KinematicWorld& originalWorld, bool useSwift)
    : world(originalWorld) , useSwift(useSwift), T(0), tau(0.), k_order(2), gl(NULL)
{
  if(useSwift) {
    makeConvexHulls(originalWorld.shapes);
    originalWorld.swift().setCutoff(2.*mlr::getParameter<double>("swiftCutoff", 0.11));
  }
  computeMeshNormals(originalWorld.shapes);
  setTiming(mlr::getParameter<uint>("timeSteps", 50), mlr::getParameter<double>("duration", 5.));
}

MotionProblem::~MotionProblem(){
  if(gl) delete gl;
  listDelete(configurations);
}

MotionProblem& MotionProblem::operator=(const MotionProblem& other) {
  HALT("does the following work and make sense?");
  world = other.world; //const_cast<const ors::KinematicWorld&>(other.world);
  useSwift = other.useSwift;
  tasks = other.tasks;
  T = other.T;
  tau = other.tau;
  k_order = other.k_order;
  phiMatrix = other.phiMatrix;
  dualSolution = other.dualSolution;
  ttMatrix = other.ttMatrix;
  return *this;
}

void MotionProblem::setTiming(uint steps, double duration){
  T = steps;
  CHECK(T, "deprecated");
  if(T) tau = duration/T; else tau=duration;
//  setupConfigurations();
}

Task* MotionProblem::addTask(const char* name, TaskMap *m, const TermType& termType){
  Task *t = new Task(m, termType);
  t->name=name;
  tasks.append(t);
  return t;
}

bool MotionProblem::parseTask(const Node *n, int Tinterval, uint Tzero){
  if(Tinterval==-1) Tinterval=T;
  //-- task?
  Task *task = Task::newTask(n, world, Tinterval, Tzero);
  if(task){
    if(n->keys.N) task->name=n->keys.last(); else{
      for(Node *p:n->parents) task->name <<'_' <<p->keys.last();
    }
    tasks.append(task);
    return true;
  }
  //-- switch?
  ors::KinematicSwitch *sw = ors::KinematicSwitch::newSwitch(n, world, Tinterval, Tzero);
  if(sw){
    switches.append(sw);
    return true;
  }
  return false;
}

void MotionProblem::parseTasks(const Graph& specs, int Tinterval, uint Tzero){
  for(Node *n:specs) parseTask(n, Tinterval, Tzero);

  //-- add TransitionTask for InvKinematics
  if(!T){
    TaskMap *map = new TaskMap_qItself();
    map->order = 0;
    Task *task = new Task(map, sumOfSqrTT);
    task->name="InvKinTransition";
    task->setCostSpecs(0, 0, world.q, 1./(tau*tau));
    tasks.append(task);
  }
}

uint MotionProblem::dim_phi(uint t) {
  uint m=0;
  for(Task *c: tasks) {
    if(c->active && c->prec.N>t && c->prec(t))
      m += c->map.dim_phi(configurations.refRange(t,t+k_order), t); //counts also constraints
  }
  return m;
}

uint MotionProblem::dim_g(uint t) {
  uint m=0;
  for(Task *c: tasks) {
    if(c->type==ineqTT && c->active && c->prec.N>t && c->prec(t))
      m += c->map.dim_phi(configurations.refRange(t,t+k_order), t);
  }
  return m;
}

uint MotionProblem::dim_h(uint t) {
  uint m=0;
  for(Task *c: tasks) {
    if(c->type==eqTT && c->active && c->prec.N>t && c->prec(t))
      m += c->map.dim_phi(configurations.refRange(t,t+k_order), t);
  }
  return m;
}

void MotionProblem::setupConfigurations(){

  //IMPORTANT: The configurations need to include the k prefix configurations!
  //Therefore configurations(0) is for time=-k and configurations(k+t) is for time=t
  CHECK(!configurations.N,"why setup again?");
//    listDelete(configurations);

  configurations.append(new ors::KinematicWorld())->copy(world, true);
  for(uint s=1;s<k_order+T;s++){
    configurations.append(new ors::KinematicWorld())->copy(*configurations(s-1), true);
    CHECK(configurations(s)==configurations.last(), "");
    //apply potential graph switches
    for(ors::KinematicSwitch *sw:switches){
      if(sw->timeOfApplication+k_order==s){
        sw->apply(*configurations(s));
        //          if(MP.useSwift) configurations(t)->swift().initActivations(*configurations(t));
      }
    }
  }
}

void MotionProblem::set_x(const arr& x){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");

  //-- set the configurations' states
  uint x_count=0;
  for(uint t=0;t<T;t++){
    uint s = t+k_order;
    uint x_dim = dim_x(t); //configurations(s)->getJointStateDimension();
//    temporallyAlignKinematicSwitchesInConfiguration(t); //this breaks the jacobian check
    if(x_dim){
      if(x.nd==1) configurations(s)->setJointState(x.refRange(x_count, x_count+x_dim-1));
      else        configurations(s)->setJointState(x[t]);
      if(useSwift) configurations(s)->stepSwift();
      x_count += x_dim;
    }
  }
  CHECK_EQ(x_count, x.N, "");
}

/// this sets the t'th configuration and then redefines all joints as fixed -> no DOFs anymore in this time slice
void MotionProblem::set_fixConfiguration(const arr& x, uint t){
  if(!configurations.N) setupConfigurations();
  CHECK(t<T,"");
  ors::KinematicWorld *W=configurations(t+k_order);
  W->setJointState(x);
  if(useSwift) W->stepSwift();
  W->zeroGaugeJoints();
  for(ors::Joint *j:W->joints) j->type = ors::JT_rigid;
  W->meldFixedJoints();
}

void MotionProblem::temporallyAlignKinematicSwitchesInConfiguration(uint t){
  for(ors::KinematicSwitch *sw:switches) if(sw->timeOfApplication<=t){
    sw->temporallyAlign(*configurations(t+k_order-1), *configurations(t+k_order), sw->timeOfApplication==t);
  }
}

void MotionProblem::displayTrajectory(int steps, const char* tag, double delay){
  if(!gl){
    gl = new OpenGL ("MotionProblem display");
    gl->camera.setDefault();
  }

  bool watch = (steps==-1);
  if(steps==1 || steps==-1){
    steps=1;
  }else{
    steps = T/steps;
  }

  for(uint t=0; t<T; t+=steps) {
    gl->clear();
    gl->add(glStandardScene, 0);
    gl->addDrawer(configurations(t+k_order));
    if(delay<0.){
      if(delay<-10.) FILE("z.graph") <<*configurations(t+k_order);
      gl->watch(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
    }else{
      gl->update(STRING(tag <<" (time " <<std::setw(3) <<t <<'/' <<T <<')').p);
      if(delay) mlr::wait(delay);
    }
  }
  if(watch)
    gl->watch(STRING(tag <<" (time " <<std::setw(3) <<T <<'/' <<T <<')').p);
}

void MotionProblem::phi_t(arr& phi, arr& J, TermTypeA& tt, uint t) {
#if 0
  phi.clear();
  if(&tt) tt.clear();
  if(&J) J.clear();
#endif
  arr y, Jy, Jtmp;
  uint dimPhi_t=0;
  for(Task *task: tasks) if(task->active && task->prec.N>t && task->prec(t)){
    task->map.phi(y, (&J?Jy:NoArr), configurations.refRange(t,t+k_order), tau, t);
    if(!y.N) continue;
    dimPhi_t += y.N;
    if(absMax(y)>1e10) MLR_MSG("WARNING y=" <<y);

    //linear transform (target shift)
    if(task->target.N==1) y -= task->target.elem(0);
    else if(task->target.nd==1) y -= task->target;
    else if(task->target.nd==2) y -= task->target[t];
    y *= sqrt(task->prec(t));
    phi.append(y);

    if(&J){
      Jy *= sqrt(task->prec(t));
      Jtmp.append(Jy);
    }

    if(&tt) for(uint i=0;i<y.N;i++) tt.append(task->type);
  }
  if(&J && dimPhi_t){
    Jtmp.reshape(dimPhi_t, Jtmp.N/dimPhi_t);
    if(t<k_order) Jtmp.delColumns(0,(k_order-t)*configurations(0)->q.N); //delete the columns that correspond to the prefix!!
    J.append(Jtmp);
  }

  CHECK_EQ(dimPhi_t, dim_phi(t), "");

  //memorize for report
  if(!phiMatrix.N) phiMatrix.resize(T);
  phiMatrix(t) = phi;
  if(&tt){
    if(!ttMatrix.N) ttMatrix.resize(T);
    ttMatrix(t) = tt;
  }
}

StringA MotionProblem::getPhiNames(uint t){
  StringA names(dim_phi(t));
  uint m=0;
  for(Task *c: tasks) if(c->active && c->prec.N>t && c->prec(t)){
    if(c->type==sumOfSqrTT) {
      uint d = c->map.dim_phi(configurations.refRange(t,t+k_order), t); //counts also constraints
      for(uint i=0;i<d;i++){
        names(m+i)=c->name;
        names(m+i) <<"_f" <<i;
      }
      m+=d;
    }
  }
  for(Task *c: tasks) if(c->active && c->prec.N>t && c->prec(t)){
    if(c->type==ineqTT) {
      uint d = c->map.dim_phi(configurations.refRange(t,t+k_order), t); //counts also constraints
      for(uint i=0;i<d;i++){
        names(m+i)=c->name;
        names(m+i) <<"_g" <<i;
      }
      m+=d;
    }
  }
  CHECK_EQ(m , names.N,"");
  return names;
}

void MotionProblem::reportFull(bool brief, ostream& os) {
  os <<"*** MotionProblem -- FeatureReport " <<endl;

  os <<"  useSwift=" <<useSwift <<endl;
  os <<"  T=" <<T <<endl;
  os <<"  tau=" <<tau <<endl;
  os <<"  k_order=" <<k_order <<endl;
  os <<"  TASKS (time idx dim name order type target scale ttMatrix phiMatrix):" <<endl;

  if(!configurations.N) setupConfigurations();

  //-- collect all task costs and constraints
  for(uint t=0; t<T; t++){
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task *c = tasks(i);
      if(!c->isActive(t)) continue;
      uint d=c->map.dim_phi(configurations.refRange(t,t+k_order), t);
      if(brief){
        if(d){
          os <<"  " <<t <<' ' <<i <<' ' <<d
              <<' ' <<std::setw(10) <<c->name
             <<' ' <<c->map.order <<' ' <<c->type <<' ';
          if(c->target.N<5) os <<'[' <<c->target <<']'; else os<<"[..]";
          os <<' ' <<c->prec(t);
          if(ttMatrix.N){
            os <<' ' <<ttMatrix(t).elem(m)
                <<' ' <<sumOfSqr(phiMatrix(t).refRange(m,m+d-1));
          }
          os <<endl;
        }
      }else{
        for(uint i=0;i<d;i++){
          os <<"  " <<t <<' ' <<i
              <<' ' <<std::setw(10) <<c->name
             <<' ' <<c->map.order <<' ' <<c->type <<' ';
          if(c->target.N==1) os <<c->target.elem(0);
          else if(c->target.nd==1) os <<c->target(i);
          else if(c->target.nd==2) os <<c->target(t,i);
          else os <<"00";
          os <<' ' <<c->prec(t);
          if(ttMatrix.N){
            os <<' ' <<ttMatrix(t)(m+i)
                <<' ' <<phiMatrix(t)(m+i);
          }
          os <<endl;
        }
      }
      m += d;
    }
    if(phiMatrix.N) CHECK_EQ(m , phiMatrix(t).N, "");
  }

  os <<"  SWITCHES: " <<switches.N <<endl;
  for(ors::KinematicSwitch *sw:switches){
    if(sw->timeOfApplication+k_order<configurations.N)
      os <<sw->shortTag(configurations(sw->timeOfApplication+k_order));
    else
      os <<sw->shortTag(NULL);
  }

}

void MotionProblem::costReport(bool gnuplt) {
  cout <<"*** MotionProblem -- CostReport" <<endl;
  if(phiMatrix.N!=T){
    CHECK(phiMatrix.N==0,"");
    phiMatrix.resize(T);
  }

  arr plotData(T,tasks.N); plotData.setZero();

  //-- collect all task costs and constraints
  double a;
  arr taskC(tasks.N); taskC.setZero();
  arr taskG(tasks.N); taskG.setZero();
  for(uint t=0; t<T; t++){
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task *c = tasks(i);
      if(!c->isActive(t)) continue;
      uint d=c->map.dim_phi(configurations.refRange(t,t+k_order), t);
      if(ttMatrix.N) for(uint i=0;i<d;i++) CHECK(ttMatrix(t)(m+i)==c->type,"");
      if(d){
        if(c->type==sumOfSqrTT){
          taskC(i) += a = sumOfSqr(phiMatrix(t).sub(m,m+d-1));
          plotData(t,i) = a;
        }
        if(c->type==ineqTT){
          double gpos=0.,gall=0.;
          for(uint j=0;j<d;j++){
            double g=phiMatrix(t)(m+j);
            if(g>0.) gpos+=g;
            gall += g;
          }
          taskG(i) += gpos;
          plotData(t,i) = gpos; //gall;
        }
        if(c->type==eqTT){
          double gpos=0.,gall=0.;
          for(uint j=0;j<d;j++){
            double h=phiMatrix(t)(m+j);
            gpos+=fabs(h);
            gall += h;
          }
          taskG(i) += gpos;
          plotData(t,i) = gpos; //all;
        }
        m += d;
      }
    }
    CHECK_EQ(m , phiMatrix(t).N, "");
  }

  //-- generate output
  cout <<" * task costs:" <<endl;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task *c = tasks(i);
    cout <<"\t '" <<c->name <<"' order=" <<c->map.order <<" type=" <<c->type;
    cout <<" \tcosts=" <<taskC(i) <<" \tconstraints=" <<taskG(i) <<endl;
    totalC += taskC(i);
    totalG += taskG(i);
  }

  cout <<"\t total task        = " <<totalC <<endl;
  cout <<"\t total constraints = " <<totalG <<endl;

  //-- write a nice gnuplot file
  ofstream fil("z.costReport");
  //first line: legend
  for(auto c:tasks){
//    uint d=c->map.dim_phi(world);
    fil <<c->name <<' '; // <<'[' <<d <<"] ";
  }
  for(auto c:tasks){
    if(c->type==ineqTT && dualSolution.N){
      fil <<c->name <<"_dual ";
    }
  }
  fil <<endl;
  //rest: just the matrix?
  if(!dualSolution.N){
    plotData.write(fil,NULL,NULL,"  ");
  }else{
    dualSolution.reshape(T, dualSolution.N/(T));
    catCol(plotData, dualSolution).write(fil,NULL,NULL,"  ");
  }
  fil.close();

  ofstream fil2("z.costReport.plt");
  fil2 <<"set key autotitle columnheader" <<endl;
  fil2 <<"set title 'costReport ( plotting sqrt(costs) )'" <<endl;
  fil2 <<"plot 'z.costReport' \\" <<endl;
  for(uint i=1;i<=tasks.N;i++) fil2 <<(i>1?"  ,''":"     ") <<" u 0:"<<i<<" w l lw 3 lc " <<i <<" lt " <<1-((i/10)%2) <<" \\" <<endl;
  if(dualSolution.N) for(uint i=0;i<tasks.N;i++) fil2 <<"  ,'' u 0:"<<1+tasks.N+i<<" w l \\" <<endl;
  fil2 <<endl;
  fil2.close();

  if(gnuplt) gnuplot("load 'z.costReport.plt'");
}

Graph MotionProblem::getReport() {
  if(phiMatrix.N!=T){
    CHECK(phiMatrix.N==0,"");
    phiMatrix.resize(T);
  }

  //-- collect all task costs and constraints
  arr taskC(tasks.N); taskC.setZero();
  arr taskG(tasks.N); taskG.setZero();
  for(uint t=0; t<T; t++){
    uint m=0;
    for(uint i=0; i<tasks.N; i++) {
      Task *c = tasks(i);
      if(!c->isActive(t)) continue;
      uint d=c->map.dim_phi(configurations.refRange(t,t+k_order), t);
      for(uint i=0;i<d;i++) CHECK(ttMatrix(t)(m+i)==c->type,"");
      if(d){
        if(c->type==sumOfSqrTT) taskC(i) += sumOfSqr(phiMatrix(t).sub(m,m+d-1));
        if(c->type==ineqTT){
          for(uint j=0;j<d;j++){
            double g=phiMatrix(t)(m+j);
            if(g>0.) taskG(i) += g;
          }
        }
        if(c->type==eqTT){
          for(uint j=0;j<d;j++) taskG(i) += fabs(phiMatrix(t)(m+j));
        }
        m += d;
      }
    }
    CHECK_EQ(m , phiMatrix(t).N, "");
  }

  Graph report;
  double totalC=0., totalG=0.;
  for(uint i=0; i<tasks.N; i++) {
    Task *c = tasks(i);
    Graph *g = &report.appendSubgraph({c->name}, {})->value;
    g->append<double>({"order"}, {}, c->map.order);
    g->append<mlr::String>({"type"}, {}, STRING(TermTypeString[c->type]));
    g->append<double>({"sqrCosts"}, {}, taskC(i));
    g->append<double>({"constraints"}, {}, taskG(i));
    totalC += taskC(i);
    totalG += taskG(i);
  }
  report.append<double>({"total","sqrCosts"}, {}, totalC);
  report.append<double>({"total","constraints"}, {}, totalG);

  return report;
}

arr MotionProblem::getInitialization(){
  if(!configurations.N) setupConfigurations();
  CHECK_EQ(configurations.N, k_order+T, "configurations are not setup yet");
  arr x;
  for(uint t=0;t<T;t++) x.append(configurations(t+k_order)->getJointState());
  return x;
}

void MotionProblem::inverseKinematics(arr& y, arr& J, arr& H, TermTypeA& tt, const arr& x){
  CHECK(!T,"");
  y.clear();
  if(&J) J.clear();
  if(&H) H.clear();
  if(&tt) tt.clear();
  set_x(x);
  phi_t(y, J, tt, 0);
}

//===========================================================================

arr getH_rate_diag(ors::KinematicWorld& world) {
  //transition cost metric
  arr W_diag;
  if(mlr::checkParameter<arr>("Wdiag")) {
    W_diag = mlr::getParameter<arr>("Wdiag");
  } else {
    W_diag = world.naturalQmetric();
  }
  return mlr::getParameter<double>("Hrate", 1.)*W_diag;
}


void sineProfile(arr& q, const arr& q0, const arr& qT,uint T){
  q.resize(T+1,q0.N);
  for(uint t=0; t<=T; t++) q[t] = q0 + .5 * (1.-cos(MLR_PI*t/T)) * (qT-q0);
}

arr reverseTrajectory(const arr& q){
  uint T=q.d0-1;
  arr r(T+1, q.d1);
  for(uint t=0; t<=T; t++) r[T-t] = q[t];
  return r;
}

void getVel(arr& v, const arr& q, double tau){
  uint T=q.d0-1;
  v.resizeAs(q);
  for(uint t=1; t<T; t++)  v[t] = (q[t+1] - q[t-1])/(2.*tau);
  v[0] = (q[1] - q[0])/tau;
  v[T] = (q[T] - q[T-1])/tau;
}

void getAcc(arr& a, const arr& q, double tau){
  uint T=q.d0-1;
  a.resizeAs(q);
  for(uint t=1; t<T; t++)  a[t] = (q[t+1] + q[t-1] - 2.*q[t])/(tau*tau);
  a[0] = a[1]/2.;
  a[T] = a[T-1]/2.;
}

RUN_ON_INIT_BEGIN(motion)
mlr::Array<ors::KinematicWorld*>::memMove=true;
RUN_ON_INIT_END(motion)
