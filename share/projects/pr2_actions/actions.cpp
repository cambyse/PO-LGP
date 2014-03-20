#include "puppeteer_internal.h"

//===========================================================================

struct MoveEffTo_ActionSymbol:ActionSymbol{
  MoveEffTo_ActionSymbol(){
    SymbolL::memMove=true;
    PDtaskL::memMove=true;
    ID=symbols().N;
    symbols().append(this);
    name="MoveEffTo";
    nargs=2;
  }
  virtual void initYourself(GroundedAction& a, Puppeteer& P) const{
    PDtask *task = P.s->MP.addPDTask(STRING("MoveEffTo_" <<a.shapeArg1), .1, .8, posTMT, a.shapeArg1);
    a.tasks.append(task);
  }
  virtual void deinitYourself(GroundedAction& a, Puppeteer& P) const{
    for(PDtask *t:a.tasks) P.s->MP.tasks.removeValue(t);
    listDelete(a.tasks);
  }
} moveEffTo_;
ActionSymbol &moveEffTo = moveEffTo_;

//===========================================================================

struct CoreTasks_ActionSymbol:ActionSymbol{
  CoreTasks_ActionSymbol(){
    ID=symbols().N;
    symbols().append(this);
    name="CoreTasks";
    nargs=0;
  }
  virtual void initYourself(GroundedAction& a, Puppeteer& P) const{
    PDtask *qitself = P.s->MP.addPDTask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, P.s->MP.H_rate_diag);
    qitself->setGains(0.,100.);
    qitself->y_ref = P.s->MP.nullSpacePD.y_ref;
    qitself->v_ref.setZero();
    qitself->prec=100.;
    a.tasks.append(qitself);

    PDtask *limits = P.s->MP.addPDTask("limits", .02, .8, qLimitsTMT);
    limits->active=true;
    limits->v_ref.setZero();
    limits->v_ref.setZero();
    limits->prec=100.;
    a.tasks.append(limits);
  }
  virtual void deinitYourself(GroundedAction& a, Puppeteer& P) const{
    for(PDtask *t:a.tasks) P.s->MP.tasks.removeValue(t);
    listDelete(a.tasks);
  }
} coreTasks_;
ActionSymbol &coreTasks = coreTasks_;

//===========================================================================

struct AlignEffTo_ActionSymbol:ActionSymbol{
  AlignEffTo_ActionSymbol(){
    ID=symbols().N;
    symbols().append(this);
    name="AlignEffTo";
    nargs=2;
  }
  virtual void initYourself(GroundedAction& a, Puppeteer& P) const{
    PDtask *task = P.s->MP.addPDTask(STRING("AlignEffTo_" <<a.shapeArg1), .2, .8, vecTMT, a.shapeArg1, ors::Vector(a.poseArg1));
    task->y_ref = a.poseArg2;
    a.tasks.append(task);
  }
  virtual void deinitYourself(GroundedAction& a, Puppeteer& P) const{
    for(PDtask *t:a.tasks) P.s->MP.tasks.removeValue(t);
    listDelete(a.tasks);
  }
} alignEffTo_;
ActionSymbol &alignEffTo = alignEffTo_;

//===========================================================================

struct PushForce_ActionSymbol:ActionSymbol{
  PushForce_ActionSymbol(){
    ID=symbols().N;
    symbols().append(this);
    name="PushForce";
    nargs=2;
  }
  virtual void initYourself(GroundedAction& a, Puppeteer& P) const{
    PDtask *task = P.s->MP.addPDTask(STRING("PushForce_" <<a.shapeArg1), .2, .8, vecTMT, a.shapeArg1, ors::Vector(a.poseArg1));
    task->y_ref = a.poseArg2;
    a.tasks.append(task);
  }
  virtual void deinitYourself(GroundedAction& a, Puppeteer& P) const{
    for(PDtask *t:a.tasks) P.s->MP.tasks.removeValue(t);
    listDelete(a.tasks);
  }
} pushForce_;
ActionSymbol &pushForce = pushForce_;

