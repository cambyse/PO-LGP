#include "actionMachine_internal.h"

//===========================================================================

struct MoveEffTo:GroundedAction{
  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  MoveEffTo(const char* s, const arr& pos){
    shapeArg1 = MT::String(s);
    poseArg1 = pos;
    SymbolL::memMove=true;
    PDtaskL::memMove=true;
    ID=symbols().N;
    symbols().append(this);
    name="MoveEffTo";
    nargs=2;
  }
  virtual void initYourself(ActionMachine& P) {
    PDtask *task;
    task = P.s->MP.addPDTask(STRING("MoveEffTo_" <<shapeArg1), 1., .8, posTMT, shapeArg1);
//    task->setGains(200.,0.);
    task->y_ref = poseArg1;
    tasks.append(task);
  }
  virtual void deinitYourself(ActionMachine& P) {
    for(PDtask *t:tasks) P.s->MP.tasks.removeValue(t);
    listDelete(tasks);
  }
  virtual bool finishedSuccess(ActionMachine& M) {
    PDtask *task=tasks(0);
    return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-2);
  }

};

//===========================================================================

struct AlignEffTo:GroundedAction{

  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  AlignEffTo(){
    ID=symbols().N;
    symbols().append(this);
    name="AlignEffTo";
    nargs=2;
  }
  virtual void initYourself(ActionMachine& P) {
    PDtask *task;
    task = P.s->MP.addPDTask(STRING("AlignEffTo_" <<shapeArg1), 2., .8, vecTMT, shapeArg1, ors::Vector(poseArg1));
//    task->setGains(100.,0.);
    task->y_ref = poseArg2;
    tasks.append(task);
  }
  virtual void deinitYourself(ActionMachine& P) {
    for(PDtask *t:tasks) P.s->MP.tasks.removeValue(t);
    listDelete(tasks);
  }
  virtual bool finishedSuccess(ActionMachine& M) {
    PDtask *task=tasks(0);
    return (task->y.N==task->y_ref.N && maxDiff(task->y, task->y_ref)<1e-1);
  }
};

//===========================================================================

struct CoreTasks:GroundedAction{

  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  CoreTasks(){
    ID=symbols().N;
    symbols().append(this);
    name="CoreTasks";
    nargs=0;
  }
  virtual void initYourself(ActionMachine& P) {
//    PDtask *qitself;
//    qitself = P.s->MP.addPDTask("DampMotion_qitself", .1, 1., qLinearTMT, NULL, NoVector, NULL, NoVector, P.s->MP.H_rate_diag);
//    qitself->setGains(0.,10.);
//    qitself->y_ref = P.s->MP.qitselfPD.y_ref;
//    qitself->v_ref.setZero();
//    qitself->prec=100.;
//    tasks.append(qitself);

    PDtask *limits;
    limits = P.s->MP.addPDTask("limits", .1, .8, qLimitsTMT);
//    limits->setGains(10.,0.);
    limits->v_ref.setZero();
    limits->v_ref.setZero();
    limits->prec=100.;
    tasks.append(limits);
  }
  virtual void deinitYourself(ActionMachine& P) {
    for(PDtask *t:tasks) P.s->MP.tasks.removeValue(t);
    listDelete(tasks);
  }
};

//===========================================================================

struct PushForce:GroundedAction{

  MT::String shapeArg1, shapeArg2;
  arr poseArg1, poseArg2;

  static Symbol symbol;
  virtual Symbol& getSymbol() {return symbol;}

  PushForce(){
    ID=symbols().N;
    symbols().append(this);
    name="PushForce";
    nargs=2;
  }
  virtual void initYourself(ActionMachine& P) {
    PDtask *task;
    task = P.s->MP.addPDTask(STRING("PushForce_" <<shapeArg1), .2, .8, vecTMT, shapeArg1, ors::Vector(poseArg1));
    task->y_ref = poseArg2;
    tasks.append(task);
  }
  virtual void deinitYourself(ActionMachine& P) {
    for(PDtask *t:tasks) P.s->MP.tasks.removeValue(t);
    listDelete(tasks);
  }
};
