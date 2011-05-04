#include "process.h"
#include "threads.h"


//===========================================================================
//
// Global Static information
//

static VariableL globalVariables;
static ProcessL  globalProcesses;


//===========================================================================
//
// Variable
//

struct sVariable{
  Variable *p;
  //ofstream os;
  Lock lock;
  ConditionVariable cond;

  sVariable(Variable *_p){ p = _p; }
};

Variable::Variable(const char *_name){
  s = new sVariable(this);
  name = _name;
  //s->os.open(STRING("var-"<<name<<".log"));
  globalVariables.memMove=true;
  globalVariables.append(this);
};

Variable::~Variable(){
  //s->os.close();
  delete s;
  globalVariables.removeValue(this);
};

void Variable::readAccess(Process *p){
  if(p) p->V.setAppend(this);
  //cout <<(p?p->name:"NULL") <<" reads  " <<name <<" state=";
  s->lock.readLock();
  //_write(cout);
  //cout <<endl;
}

void Variable::writeAccess(Process *p){
  if(p) p->V.setAppend(this);
  //cout <<(p?p->name:"NULL") <<" writes " <<name <<" state=";
  s->lock.writeLock();
  //_write(cout);  cout <<endl;
  //_write(s->os);  s->os <<endl;
}

void Variable::deAccess(Process *p){
  //cout <<(p?p->name:"NULL") <<" frees  " <<name <<" state=";
  //_write(cout);
  //cout <<endl;
  s->lock.unlock(); 
}

int  Variable::getCondition(){
  return s->cond.getState();
}

void Variable::setCondition(int i){
  s->cond.setState(i);
}

void Variable::waitForConditionSignal(){
  s->cond.waitForSignal();
}

void Variable::waitForConditionEq(int i){
  s->cond.waitForStateEq(i);
}

void Variable::waitForConditionNotEq(int i){
  s->cond.waitForStateNotEq(i);
}



//===========================================================================
//
// Process
//

struct sProcess:public StepThread{
  Process *p;
  sProcess(Process *_p,const char* name):StepThread(name){
    p=_p;
  };

  void open(){ p->open(); }
  void step(){ p->step(); }
  void close(){ p->close(); }
};

Process::Process(const char *_name){
  s = new sProcess(this,_name);
  name = _name;
  globalProcesses.memMove=true;
  globalProcesses.append(this);
}

Process::~Process(){
  delete s;
  globalProcesses.removeValue(this);
}

void Process::threadOpen(int priority){
  s->threadOpen(priority);
}

void Process::threadClose(){
  s->threadClose();
}

void Process::threadStep(bool wait){
  s->threadStep(wait);
}

void Process::threadStepOrSkip(uint maxSkips){
  s->threadStepOrSkip(maxSkips);
}

bool Process::threadIsIdle(){
  return s->threadIsReady();
}

void Process::threadWait(){
  s->threadWait();
}

void Process::threadLoop(){
  s->threadLoop();
}

void Process::threadLoopWithBeat(double sec){
  s->threadLoopWithBeat(sec);
}

void Process::threadLoopSyncWithDone(Process& p){
  s->threadLoopSyncWithDone(*p.s);
}

void Process::threadStop(){
  s->threadLoopStop();
}

//===========================================================================
//
// Group
//

void Group::set(const VariableL &_V,const ProcessL &_P){
  V = _V;
  P = _P;
}

void Group::loop(){
  Process *p; uint i;
  for_list(i,p,P) p->threadLoop();
}

void Group::stop(){
  Process *p; uint i;
  for_list(i,p,P) p->threadStop();
}

void Group::close(){
  Process *p; uint i;
  for_list(i,p,P) p->threadClose();
}


//===========================================================================
//
// Global Static information
//

void reportGlobalProcessGraph(){
  ofstream fil("proc.graph");
  uint i,j;
  Variable *v;
  Process *p;
  for_list(i,v,globalVariables){
    fil <<"Variable " <<v->name <<endl;
  }
  fil <<endl;
  for_list(i,p,globalProcesses){
    fil <<"Process " <<p->name <<" (";
    for_list(j,v,p->V){
      if(j) fil <<',';
      fil <<v->name;
    }
    fil <<")" <<endl;
  }
  fil.close();
}
