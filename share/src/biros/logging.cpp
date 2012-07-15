#include "logging.h"
#include "control.h"
#include "biros_internal.h"
#include "../hardware/hardware.h"

bool enableLogging=false;
AccessController accessController;

//some data we want to associate with each variable
struct LoggerVariableData {
  //-- the contoller (user interface) may block accesses (AFTER they're done to allow inspection)
  bool controllerBlocksRead, controllerBlocksWrite;
  
  //-- or replay may block access to ensure right revision
  /* here every process sleeps when they want to access a variable not having the correct revision yet */
  ConditionVariable readCondVar;
  /* here everyone sleeps who wants to have write access */
  ConditionVariable writeCondVar;
  
  LoggerVariableData(): controllerBlocksRead(false), controllerBlocksWrite(false){}
};

struct sAccessController{
  bool enableAccessLog;
  bool enableDataLog;
  bool replay;
  Lock eventsLock;
  
  LoggerVariableData* getVariableData(const Variable *v);
  
  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(){ //the caller goes to sleep
    ConditionVariable *c = new ConditionVariable;
    breakpointMutex.lock();
    breakpointQueue.append(c);
    breakpointMutex.unlock();
    c->waitForSignal();
  }
  void breakpointNext(){ //first in the queue is being woke up
    breakpointMutex.lock();
    ConditionVariable *c = breakpointQueue.popFirst();
    breakpointMutex.unlock();
    if(!c) return;
    c->broadcast();
    delete c;
  }
};

AccessController::AccessController()
:dumpFile(NULL){
  s = new sAccessController;
  s->enableAccessLog = false;
  s->replay = false;
}

AccessController::~AccessController(){
  if(dumpFile){ dumpFile->close();  delete dumpFile; }
  delete s;
}
  
void AccessController::queryReadAccess(Variable *v, const Process *p){
  //VariableBlock *block=s->getVariableBlock(v);
  //block->writeCondVar.waitForSignaltateNotEq(-2);
}

void AccessController::queryWriteAccess(Variable *v, const Process *p){
  //VariableBlock *block=s->getVariableBlock(v);
  //block->writeCondVar.waitForStateNotEq(-2);
}
  
void AccessController::logReadAccess(const Variable *v, const Process *p) {
  if(!s->enableAccessLog || s->replay) return;
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::read, v->revision, p?p->step_count:0);
  s->eventsLock.writeLock();
  events.append(e);
  s->eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void AccessController::logWriteAccess(const Variable *v, const Process *p) {
  if(!s->enableAccessLog || s->replay) return;
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::write, v->revision, p?p->step_count:0);
  s->eventsLock.writeLock();
  events.append(e);
  s->eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void AccessController::logReadDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  if(s->getVariableData(v)->controllerBlocksRead)
    s->breakpointSleep();
}

void AccessController::logWriteDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(s->getVariableData(v)->controllerBlocksWrite)
    s->breakpointSleep();
}

void AccessController::dumpEventList(){
  if(!dumpFile){
    dumpFile = new ofstream;
    MT::open(*dumpFile,"z.eventLog");
  }
  
  AccessEventL copy;
  s->eventsLock.writeLock();
  copy.takeOver(events);
  events.clear();
  s->eventsLock.unlock();
  
  uint i;
  AccessEvent *e;
  for_list(i,e,copy){
    (*dumpFile)
      <<(e->type==AccessEvent::read?'r':'w')
      <<' ' <<e->var->name <<'-' <<e->var->id
      <<' ' <<e->revision
      <<' ' <<(e->proc?e->proc->name:"NULL")
      <<' ' <<e->procStep <<endl;
  }
}

//===========================================================================

LoggerVariableData* sAccessController::getVariableData(const Variable* v){
  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
  return v->s->loggerData;
}
