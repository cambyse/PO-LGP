#include "logging.h"
#include "control.h"
#include "biros_internal.h"
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

AccessController::AccessController(){
  s = new sAccessController;
  s->enableAccessLog = true;
  s->replay = false;
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
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::read, v->revision, p->step_count);
  s->eventsLock.writeLock();
  events.append(e);
  s->eventsLock.unlock();
}

void AccessController::logWriteAccess(const Variable *v, const Process *p) {
  if(!s->enableAccessLog || s->replay) return;
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::write, v->revision, p->step_count);
  s->eventsLock.writeLock();
  events.append(e);
  s->eventsLock.unlock();
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

//===========================================================================

LoggerVariableData* sAccessController::getVariableData(const Variable* v){
  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
  return v->s->loggerData;
}
