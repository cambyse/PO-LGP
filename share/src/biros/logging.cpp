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

  ConditionVariable blockMode;

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
:eventsFile(NULL){
  s = new sAccessController;
  s->enableAccessLog = false;
  s->replay = false;
  s->blockMode.setState(0);
}

AccessController::~AccessController(){
  if(eventsFile){ eventsFile->close();  delete eventsFile; }
  delete s;
}

//0 = reads run, writes run
//1 = reads run, next write runs
//2 = reads run, writes blocked
//3 = next read runs, writes blocked
//4 = reads blocked,  writes blocked

void AccessController::blockAllAccesses(){
  s->blockMode.setState(4);
}

void AccessController::unblockAllAccesses(){
  s->blockMode.setState(0);
}

void AccessController::stepToNextAccess(){
  s->blockMode.setState(3, true);
}

void AccessController::stepToNextWriteAccess(){
  s->blockMode.setState(1, true);
}

void AccessController::queryReadAccess(Variable *v, const Process *p){
  s->blockMode.lock();
  if(s->blockMode.state>=4){
    AccessEvent *e = new AccessEvent(v, p, AccessEvent::read, v->revision, p?p->step_count:0);
    s->blockMode.waitForStateSmallerThan(4, true);
    if(s->blockMode.state==3) s->blockMode.state=4; //only this read steps, next to wait
    blockedAccesses.removeValue(e);
    delete e;
  }
  s->blockMode.unlock();
}

void AccessController::queryWriteAccess(Variable *v, const Process *p){
  s->blockMode.lock();
  if(s->blockMode.state>=2){
    AccessEvent *e = new AccessEvent(v, p, AccessEvent::write, v->revision, p?p->step_count:0);
    blockedAccesses.append(e);
    s->blockMode.waitForStateSmallerThan(2, true);
    if(s->blockMode.state==1) s->blockMode.state=4; //only this read steps, next to wait
    blockedAccesses.removeValue(e);
    delete e;
  }
  s->blockMode.unlock();
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
  if(!eventsFile){
    eventsFile = new ofstream;
    MT::open(*eventsFile,"z.eventLog");
  }
  
  AccessEventL copy;
  s->eventsLock.writeLock();
  copy.takeOver(events);
  events.clear();
  s->eventsLock.unlock();
  
  uint i;
  AccessEvent *e;
  for_list(i,e,copy){
    (*eventsFile)
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
