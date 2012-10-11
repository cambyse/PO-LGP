#include "biros.h"
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

sAccessController::sAccessController():enableAccessLog(false),replay(false),blockMode(0),eventsFile(NULL){
}

sAccessController::~sAccessController(){
  //if(eventsFile){ eventsFile->close();  delete eventsFile; }
  //delete s;
}

void sAccessController::breakpointSleep(){ //the caller goes to sleep
  ConditionVariable *c = new ConditionVariable;
  breakpointMutex.lock();
  breakpointQueue.append(c);
  breakpointMutex.unlock();
  c->waitForSignal();
}

void sAccessController::breakpointNext(){ //first in the queue is being woke up
  breakpointMutex.lock();
  ConditionVariable *c = breakpointQueue.popFirst();
  breakpointMutex.unlock();
  if(!c) return;
  c->broadcast();
  delete c;
}

void sAccessController::queryReadAccess(Variable *v, const Process *p){
  blockMode.lock();
  if(blockMode.value>=4){
    AccessEvent *e = new AccessEvent(v, p, AccessEvent::read, v->revision, p?p->step_count:0);
    blockMode.waitForValueSmallerThan(4, true);
    if(blockMode.value==3) blockMode.value=4; //only this read steps, next to wait
    blockedAccesses.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void sAccessController::queryWriteAccess(Variable *v, const Process *p){
  blockMode.lock();
  if(blockMode.value>=2){
    AccessEvent *e = new AccessEvent(v, p, AccessEvent::write, v->revision, p?p->step_count:0);
    blockedAccesses.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=4; //only this read steps, next to wait
    blockedAccesses.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}
  
void sAccessController::logReadAccess(const Variable *v, const Process *p) {
  if(!enableAccessLog || replay) return;
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::read, v->revision, p?p->step_count:0);
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sAccessController::logWriteAccess(const Variable *v, const Process *p) {
  if(!enableAccessLog || replay) return;
  AccessEvent *e = new AccessEvent(v, p, AccessEvent::write, v->revision, p?p->step_count:0);
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sAccessController::logReadDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  if(getVariableData(v)->controllerBlocksRead)
    breakpointSleep();
}

void sAccessController::logWriteDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(getVariableData(v)->controllerBlocksWrite)
    breakpointSleep();
}

//==============================================================================

void BirosInfo::blockAllAccesses(){
  acc->blockMode.setValue(4);
}

void BirosInfo::unblockAllAccesses(){
  acc->blockMode.setValue(0);
}

void BirosInfo::stepToNextAccess(){
  acc->blockMode.setValue(3, true);
}

void BirosInfo::stepToNextWriteAccess(){
  acc->blockMode.setValue(1, true);
}


void sAccessController::dumpEventList(){
  if(!eventsFile){
    eventsFile = new ofstream;
    MT::open(*eventsFile,"z.eventLog");
  }
  
  AccessEventL copy;
  eventsLock.writeLock();
  copy.takeOver(events);
  events.clear();
  eventsLock.unlock();
  
  uint i;
  AccessEvent *e;
  for_list(i,e,copy){
    (*eventsFile)
      <<(e->type==AccessEvent::read?'r':'w')
      <<' ' <<e->var->name <<'-' <<e->var->id
      <<' ' <<e->revision
      <<' ' <<(e->proc?e->proc->name:STRING("NULL"))
      <<' ' <<e->procStep <<endl;
  }
}

//===========================================================================

LoggerVariableData* sAccessController::getVariableData(const Variable* v){
  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
  return v->s->loggerData;
}
