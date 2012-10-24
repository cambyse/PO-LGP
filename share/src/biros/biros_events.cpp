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

sBirosEventController::sBirosEventController():enableEventLog(false),enableReplay(false),blockMode(0),eventsFile(NULL){
  events.memMove=true;
  blockedEvents.memMove=true;
}

sBirosEventController::~sBirosEventController(){
  //if(eventsFile){ eventsFile->close();  delete eventsFile; }
  //delete s;
}

void sBirosEventController::breakpointSleep(){ //the caller goes to sleep
  ConditionVariable *c = new ConditionVariable;
  breakpointMutex.lock();
  breakpointQueue.append(c);
  breakpointMutex.unlock();
  c->waitForSignal();
}

void sBirosEventController::breakpointNext(){ //first in the queue is being woke up
  breakpointMutex.lock();
  ConditionVariable *c = breakpointQueue.popFirst();
  breakpointMutex.unlock();
  if(!c) return;
  c->broadcast();
  delete c;
}

void sBirosEventController::queryReadAccess(Variable *v, const Process *p){
  blockMode.lock();
  if(blockMode.value>=1){
    BirosEvent *e = new BirosEvent(v, p, BirosEvent::read, v->revision, p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2; //1: only ONE reader
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void sBirosEventController::queryWriteAccess(Variable *v, const Process *p){
  blockMode.lock();
  if(blockMode.value>=1){
    BirosEvent *e = new BirosEvent(v, p, BirosEvent::write, v->revision, p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2;
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}
  
void sBirosEventController::logReadAccess(const Variable *v, const Process *p) {
  if(!enableEventLog || enableReplay) return;
  BirosEvent *e = new BirosEvent(v, p, BirosEvent::read, v->revision, p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sBirosEventController::logWriteAccess(const Variable *v, const Process *p) {
  if(!enableEventLog || enableReplay) return;
  BirosEvent *e = new BirosEvent(v, p, BirosEvent::write, v->revision, p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sBirosEventController::logReadDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  if(getVariableData(v)->controllerBlocksRead)
    breakpointSleep();
}

void sBirosEventController::logWriteDeAccess(const Variable *v, const Process *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(getVariableData(v)->controllerBlocksWrite)
    breakpointSleep();
}

void sBirosEventController::logStepBegin(const Process *p) {
  if(!enableEventLog || enableReplay) return;
  BirosEvent *e = new BirosEvent(NULL, p, BirosEvent::stepBegin, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sBirosEventController::logStepEnd(const Process *p) {
  if(!enableEventLog || enableReplay) return;
  BirosEvent *e = new BirosEvent(NULL, p, BirosEvent::stepEnd, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void sBirosEventController::writeEventList(ostream& os, bool blocked, uint max, bool clear){
  BirosEventL copy;
  eventsLock.writeLock();
  if(!blocked){
    if(clear){
      copy.takeOver(events);
      events.clear();
    }else{
      copy = events;
    }
  }else{
    if(clear){
      copy.takeOver(blockedEvents);
      blockedEvents.clear();
    }else{
      copy = blockedEvents;
    }
  }
  eventsLock.unlock();
  uint i;
  BirosEvent *e;
  for_list(i,e,copy){
    if(!i && max && copy.N>max){ i=copy.N-max; e=copy(i); }
    switch(e->type){
    case BirosEvent::read: os <<'r';  break;
    case BirosEvent::write: os <<'w';  break;
    case BirosEvent::stepBegin: os <<'b';  break;
    case BirosEvent::stepEnd: os <<'e';  break;
    }

    os
      <<' ' <<(e->proc?e->proc->name:STRING("NULL"))
      <<' ' <<(e->var?e->var->name:STRING("NULL")) <<'_' <<(e->var?e->var->id:0)
      <<' ' <<e->revision
      <<' ' <<e->procStep
      <<' ' <<e->time
      <<endl;
  }
}

void sBirosEventController::dumpEventList(){
  if(!events.N) return;

  if(!eventsFile){
    eventsFile = new ofstream;
    MT::open(*eventsFile,"z.eventLog");
  }

  writeEventList(*eventsFile, false, 0, true);
}

//==============================================================================

void BirosInfo::enableAccessLog(){
  acc->enableEventLog = true;
}

void BirosInfo::dumpAccessLog(){
  acc->dumpEventList();
}

void BirosInfo::blockAllAccesses(){
  acc->blockMode.setValue(2);
}

void BirosInfo::unblockAllAccesses(){
  acc->blockMode.setValue(0);
}

void BirosInfo::stepToNextAccess(){
  acc->blockMode.setValue(1, true);
}

void BirosInfo::stepToNextWriteAccess(){
  acc->blockMode.setValue(1, true);
}


//===========================================================================

LoggerVariableData* sBirosEventController::getVariableData(const Variable* v){
  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
  return v->s->loggerData;
}
