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
#include <sys/syscall.h>
#include <map>
#include <signal.h>

#include "engine.h"
#include "engine_internal.h"

Singleton<Engine> SingleEngine;

System& NoSystem = *((System*)NULL);


//===========================================================================
//
// Variable
//

//struct sVariable {
//  virtual ~sVariable(){}
//  mlr::Array<struct FieldRegistration*> fields; //? make static? not recreating for each variable?
//  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

//  virtual void serializeToString(mlr::String &string) const;
//  virtual void deSerializeFromString(const mlr::String &string);

//  sVariable():loggerData(NULL){}
//};

//Variable_SharedMemory::Variable_SharedMemory(const char *_name):Variable(_name), s(NULL), revision(0) {
//  s = new sVariable();
//  listeners.memMove=true;
//  //MT logValues = false;
//  //MT dbDrivenReplay = false;
//  //MT pthread_mutex_init(&replay_mutex, NULL);
////  if(&(biros()) != NULL) { //-> birosInfo itself will not be registered!
////    biros().writeAccess(NULL);
////    biros().variables.memMove = true;
////    biros().variables.append(this);
////    biros().deAccess(NULL);
////  }
//}

//Variable_SharedMemory::~Variable_SharedMemory() {
////  if(this != global_birosInfo) { //-> birosInfo itself will not be de-registered!
////    biros().writeAccess(NULL);
////    biros().variables.removeValue(this);
////    biros().deAccess(NULL);
////  }
//  //for (uint i=0; i<s->fields.N; i++) delete s->fields(i);

//  //MT pthread_mutex_destroy(&replay_mutex);

//  delete s;
//}

//int Variable_SharedMemory::readAccess(Module *m) {
//  Module *p = m?(Module*) m:NULL;
//  engine().acc->queryReadAccess(this, p);
//  rwlock.readLock();
//  engine().acc->logReadAccess(this, p);
//  return revision.getValue();
//}

//int Variable_SharedMemory::writeAccess(Module *m) {
//  Module *p = m?(Module*) m:NULL;
//  engine().acc->queryWriteAccess(this, p);
//  rwlock.writeLock();
//  int r = revision.incrementValue();
//  revision_time = mlr::clockTime();
//  engine().acc->logWriteAccess(this, p);
//  for(Module *l: listeners) if(l!=m) engine().step(*l, true);
//  return r;
//}

//int Variable_SharedMemory::deAccess(Module *m) {
//  Module *p = m?(Module*) m:NULL;
//  if(rwlock.state == -1) { //log a revision after write access
//    //MT logService.logRevision(this);
//    //MT logService.setValueIfDbDriven(this); //this should be done within queryREADAccess, no?!
//    engine().acc->logWriteDeAccess(this,p);
//  } else {
//    engine().acc->logReadDeAccess(this,p);
//  }
//  int rev=revision.getValue();
//  rwlock.unlock();
//  return rev;
//}

//double Variable_SharedMemory::revisionTime(){
//  return revision_time;
//}

//int Variable_SharedMemory::revisionNumber(){
//  return revision.getValue();
//}

//int Variable_SharedMemory::waitForNextRevision(){
//  revision.lock();
//  revision.waitForSignal(true);
//  int rev = revision.value;
//  revision.unlock();
//  return rev;
//}

//int Variable_SharedMemory::waitForRevisionGreaterThan(int rev) {
//  revision.lock();
//  revision.waitForValueGreaterThan(rev, true);
//  rev = revision.value;
//  revision.unlock();
//  return rev;
//}

//FieldRegistration& Variable_SharedMemory::get_field(uint i) const{
//  return *s->fields(i);
//}

//void sVariable::serializeToString(mlr::String &string) const {
//#if 0
//  string.clear();
//  mlr::String field_string;
//  field_string.clear();

//  // go through fields
//  for (uint i=0; i < fields.N; i++) {

//    fields(i)->writeValue(field_string);

//    // replace every occurence of "\" by "\\"
//    for (uint j=0; j < field_string.N; j++) {
//      char c = field_string(j);
//      if('\\' == c) string << '\\';
//      string << c;
//    }

//    // add seperator after field
//    string << "\\,";
//  }
//#endif
//  NIY
//}

//void sVariable::deSerializeFromString(const mlr::String &string) {
//#if 0
//  mlr::String string_copy(string), field_string;
//  field_string.clear();
//  uint j = 0;
//  for (uint i=0; i< fields.N; i++) {
//    // get field strings from string (seperated by "\\,")
//    bool escaped = false; // true if previous char was '\\'
//    while (j < string_copy.N) {
//      char c = string_copy(j++);
//      if('\\' == c) {
//        escaped = true;
//      } else {
//        if(escaped) {
//          if(',' == c) {
//            break;
//          }
//        }
//        escaped = false;
//        field_string << c;
//      }
//    }
//    fields(i)->readValue(field_string);
//  }
//#endif
//  NIY
//}


//===========================================================================
//
// SystemDescription
//

Module* System::addModule(const char *dclName, const char *name, Module::StepMode mode, double beat){
  //find the dcl in the registry
  Node *modReg = registry().getNode("Decl_Module", dclName);
  if(!modReg){
    MLR_MSG("could not find Decl_Module " <<dclName);
    return NULL;
  }
  Module *m = (Module*)modReg->get<Type>().newInstance();
  currentlyCreating = NULL;
  for(Access *a: m->accesses) a->module = m;
  this->append(m);

  m->mode = mode;
  m->beat = beat;
  return m;
}

//Module* System::addModule(const char *dclName, const char *name, const uintA& accIdxs, Module::StepMode mode, double beat){
//  Module *m = addModule(dclName, name, mode, beat);
//  if(accIdxs.N != m->accesses.N) HALT("given and needed #acc mismatch");
//  for_list(Access, a, m->accesses) a->var = vars(accIdxs(a_COUNT));
//  return m;
//}

Module* System::addModule(const char *dclName, const char *name, const StringA& accRenamings, Module::StepMode mode, double beat){
  Module *m = addModule(dclName, name, mode, beat);
  if(accRenamings.N != m->accesses.N) HALT("given and needed #acc mismatch");
  for_list(Access, a, m->accesses) a->name = accRenamings(a_COUNT);
  return m;
}

RevisionedAccessGatedClass* System::connect(Access& acc, const char *variable_name){
  RevisionedAccessGatedClass *v = listFindByName(vars, variable_name);
  if(v){ //variable exists -> link it
    acc.linkToVariable(v);
  }else{ //variable does not exist yet
    acc.createVariable(variable_name);
    vars.append(acc.var);
  }
  return acc.var;
}

void System::connect(){
  //first collect all accesses; the union of System accesses and all module accesses
  AccessL accs;

  for(Module *m: *this){ for(Access *a: m->accesses) accs.append(a); }
  for(Access *a: accesses) accs.append(a);

  for(Access *a: accs){
    Module *m=a->module;
    RevisionedAccessGatedClass *v = NULL;
    if(!a->var) v = connect(*a, a->name); //access is not connected yet
    else v = a->var;

    if(m &&
       ( m->mode==Module::listenAll ||
         (m->mode==Module::listenFirst && a==m->accesses(0)) ) ){
      v->listeners.setAppend(m);
    }
  }
}

void System::run(bool waitForOpened){
  //connect();

  signal(SIGINT, signalhandler);

  //open modules
  for(Module *m: *this) m->threadOpen();
  if(waitForOpened) for(Module *m: *this) m->waitForOpened();

  //loop modules
  for(Module *m: *this){
    //start looping if in loop mode:
    switch(m->mode){
      case Module::loopWithBeat:  m->threadLoopWithBeat(m->beat);  break;
      case Module::loopFull:      m->threadLoop();  break;
      default:  break;
    }
  }
}

Graph System::graph() const{
  Graph g;
  g.newNode<bool>({"SystemModule", name}, {}, NULL, false);
  g.checkConsistency();
  std::map<RevisionedAccessGatedClass*, Node*> vit;
  for(RevisionedAccessGatedClass *v: vars) vit[v] = g.newNode({"Variable", v->name}, {}, v, false);
  g.checkConsistency();
  for(Module *m: *this){
    Node *mit = g.newNode({"Module", m->name}, {}, &m, false);
    g.checkConsistency();
    for(Access *a: m->accesses){
      Node *ait = g.newNode({"Access", a->name}, {}, &a, false);
      ait->parents.append(mit);
      mit->parentOf.append(ait);
      if(a->var){
        ait->parents.append(vit[a->var]);
        vit[a->var]->parentOf.append(ait);
      }
      g.checkConsistency();
    }
  }
  g.checkConsistency();
  return g;
}

void System::write(ostream& os) const{
  os <<graph() <<endl;
}


//===========================================================================
//
// Engine
//


Engine& engine(){  return SingleEngine(); }

Engine::Engine(): mode(none), system(NULL), shutdown(0) {
  acc = new EventController;
};

Engine::~Engine(){
  //acc -> dumpEventList();
  delete acc;
}

void Engine::open(System& S, bool waitForOpened){
  system = &S;

  //S.connect();

  signal(SIGINT, signalhandler);

  if(mode==none) mode=threaded;

  //open modules
  if(mode==threaded){
    for(Module *m: S) m->threadOpen();
    if(waitForOpened) for(Module *m: S) m->waitForOpened();
    for(Module *m: S){
      //start looping if in loop mode:
      switch(m->mode){
      case Module::loopWithBeat:  m->threadLoopWithBeat(m->beat);  break;
      case Module::loopFull:      m->threadLoop();  break;
      default:  break;
      }
    }
  }

  if(mode==serial){
    for(Module *m: S) m->open();
  }
}

void Engine::step(System &S){
  if(&S) system=&S;
  for(Module *m: S) step(*m);
}

void Engine::step(Module &m, bool threadedOnly){
  if(threadedOnly && mode!=threaded) return;
  if(mode==none) MLR_MSG("ommiting stepping: no step mode");
  if(mode==threaded) m.threadStep();
  if(mode==serial)   m.step();
}

void Engine::test(System& S){
  if(&S) system=&S;
  CHECK(mode!=threaded,"");
  mode=serial;
  open(S);
  for(Module *m: S) m->test();
  close(S);
}

void Engine::close(System& S){
  if(&S) system=&S;
  for(Module *m: *system){
    if(mode==threaded) m->threadClose();
    if(mode==serial)   m->close();
  }
}

void Engine::cancel(System& S){
  if(&S) system=&S;
  for(Module *m: *system){
    if(mode==threaded) m->threadCancel();
  }
}

void Engine::enableAccessLog(){
  acc->enableEventLog = true;
}

void Engine::dumpAccessLog(){
  acc->dumpEventList();
}

void Engine::blockAllAccesses(){
  acc->blockMode.setValue(2);
}

void Engine::unblockAllAccesses(){
  acc->blockMode.setValue(0);
}

void Engine::stepToNextAccess(){
  acc->blockMode.setValue(1, true);
}

void Engine::stepToNextWriteAccess(){
  acc->blockMode.setValue(1, true);
}


//===========================================================================
//
// LoggerVariableData
//

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


//===========================================================================
//
// EventController
//

EventController::EventController():enableEventLog(false),enableReplay(false),blockMode(0),eventsFile(NULL){
  events.memMove=true;
  blockedEvents.memMove=true;
}

EventController::~EventController(){
  //if(eventsFile){ eventsFile->close();  delete eventsFile; }
  //delete s;
}

void EventController::breakpointSleep(){ //the caller goes to sleep
  ConditionVariable *c = new ConditionVariable;
  breakpointMutex.lock();
  breakpointQueue.append(c);
  breakpointMutex.unlock();
  c->waitForSignal();
}

void EventController::breakpointNext(){ //first in the queue is being woke up
  breakpointMutex.lock();
  ConditionVariable *c = breakpointQueue.popFirst();
  breakpointMutex.unlock();
  if(!c) return;
  c->broadcast();
  delete c;
}

void EventController::queryReadAccess(RevisionedAccessGatedClass *v, const Module *p){
  blockMode.lock();
  if(blockMode.value>=1){
    EventRecord *e = new EventRecord(v, p, EventRecord::read, v->revision.getValue(), p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2; //1: only ONE reader
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void EventController::queryWriteAccess(RevisionedAccessGatedClass *v, const Module *p){
  blockMode.lock();
  if(blockMode.value>=1){
    EventRecord *e = new EventRecord(v, p, EventRecord::write, v->revision.getValue(), p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2;
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void EventController::logReadAccess(const RevisionedAccessGatedClass *v, const Module *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(v, p, EventRecord::read, v->revision.getValue(), p?p->step_count:0, mlr::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logWriteAccess(const RevisionedAccessGatedClass *v, const Module *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(v, p, EventRecord::write, v->revision.getValue(), p?p->step_count:0, mlr::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logReadDeAccess(const RevisionedAccessGatedClass *v, const Module *p) {
  //do something if in replay mode
  if(getVariableData(v)->controllerBlocksRead)
    breakpointSleep();
}

void EventController::logWriteDeAccess(const RevisionedAccessGatedClass *v, const Module *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(getVariableData(v)->controllerBlocksWrite)
    breakpointSleep();
}

void EventController::logStepBegin(const Module *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(NULL, p, EventRecord::stepBegin, 0, p->step_count, mlr::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logStepEnd(const Module *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(NULL, p, EventRecord::stepEnd, 0, p->step_count, mlr::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::writeEventList(ostream& os, bool blocked, uint max, bool clear){
  EventRecordL copy;
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
  for_list(EventRecord, e, copy){
    if(!e_COUNT && max && copy.N>max){ e_COUNT=copy.N-max; e=copy(e_COUNT); }
    switch(e->type){
    case EventRecord::read: os <<'r';  break;
    case EventRecord::write: os <<'w';  break;
    case EventRecord::stepBegin: os <<'b';  break;
    case EventRecord::stepEnd: os <<'e';  break;
    }

    os
      <<' ' <<(e->module?e->module->name:STRING("NULL"))
      <<' ' <<(e->variable?e->variable->name:STRING("NULL"))
      <<' ' <<e->revision
      <<' ' <<e->procStep
      <<' ' <<e->time
      <<endl;
  }
}

void EventController::dumpEventList(){
  if(!events.N) return;

  if(!eventsFile){
    eventsFile = new ofstream;
    mlr::open(*eventsFile,"z.eventLog");
  }

  writeEventList(*eventsFile, false, 0, true);
}

LoggerVariableData* EventController::getVariableData(const RevisionedAccessGatedClass* v){
//  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
//  return v->s->loggerData;
  NIY;
  return NULL;
}


