/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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
// ModuleThread
//

void ModuleThread::step(){
  engine().acc->logStepBegin(this);
  m->step();
  step_count++;
  engine().acc->logStepEnd(this);
}

//===========================================================================
//
// Variable
//

struct sVariable {
  virtual ~sVariable(){}
  MT::Array<struct FieldRegistration*> fields; //? make static? not recreating for each variable?
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);

  sVariable():loggerData(NULL){}
};

Variable::Variable(const char *_name):VariableAccess(_name), s(NULL), revision(0) {
  s = new sVariable();
  listeners.memMove=true;
  //MT logValues = false;
  //MT dbDrivenReplay = false;
  //MT pthread_mutex_init(&replay_mutex, NULL);
//  if(&(biros()) != NULL) { //-> birosInfo itself will not be registered!
//    biros().writeAccess(NULL);
//    biros().variables.memMove = true;
//    biros().variables.append(this);
//    biros().deAccess(NULL);
//  }
}

Variable::~Variable() {
//  if(this != global_birosInfo) { //-> birosInfo itself will not be de-registered!
//    biros().writeAccess(NULL);
//    biros().variables.removeValue(this);
//    biros().deAccess(NULL);
//  }
  //for (uint i=0; i<s->fields.N; i++) delete s->fields(i);

  //MT pthread_mutex_destroy(&replay_mutex);

  delete s;
}

int Variable::readAccess(Module *m) {
  ModuleThread *p = m?(ModuleThread*) m->thread:NULL;
  engine().acc->queryReadAccess(this, p);
  rwlock.readLock();
  engine().acc->logReadAccess(this, p);
  return revision.getValue();
}

int Variable::writeAccess(Module *m) {
  ModuleThread *p = m?(ModuleThread*) m->thread:NULL;
  engine().acc->queryWriteAccess(this, p);
  rwlock.writeLock();
  int r = revision.incrementValue();
  revision_time = MT::clockTime();
  engine().acc->logWriteAccess(this, p);
  for_list_(Module, l, listeners) if(l!=m) engine().step(*l, true);
  return r;
}

int Variable::deAccess(Module *m) {
  ModuleThread *p = m?(ModuleThread*) m->thread:NULL;
  if(rwlock.state == -1) { //log a revision after write access
    //MT logService.logRevision(this);
    //MT logService.setValueIfDbDriven(this); //this should be done within queryREADAccess, no?!
    engine().acc->logWriteDeAccess(this,p);
  } else {
    engine().acc->logReadDeAccess(this,p);
  }
  int rev=revision.getValue();
  rwlock.unlock();
  return rev;
}

double Variable::revisionTime(){
  return revision_time;
}

int Variable::revisionNumber(){
  return revision.getValue();
}

int Variable::waitForNextWriteAccess(){
  revision.lock();
  revision.waitForSignal(true);
  int rev = revision.value;
  revision.unlock();
  return rev;
}

int Variable::waitForRevisionGreaterThan(int rev) {
  revision.lock();
  revision.waitForValueGreaterThan(rev, true);
  rev = revision.value;
  revision.unlock();
  return rev;
}

FieldRegistration& Variable::get_field(uint i) const{
  return *s->fields(i);
}

void sVariable::serializeToString(MT::String &string) const {
#if 0
  string.clear();
  MT::String field_string;
  field_string.clear();

  // go through fields
  for (uint i=0; i < fields.N; i++) {

    fields(i)->writeValue(field_string);

    // replace every occurence of "\" by "\\"
    for (uint j=0; j < field_string.N; j++) {
      char c = field_string(j);
      if('\\' == c) string << '\\';
      string << c;
    }

    // add seperator after field
    string << "\\,";
  }
#endif
  NIY
}

void sVariable::deSerializeFromString(const MT::String &string) {
#if 0
  MT::String string_copy(string), field_string;
  field_string.clear();
  uint j = 0;
  for (uint i=0; i< fields.N; i++) {
    // get field strings from string (seperated by "\\,")
    bool escaped = false; // true if previous char was '\\'
    while (j < string_copy.N) {
      char c = string_copy(j++);
      if('\\' == c) {
        escaped = true;
      } else {
        if(escaped) {
          if(',' == c) {
            break;
          }
        }
        escaped = false;
        field_string << c;
      }
    }
    fields(i)->readValue(field_string);
  }
#endif
  NIY
}


//===========================================================================
//
// SystemDescription
//

Module* System::addModule(const char *dclName, const char *name, ModuleThread::StepMode mode, double beat){
  //find the dcl in the registry
  Item *modReg = registry().getItem("Decl_Module", dclName);
  if(!modReg){
    MT_MSG("could not find Decl_Module " <<dclName);
    return NULL;
  }
  Module *m = (Module*)modReg->value<Type>()->newInstance();
  currentlyCreating = NULL;
  for_list_(Access, a, m->accesses) a->module = m;
  mts.append(m);

  m->thread = new ModuleThread(m, name?name:dclName);
  m->thread->mode = mode;
  m->thread->beat = beat;
  return m;
}

Module* System::addModule(const char *dclName, const char *name, const uintA& accIdxs, ModuleThread::StepMode mode, double beat){
  Module *m = addModule(dclName, name, mode, beat);
  if(accIdxs.N != m->accesses.N) HALT("given and needed #acc mismatch");
  for_list_(Access, a, m->accesses) a->var = vars(accIdxs(a_COUNT));
  return m;
}

Module* System::addModule(const char *dclName, const char *name, const StringA& accRenamings, ModuleThread::StepMode mode, double beat){
  Module *m = addModule(dclName, name, mode, beat);
  if(accRenamings.N != m->accesses.N) HALT("given and needed #acc mismatch");
  for_list_(Access, a, m->accesses) a->name = accRenamings(a_COUNT);
  return m;
}

Variable* System::connect(Access& acc, const char *variable_name){
  Variable *v = listFindByName(vars, variable_name);
  if(v){ //variable exists -> check type
    if(*v->type != *acc.type) HALT("trying to connect an access '" <<acc.name <<*acc.type <<" with a variable " <<v->name <<*v->type);
    //good: just connect
    acc.var = v;
  }else{ //variable does not exist yet
    acc.var = v = addVariable(acc);
    v->name = variable_name; //give it the name that it is supposed to have!
  }
  return v;
}

void System::connect(){
  //first collect all accesses
  AccessL accs;

  { for_list_(Module, m, mts){ for_list_(Access, a, m->accesses) accs.append(a); } }
  { for_list_(Access, a, accesses) accs.append(a); }

  for_list_(Access, a, accs){
    Module *m=a->module;
    Variable *v = NULL;
    if(!a->var) v = connect(*a, a->name); //access is not connected yet
    else v = dynamic_cast<Variable*>(a->var);

    if(m->thread &&
       ( m->thread->mode==ModuleThread::listenAll ||
         (m->thread->mode==ModuleThread::listenFirst && a==m->accesses(0)) ) ){
      v->listeners.setAppend(m);
    }
  }
}

VariableL createVariables(const ModuleL& ms){
  System S;
  S.mts=ms;
  S.connect();
  cout <<"completed system: " <<S <<endl;
  return S.vars;
}

KeyValueGraph System::graph() const{
  KeyValueGraph g;
  g.append<bool>("SystemModule", name, NULL);
  std::map<VariableAccess*, Item*> vit;
  for_list_(Variable, v, vars) vit[v] = g.append("Variable", v->name, v);
  for_list_(Module, m, mts){
    Item *mit = g.append("Module", m->name, m);
    for_list_(Access, a, m->accesses){
      Item *ait = g.append("Access", a->name, a);
      ait->parents.append(mit);
      if(a->var) ait->parents.append(vit[a->var]);
    }
  }
  return g;
}

void System::write(ostream& os) const{
  cout <<graph() <<endl;
}


//===========================================================================
//
// Engine
//

void signalhandler(int s){
  int calls = engine().shutdown.incrementValue();
  cerr <<"\n*** System received signal " <<s <<" -- count " <<calls;
  if(calls==1){
    cout <<" -- waiting for main loop to break on engine().shutdown.getValue()" <<endl;
  }
  if(calls==2){
    cout <<" -- smoothly closing modules directly" <<endl;
    engine().close(); //might lead to a hangup of the main loop, but processes should close
  }
  if(calls==3){
    cout <<" -- cancelling threads to force closing" <<endl;
    engine().cancel();
  }
  if(calls>3){
    cerr <<" ** shutdown failed - hard exit!" <<endl;
    exit(1);
  }
}

Engine& engine(){  return SingleEngine(); }

Engine::Engine(): mode(none), system(NULL), shutdown(false) {
  acc = new EventController;
};

Engine::~Engine(){
  //acc -> dumpEventList();
  delete acc;
}

void Engine::open(System& S){
  system = &S;

  S.connect();

  signal(SIGINT, signalhandler);

  if(mode==none) mode=threaded;

//  //create pre-defined variables
//  ItemL variables = S.system.getTypedItems<SystemDescription::VariableEntry>("Variable");
//  for_list_(Item, varIt, variables){
//    SystemDescription::VariableEntry *v = varIt->value<SystemDescription::VariableEntry>();
//    cout <<"creating " <<varIt->keys(1) <<": " <<*(v->type) <<endl;
//    v->var = new Variable(varIt->keys(1));
//    v->var->data = v->type->newInstance();
//  }

//  //create modules
//  ItemL modules = S.system.getTypedItems<SystemDescription::ModuleEntry>("Module");
//  for_list_(Item, modIt, modules){
//    SystemDescription::ModuleEntry *m = modIt->value<SystemDescription::ModuleEntry>();
//    cout <<"creating " <<modIt->keys(1) <<": " <<*(m->type) <<endl;
//    if(mode==threaded){
//      Process *proc = new Process(m->type);
//      proc->threadOpen();
//      proc->waitForIdle();
//      m->mod = proc->module;
//      m->mod->name = modIt->keys(1);
//    }else{
//      m->mod = (Module*)m->type->newInstance();
//    }

//    //accesses have automatically been created as member of a module,
//    //need to link them now
//    CHECK(m->mod->accesses.N==modIt->parentOf.N,"dammit");
//    for_list_(Item, accIt, modIt->parentOf){
//      Access *a = m->mod->accesses(accIt_COUNT);
//      //SystemDescription::AccessEntry *acc = accIt->value<SystemDescription::AccessEntry>();
//      //CHECK(acc->type == a->type,"");
//      Item *varIt = accIt->parents(1);
//      CHECK(varIt->keys(0)=="Variable","");
//      SystemDescription::VariableEntry *v = varIt->value<SystemDescription::VariableEntry>();
//      CHECK(v,"");
//      cout <<"linking access " <<modIt->keys(1) <<"->" <<a->name
//          <<" with Variable (" <<*(v->type) <<")" <<endl;
//      a->variable = v->var;
//      a->setData(v->var->data);
//      if(m->mod->proc && m->mode==SystemDescription::listenAll){
//        m->mod->proc->listenTo(a->variable->revision);
//      }
//    }
//  }

  //open modules
  if(mode==threaded){
    for_list_(Module, m, S.mts){
      m->thread->threadOpen();
      //start looping if in loop mode:
      switch(m->thread->mode){
      case ModuleThread::loopWithBeat:  m->thread->threadLoopWithBeat(m->thread->beat);  break;
      case ModuleThread::loopFull:  m->thread->threadLoop();  break;
      default:  break;
      }
    }
  }

  if(mode==serial){
    for_list_(Module, m, S.mts) m->open();
  }
}

void Engine::step(System &S){
  if(&S) system=&S;
  for_list_(Module, m, S.mts) step(*m);
}

void Engine::step(Module &m, bool threadedOnly){
  if(threadedOnly && mode!=threaded) return;
  if(mode==none) MT_MSG("ommiting stepping: no step mode");
  if(mode==threaded) m.thread->threadStep();
  if(mode==serial)   m.step();
}

void Engine::test(System& S){
  if(&S) system=&S;
  CHECK(mode!=threaded,"");
  mode=serial;
  open(S);
  for_list_(Module, m, S.mts) m->test();
  close(S);
}

void Engine::close(System& S){
  if(&S) system=&S;
  for_list_(Module, m, system->mts){
    if(mode==threaded) m->thread->threadClose();
    if(mode==serial)   m->close();
  }
}

void Engine::cancel(System& S){
  if(&S) system=&S;
  for_list_(Module, m, system->mts){
    if(mode==threaded) m->thread->threadCancel();
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

void EventController::queryReadAccess(Variable *v, const ModuleThread *p){
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

void EventController::queryWriteAccess(Variable *v, const ModuleThread *p){
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

void EventController::logReadAccess(const Variable *v, const ModuleThread *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(v, p, EventRecord::read, v->revision.getValue(), p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logWriteAccess(const Variable *v, const ModuleThread *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(v, p, EventRecord::write, v->revision.getValue(), p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logReadDeAccess(const Variable *v, const ModuleThread *p) {
  //do something if in replay mode
  if(getVariableData(v)->controllerBlocksRead)
    breakpointSleep();
}

void EventController::logWriteDeAccess(const Variable *v, const ModuleThread *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(getVariableData(v)->controllerBlocksWrite)
    breakpointSleep();
}

void EventController::logStepBegin(const ModuleThread *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(NULL, p, EventRecord::stepBegin, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logStepEnd(const ModuleThread *p) {
  if(!enableEventLog || enableReplay) return;
  EventRecord *e = new EventRecord(NULL, p, EventRecord::stepEnd, 0, p->step_count, MT::realTime());
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
  uint i;
  EventRecord *e;
  for_list(i,e,copy){
    if(!i && max && copy.N>max){ i=copy.N-max; e=copy(i); }
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
    MT::open(*eventsFile,"z.eventLog");
  }

  writeEventList(*eventsFile, false, 0, true);
}

LoggerVariableData* EventController::getVariableData(const Variable* v){
  if(!v->s->loggerData) v->s->loggerData = new LoggerVariableData();
  return v->s->loggerData;
}


