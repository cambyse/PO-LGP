#include <sys/syscall.h>

#include "engine.h"
#include "engine_internal.h"

Singleton<Engine> single_systemMonitor;

//===========================================================================
//
// SystemDescription
//

void SystemDescription::newModule(const char *name, const char *dclName, const VariableEntryL& vars){
  ModuleEntry m;
  m.name = name;
  //find the dcl in the registry
  Item *reg = registry().getItem("Decl_Module", STRING(strlen(dclName)<<dclName)); //OpencvCamera::staticRegistrator.reg;
  if(!reg){
    MT_MSG("could not find Decl_Module" <<dclName);
    return;
  }
  m.dcl = reg->value<TypeInfo>();

  //find the vars for explicit connecting
  if(vars.N){
    CHECK(vars.N == reg->parentOf.N,"");
    for(uint i=0;i<vars.N;i++){
      cout <<"linking access " <<m.name <<"->" <<reg->parentOf(i)->keys(1)
           <<" with Variable " <<vars(i)->name <<" (" <<*(vars(i)->dcl) <<")" <<endl;
      m.accs.append(vars(i));
    }
  }

  modules.append(m);
}

void SystemDescription::report(){
  cout <<"** Variables" <<endl;
  for_list_(VariableEntry, v, variables){
    cout <<v->name <<": " <<*(v->dcl) <<endl;
  }
  cout <<"** Modules" <<endl;
  for_(ModuleEntry, m, modules){
    cout <<m->name <<": " <<*(m->dcl) <<" with " <<m->accs.N <<" accesses" <<endl;
  }
}

void SystemDescription::complete(){
  //TODO
  //go through all modules, check if they're linked to variables, create variables as needed
}


//===========================================================================
//
// Engine
//

Engine& engine(){  return single_systemMonitor.obj(); }

Engine::Engine(): mode(none) {
  acc = new EventController;
};

Engine::~Engine(){
  //acc -> dumpEventList();
  delete acc;
}

void Engine::create(SystemDescription& S){
  if(mode==none) mode=serial;
  for_list_(SystemDescription::VariableEntry, v, S.variables){
    cout <<"creating " <<v->name <<": " <<*(v->dcl) <<endl;
    v->var = new Variable(v->name);
    v->var->data = v->dcl->newInstance();
  }
  for_(SystemDescription::ModuleEntry, m, S.modules){
    cout <<"creating " <<m->name <<": " <<*(m->dcl) <<endl;
    if(mode==threaded){
      Process *proc = new Process(m->dcl);
      proc->threadOpen();
      proc->waitForIdle();
      m->mod = proc->module;
      m->mod->name = m->name;
    }else{
      m->mod = (Module*)m->dcl->newInstance();
    }

    CHECK(m->mod->accesses.N==m->accs.N,"");
    for(uint i=0;i<m->accs.N;i++){
      Access *a = m->mod->accesses(i);
      SystemDescription::VariableEntry *v = m->accs(i);
      cout <<"linking access " <<m->name <<"->" <<a->name
          <<" with Variable " <<v->name <<" (" <<*(v->dcl) <<")" <<endl;
      a->variable = v->var;
      a->setData(v->var->data);
      if(mode==threaded){
        m->mod->proc->listenTo(a->variable);
      }
    }
  }
}

void Engine::step(Module &m){
  CHECK(mode!=none,"");
  if(mode==threaded) m.proc->threadStep();
  if(mode==serial)   m.step();
}

void Engine::step(SystemDescription& S){
  for_(SystemDescription::ModuleEntry, m, S.modules) step(*m->mod);
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

void EventController::queryReadAccess(Variable *v, const Module *p){
  blockMode.lock();
  if(blockMode.value>=1){
    Event *e = new Event(v, p, Event::read, v->revision.getValue(), p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2; //1: only ONE reader
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void EventController::queryWriteAccess(Variable *v, const Module *p){
  blockMode.lock();
  if(blockMode.value>=1){
    Event *e = new Event(v, p, Event::write, v->revision.getValue(), p?p->step_count:0, 0.);
    blockedEvents.append(e);
    blockMode.waitForValueSmallerThan(2, true);
    if(blockMode.value==1) blockMode.value=2;
    blockedEvents.removeValue(e);
    delete e;
  }
  blockMode.unlock();
}

void EventController::logReadAccess(const Variable *v, const Module *p) {
  if(!enableEventLog || enableReplay) return;
  Event *e = new Event(v, p, Event::read, v->revision.getValue(), p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logWriteAccess(const Variable *v, const Module *p) {
  if(!enableEventLog || enableReplay) return;
  Event *e = new Event(v, p, Event::write, v->revision.getValue(), p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logReadDeAccess(const Variable *v, const Module *p) {
  //do something if in replay mode
  if(getVariableData(v)->controllerBlocksRead)
    breakpointSleep();
}

void EventController::logWriteDeAccess(const Variable *v, const Module *p) {
  //do something if in replay mode
  //do something if enableDataLog
  if(getVariableData(v)->controllerBlocksWrite)
    breakpointSleep();
}

void EventController::logStepBegin(const Module *p) {
  if(!enableEventLog || enableReplay) return;
  Event *e = new Event(NULL, p, Event::stepBegin, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logStepEnd(const Module *p) {
  if(!enableEventLog || enableReplay) return;
  Event *e = new Event(NULL, p, Event::stepEnd, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::writeEventList(ostream& os, bool blocked, uint max, bool clear){
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
  Event *e;
  for_list(i,e,copy){
    if(!i && max && copy.N>max){ i=copy.N-max; e=copy(i); }
    switch(e->type){
    case Event::read: os <<'r';  break;
    case Event::write: os <<'w';  break;
    case Event::stepBegin: os <<'b';  break;
    case Event::stepEnd: os <<'e';  break;
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


//===========================================================================
//
// Process
//

Process::Process(TypeInfo *_moduleDcl): module(NULL), moduleDcl(_moduleDcl), state(tsCLOSE), tid(0), metronome(NULL)  {
  listensTo.memMove=true;
//  biros().writeAccess(module);
//  biros().processes.memMove=true;
//  biros().processes.append(this);
//  biros().deAccess(module);
}

Process::~Process() {
  if(Thread::isOpen() || state.value!=tsCLOSE) threadClose();
//  biros().writeAccess(module);
//  biros().processes.removeValue(this);
//  biros().deAccess(module);
}

void Process::threadOpen(int priority) {
  state.lock();
  if(isOpen()){ state.unlock(); return; } //this is already open -- or has just beend opened (parallel call to threadOpen)
  Thread::open(STRING("--TODO: module name"));
  state.value=tsOPENING;
  state.unlock();
}

void Process::threadClose() {
  state.setValue(tsCLOSE);
  Thread::close();
}

void Process::threadStep(uint steps, bool wait) {
  if(!Thread::isOpen()) threadOpen();
  //CHECK(state.value==tsIDLE, "never step while thread is busy!");
  state.setValue(steps);
  if(wait) waitForIdle();
}

void Process::listenTo(const VariableL &signalingVars) {
  uint i;  Variable *v;
  for_list(i, v, signalingVars) listenTo(v);
}

void Process::listenTo(Variable *v) {
  v->rwlock.writeLock(); //don't want to increase revision and broadcast!
  v->s->listeners.setAppend(module);
  v->rwlock.unlock();
  listensTo.setAppend(v);
}

void Process::stopListeningTo(Variable *v){
  v->rwlock.writeLock(); //don't want to increase revision and broadcast!
  v->s->listeners.removeValue(module);
  v->rwlock.unlock();
  listensTo.removeValue(v);
}

bool Process::isIdle() {
  return state.getValue()==tsIDLE;
}

bool Process::isClosed() {
  return state.getValue()==tsCLOSE;
}

void Process::waitForIdle() {
  state.waitForValueEq(tsIDLE);
}

void Process::threadLoop() {
  if(!Thread::isOpen()) threadOpen();
  state.setValue(tsLOOPING);
}

void Process::threadLoopWithBeat(double sec) {
  if(!metronome)
    metronome=new Metronome("threadTiccer", 1000.*sec);
  else
    metronome->reset(1000.*sec);
  if(!Thread::isOpen()) threadOpen();
  state.setValue(tsBEATING);
}

void Process::threadStop() {
  CHECK(Thread::isOpen(), "called stop to closed thread");
  state.setValue(tsIDLE);
}

void Process::main() {
  tid = syscall(SYS_gettid);

  //http://linux.die.net/man/3/setpriority
  //if(Thread::threadPriority) setRRscheduling(Thread::threadPriority);
  //if(Thread::threadPriority) setNice(Thread::threadPriority);

  //c'tor of module
  CHECK(module==NULL,"module mustn't exist yet");
  module = (Module*)moduleDcl->newInstance();
  module->proc = this;

  state.lock();
  if(state.value==tsOPENING){
    state.value=tsIDLE;
    state.broadcast();
  }
  //if not =tsOPENING anymore -> the state was set on looping or beating already
  state.unlock();


  //timer.reset();
  bool waitForTic=false;
  for(;;){
    //-- wait for a non-idle state
    state.lock();
    state.waitForValueNotEq(tsIDLE, true);
    if(state.value==tsCLOSE) { state.unlock();  break; }
    if(state.value==tsBEATING) waitForTic=true; else waitForTic=false;
    if(state.value>0) state.value--; //count down
    state.unlock();

    if(waitForTic) metronome->waitForTic();

    //-- make a step
    engine().acc->logStepBegin(module);
    module->step(); //virtual step routine
    module->step_count++;
    engine().acc->logStepEnd(module);

    //-- broadcast in case somebody was waiting for a finished step
    state.lock();
    state.broadcast();
    state.unlock();
  };

  //d'tor of module
  delete module;
  module=NULL;
}
