#include <sys/syscall.h>

#include "engine.h"
#include "engine_internal.h"

Singleton<Engine> single_systemMonitor;

typedef MT::Array<SystemDescription::ModuleEntry*> ModuleEntryL;
typedef MT::Array<SystemDescription::VariableEntry*> VariableEntryL;

//===========================================================================
//
// SystemDescription
//

void SystemDescription::addModule(const char *dclName, const char *name, const ItemL& vars, StepMode mode, double beat){
  //find the dcl in the registry
  Item *modReg = registry().getItem("Decl_Module", STRING(strlen(dclName)<<dclName)); //OpencvCamera::staticRegistrator.reg;
  if(!modReg){
    MT_MSG("could not find Decl_Module" <<dclName);
    return;
  }
  ModuleEntry *m = new ModuleEntry;
  m->reg = modReg;
  m->type = modReg->value<Type>();
  m->mode = mode;
  m->beat = beat;
  Item* modIt = system.append<ModuleEntry>(STRINGS("Module", (name?name:modReg->keys(1))), m);

  for_list_(Item, accReg, modReg->parentOf){
    AccessEntry *a = new AccessEntry;
    a->reg = accReg;
    a->type = accReg->value<Type>();
    if(!&vars || !vars.N){
      system.append<AccessEntry>(STRINGS("Access", accReg->keys(1)), ARRAY(modIt), a);
    }else{
      Item *varIt = vars(accReg_COUNT);
      VariableEntry *v = varIt->value<VariableEntry>();
      cout <<"linking access " <<modIt->keys(1) <<"->" <<accReg->keys(1)
           <<" with Variable (" <<*(v->type) <<")" <<endl;
      system.append<AccessEntry>(STRINGS("Access", varIt->keys(1)), ARRAY(modIt, varIt), new AccessEntry());
    }
  }
}

void SystemDescription::complete(){
#if 0
  ItemL modules = system.getTypedItems<ModuleEntry>("Module");
  for_list_(Item, it, modules){
    ModuleEntry *m=it->value<ModuleEntry>();
    Item *reg=m->reg;
    if(!m->accs.N && reg->parentOf.N){ //declaration has children but description no accesses...
      for_list_(Item, acc, reg->parentOf){
        CHECK(acc->keys(0)=="Decl_Access","");
        Item* varIt = getVariableEntry(acc->keys(1), *acc->value<Type>());
        VariableEntry *v=NULL;
        if(!varIt){ //we need to add a variable
          cout <<"adding-on-complete Variable " <<acc->keys(1) <<": " <<*(acc->value<Type>()) <<endl;
          v = new SystemDescription::VariableEntry;
          v->type = acc->value<Type>();
          varIt = system.append<VariableEntry>(STRINGS("Variable", acc->keys(1)), v);
        }else{
          v = varIt->value<VariableEntry>();
        }
        cout <<"linking-on-complete access " <<it->keys(1) <<"->" <<acc->keys(1)
             <<" with Variable " <<varIt->keys(1) <<"(" <<*(v->type) <<")" <<endl;
        m->accs.append(v);
        system.append<AccessEntry>(STRINGS("Access", acc->keys(1)), ARRAY(it, varIt), new AccessEntry());
      }
    }
  }
#else
  ItemL accesses = system.getTypedItems<AccessEntry>("Access");
  for_list_(Item, accIt, accesses){
    AccessEntry *a=accIt->value<AccessEntry>();
    CHECK(accIt->parents.N==1 || accIt->parents.N==2,"");
    if(accIt->parents.N==1){ //access has no variable yet...
      Item* varIt = getVariableEntry(accIt->keys(1), *a->type);
      VariableEntry *v=NULL;
      if(!varIt){ //we need to add a variable
        cout <<"adding-on-complete Variable " <<accIt->keys(1) <<": " <<*a->type <<endl;
        v = new SystemDescription::VariableEntry;
        v->type = a->type;
        varIt = system.append<VariableEntry>(STRINGS("Variable", accIt->keys(1)), v);
      }else{
        v = varIt->value<VariableEntry>();
      }
      cout <<"linking-on-complete access " <<accIt->parents(0)->keys(1) <<"->" <<accIt->keys(1)
          <<" with Variable (" <<*(v->type) <<")" <<endl;
      accIt->parents.append(varIt);
      varIt->parentOf.append(accIt);
    }
  }
#endif
}

Item* SystemDescription::getVariableEntry(const Access& acc){
  return getVariableEntry(acc.name, *acc.reg->value<Type>());
}

Item* SystemDescription::getVariableEntry(const char* name, const Type& type){
  ItemL variables = system.getTypedItems<VariableEntry>("Variable");
  for_list_(Item, it, variables){
    VariableEntry *v = it->value<VariableEntry>();
    if(it->keys(1)==name){
      if(v->type->typeId()!=type.typeId())
        HALT("mist");
      return it;
    }
  }
  return NULL;
}

void SystemDescription::report(){
  cout <<system <<endl;
}


//===========================================================================
//
// Engine
//

Engine& engine(){  return single_systemMonitor.obj(); }

Engine::Engine(): mode(none), system(NULL) {
  acc = new EventController;
};

Engine::~Engine(){
  //acc -> dumpEventList();
  delete acc;
}

void Engine::create(SystemDescription& S){
  system = &S.system;
  S.complete();

  if(mode==none) mode=serial;

  //create pre-defined variables
  ItemL variables = S.system.getTypedItems<SystemDescription::VariableEntry>("Variable");
  for_list_(Item, varIt, variables){
    SystemDescription::VariableEntry *v = varIt->value<SystemDescription::VariableEntry>();
    cout <<"creating " <<varIt->keys(1) <<": " <<*(v->type) <<endl;
    v->var = new Variable(varIt->keys(1));
    v->var->data = v->type->newInstance();
  }

  //create modules
  ItemL modules = S.system.getTypedItems<SystemDescription::ModuleEntry>("Module");
  for_list_(Item, modIt, modules){
    SystemDescription::ModuleEntry *m = modIt->value<SystemDescription::ModuleEntry>();
    cout <<"creating " <<modIt->keys(1) <<": " <<*(m->type) <<endl;
    if(mode==threaded){
      Process *proc = new Process(m->type);
      proc->threadOpen();
      proc->waitForIdle();
      m->mod = proc->module;
      m->mod->name = modIt->keys(1);
    }else{
      m->mod = (Module*)m->type->newInstance();
    }

    //accesses have automatically been created as member of a module,
    //need to link them now
    CHECK(m->mod->accesses.N==modIt->parentOf.N,"dammit");
    for_list_(Item, accIt, modIt->parentOf){
      Access *a = m->mod->accesses(accIt_COUNT);
      //SystemDescription::AccessEntry *acc = accIt->value<SystemDescription::AccessEntry>();
      //CHECK(acc->type == a->type,"");
      Item *varIt = accIt->parents(1);
      CHECK(varIt->keys(0)=="Variable","");
      SystemDescription::VariableEntry *v = varIt->value<SystemDescription::VariableEntry>();
      CHECK(v,"");
      cout <<"linking access " <<modIt->keys(1) <<"->" <<a->name
          <<" with Variable (" <<*(v->type) <<")" <<endl;
      a->variable = v->var;
      a->setData(v->var->data);
      if(m->mod->proc && m->mode==SystemDescription::listenAll){
        m->mod->proc->listenTo(a->variable->revision);
      }
    }
  }

  //start modules modules
  if(mode==threaded){
    for_list_(Item, modIt, modules){
      SystemDescription::ModuleEntry *m = modIt->value<SystemDescription::ModuleEntry>();
      switch(m->mode){
      case SystemDescription::loopWithBeat:  m->mod->proc->threadLoopWithBeat(m->beat);  break;
      case SystemDescription::loopFull:  m->mod->proc->threadLoop();  break;
      default:  break;
      }
    }
  }
}

void Engine::step(Module &m){
  CHECK(mode!=none,"");
  if(mode==threaded){
    m.proc->threadStep();
  }
  if(mode==serial)   m.step();
}

void Engine::step(SystemDescription& S){
  ModuleEntryL modules = S.system.getTypedValues<SystemDescription::ModuleEntry>("Module");
  for_list_(SystemDescription::ModuleEntry, m, modules) step(*m->mod);
}

void Engine::test(SystemDescription& S){
  CHECK(mode!=threaded,"");
  mode=serial;
  create(S);
  ModuleEntryL modules = S.system.getTypedValues<SystemDescription::ModuleEntry>("Module");
  for_list_(SystemDescription::ModuleEntry, m, modules) m->mod->test();
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


