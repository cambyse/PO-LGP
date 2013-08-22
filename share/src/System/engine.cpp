#include <sys/syscall.h>
#include <map>

#include "engine.h"
#include "engine_internal.h"

Singleton<Engine> singleton_Engine;

//===========================================================================
//
// Variable
//

Variable::Variable(const char *_name):DataAccess(_name), s(NULL), revision(0), reg(NULL) {
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
  engine().acc->logWriteAccess(this, p);
  uint i; ModuleThread *l;
  for_list(i, l, listeners) if(l!=p) engine().step(*l, true);
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

void Variable::waitForNextWriteAccess(){
  revision.waitForSignal();
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

ModuleThread *System::addModule(const char *dclName, const char *name, ModuleThread::StepMode mode, double beat){
  //find the dcl in the registry
  Item *modReg = registry().getItem("Decl_Module", dclName);
  if(!modReg){
    MT_MSG("could not find Decl_Module " <<dclName);
    return NULL;
  }
  Module *m = (Module*)modReg->value<Type>()->newInstance();
  for_list_(Access, a, m->accesses) a->module = m;
  ModuleThread *mt = new ModuleThread(m, name?name:dclName);
  mt->mode = mode;
  mt->beat = beat;
  mts.append(mt);
  return mt;
}

void System::addModule(const char *dclName, const char *name, const uintA& accIdxs, ModuleThread::StepMode mode, double beat){
  ModuleThread *mt = addModule(dclName, name, mode, beat);
  if(accIdxs.N != mt->m->accesses.N) HALT("given and needed #acc mismatch");
  for_list_(Access, a, mt->m->accesses) a->var = vars(accIdxs(a_COUNT));
}

void System::addModule(const char *dclName, const char *name, const StringA& accNames, ModuleThread::StepMode mode, double beat){
  ModuleThread *mt = addModule(dclName, name, mode, beat);
  if(accNames.N != mt->m->accesses.N) HALT("given and needed #acc mismatch");
  for_list_(Access, a, mt->m->accesses) a->name = accNames(a_COUNT);
}

//  Item* modIt = system.append<ModuleEntry>(STRINGS("Module", (name?name:modReg->keys(1))), m);

//  for_list_(Item, accReg, modReg->parentOf){
//    AccessEntry *a = new AccessEntry;
//    a->reg = accReg;
//    a->type = accReg->value<Type>();
//    if(!&vars || !vars.N){
//      system.append<AccessEntry>(STRINGS("Access", accReg->keys(1)), ARRAY(modIt), a);
//    }else{
//      Item *varIt = vars(accReg_COUNT);
//      VariableEntry *v = varIt->value<VariableEntry>();
//      cout <<"linking access " <<modIt->keys(1) <<"->" <<accReg->keys(1)
//           <<" with Variable (" <<*(v->type) <<")" <<endl;
//      system.append<AccessEntry>(STRINGS("Access", varIt->keys(1)), ARRAY(modIt, varIt), new AccessEntry());
//    }
//  }

void System::complete(){
//#if 0
//  ItemL modules = system.getTypedItems<ModuleEntry>("Module");
//  for_list_(Item, it, modules){
//    ModuleEntry *m=it->value<ModuleEntry>();
//    Item *reg=m->reg;
//    if(!m->accs.N && reg->parentOf.N){ //declaration has children but description no accesses...
//      for_list_(Item, acc, reg->parentOf){
//        CHECK(acc->keys(0)=="Decl_Access","");
//        Item* varIt = getVariableEntry(acc->keys(1), *acc->value<Type>());
//        VariableEntry *v=NULL;
//        if(!varIt){ //we need to add a variable
//          cout <<"adding-on-complete Variable " <<acc->keys(1) <<": " <<*(acc->value<Type>()) <<endl;
//          v = new SystemDescription::VariableEntry;
//          v->type = acc->value<Type>();
//          varIt = system.append<VariableEntry>(STRINGS("Variable", acc->keys(1)), v);
//        }else{
//          v = varIt->value<VariableEntry>();
//        }
//        cout <<"linking-on-complete access " <<it->keys(1) <<"->" <<acc->keys(1)
//             <<" with Variable " <<varIt->keys(1) <<"(" <<*(v->type) <<")" <<endl;
//        m->accs.append(v);
//        system.append<AccessEntry>(STRINGS("Access", acc->keys(1)), ARRAY(it, varIt), new AccessEntry());
//      }
//    }
//  }
//#else
  for_list_(ModuleThread, m, mts){
    for_list_(Access, a, m->m->accesses){
      Variable *v = NULL;
      if(!a->var){ //access is not connected yet
         v = listFindByName(vars, a->name);
        if(v){ //variable exists -> check type
          if(*v->type != *a->type) HALT("dammit!");
          //good: just connect
          a->var = v;
        }else{ //variable does not exist yet
          a->var = v = addVariable(a);
        }
      }else{
        v = dynamic_cast<Variable*>(a->var);
      }
      if(m->mode==ModuleThread::listenAll || (m->mode==ModuleThread::listenFirst && !a_COUNT)){
        v->listeners.setAppend(m);
      }
    }
  }

//  ItemL accesses = system.getTypedItems<AccessEntry>("Access");
//  for_list_(Item, accIt, accesses){
//    AccessEntry *a=accIt->value<AccessEntry>();
//    CHECK(accIt->parents.N==1 || accIt->parents.N==2,"");
//    if(accIt->parents.N==1){ //access has no variable yet...
//      Item* varIt = getVariableEntry(accIt->keys(1), *a->type);
//      VariableEntry *v=NULL;
//      if(!varIt){ //we need to add a variable
//        cout <<"adding-on-complete Variable " <<accIt->keys(1) <<": " <<*a->type <<endl;
//        v = new SystemDescription::VariableEntry;
//        v->type = a->type;
//        varIt = system.append<VariableEntry>(STRINGS("Variable", accIt->keys(1)), v);
//      }else{
//        v = varIt->value<VariableEntry>();
//      }
//      cout <<"linking-on-complete access " <<accIt->parents(0)->keys(1) <<"->" <<accIt->keys(1)
//          <<" with Variable (" <<*(v->type) <<")" <<endl;
//      accIt->parents.append(varIt);
//      varIt->parentOf.append(accIt);
//    }
//  }
//#endif
}

//Item* SystemDescription::getVariableEntry(const Access& acc){
//  return getVariableEntry(acc.name, *acc.reg->value<Type>());
//}

//Item* SystemDescription::getVariableEntry(const char* name, const Type& type){
//  ItemL variables = system.getTypedItems<VariableEntry>("Variable");
//  for_list_(Item, it, variables){
//    VariableEntry *v = it->value<VariableEntry>();
//    if(it->keys(1)==name){
//      if(v->type->typeId()!=type.typeId())
//        HALT("mist");
//      return it;
//    }
//  }
//  return NULL;
//}


KeyValueGraph System::graph() const{
  KeyValueGraph g;
  std::map<DataAccess*, Item*> vit;
  for_list_(Variable, v, vars) vit[v] = g.append("Variable", v->name.p, v);
  for_list_(ModuleThread, m, mts){
    Item *mit = g.append("ModuleThread", m->name, m);
    for_list_(Access, a, m->m->accesses){
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

Engine& engine(){  return singleton_Engine.obj(); }

Engine::Engine(): mode(none), system(NULL) {
  acc = new EventController;
};

Engine::~Engine(){
  //acc -> dumpEventList();
  delete acc;
}

void Engine::open(System& S){
  S.complete();

  if(mode==none) mode=serial;

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
    for_list_(ModuleThread, m, S.mts){
      m->threadOpen();
      //start looping if in loop mode:
      switch(m->mode){
      case ModuleThread::loopWithBeat:  m->threadLoopWithBeat(m->beat);  break;
      case ModuleThread::loopFull:  m->threadLoop();  break;
      default:  break;
      }
    }
  }

  if(mode==serial){
    for_list_(ModuleThread, m, S.mts) m->open();
  }
}

void Engine::step(ModuleThread &m, bool threadedOnly){
  if(threadedOnly && mode!=threaded) return;
  if(mode==none) MT_MSG("ommiting stepping: no step mode");
  if(mode==threaded) m.threadStep();
  if(mode==serial)   m.step();
}

void Engine::step(System& S){
  for_list_(ModuleThread, m, S.mts) step(*m);
}

void Engine::test(System& S){
  CHECK(mode!=threaded,"");
  mode=serial;
  open(S);
  for_list_(ModuleThread, m, S.mts) m->m->test();
  close(S);
}

void Engine::close(System& S){
  for_list_(ModuleThread, m, S.mts){
    if(mode==threaded) m->threadClose();
    if(mode==serial)   m->close();
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
    Event *e = new Event(v, p, Event::read, v->revision.getValue(), p?p->step_count:0, 0.);
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
    Event *e = new Event(v, p, Event::write, v->revision.getValue(), p?p->step_count:0, 0.);
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
  Event *e = new Event(v, p, Event::read, v->revision.getValue(), p?p->step_count:0, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logWriteAccess(const Variable *v, const ModuleThread *p) {
  if(!enableEventLog || enableReplay) return;
  Event *e = new Event(v, p, Event::write, v->revision.getValue(), p?p->step_count:0, MT::realTime());
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
  Event *e = new Event(NULL, p, Event::stepBegin, 0, p->step_count, MT::realTime());
  eventsLock.writeLock();
  events.append(e);
  eventsLock.unlock();
  if(events.N>100) dumpEventList();
}

void EventController::logStepEnd(const ModuleThread *p) {
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


