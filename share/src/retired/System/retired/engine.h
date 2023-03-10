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
#ifndef system_engine_h
#define system_engine_h

#include <Core/thread.h>
#include <Core/thread.h>

struct Module;
typedef mlr::Array<Module*> ModuleL;
typedef mlr::Array<VariableBase*> VariableL;

//===========================================================================

//VariableL createVariables(const ModuleL& ms); ///< internally calls System::connect();

//===========================================================================
/**
 * A list of Modules and Variables that can be autoconnected and, as a group, opened, stepped and closed
 */

struct System:ModuleL{
//  ModuleL modules;
  VariableL vars;
  AccessL accesses;
  mlr::String name;

  System(const char* _name=NULL):name(_name){
    currentlyCreatingAccessL=&accesses;
  }

  virtual void stepAll(){  for(Module *m: *this) m->step();  }
  virtual void openAll(){  for(Module *m: *this) m->open();  }
  virtual void closeAll(){  for(Module *m: *this) m->close();  }

  void run(bool waitForOpened=true);
  void close(){
    for(Module *m: *this)  m->threadClose();
  }


  //-- add variables
  template<class T> VariableData<T>* addVariable(const char *name){
    VariableData<T> *v = new VariableData<T>(name);
    vars.append(v);
    return v;
  }

  //-- access vars
  template<class T> VariableData<T>* getVar(uint i){ return dynamic_cast<VariableData<T>* >(vars.elem(i)); }
  template<class T> Access<T> getConnectedAccess(const char* varName){
    Access<T> acc(varName);
    VariableBase *v = listFindByName(vars, varName);
    if(v){ //variable exists -> link it
      acc.linkToVariable(v);
    }else{ //variable does not exist yet
      acc.createVariable(varName);
    }
    return acc;
  }

  //-- add modules
  void addModule(Module& m, Module::StepMode mode=Module::listenFirst, double beat=0.){
    currentlyCreating=NULL;
    for(Access *a: m.accesses) CHECK(a->module == &m,"");
    this->append(&m);
    m.mode = mode;
    m.beat = beat;
  }

  template<class T> T* addModule(const char *name=NULL, Module::StepMode mode=Module::listenFirst, double beat=0.){
    T *m = new T;
    CHECK(dynamic_cast<Module*>(m)!=NULL, "this thing is not derived from Module");
    addModule(*m, mode, beat);
    return m;
  }

  //-- add modules
  template<class T> T* addModule(const char *name, const StringA& accessConnectRules, Module::StepMode mode=Module::listenFirst, double beat=0.){
    T *m = addModule<T>(name, mode, beat);
    if(accessConnectRules.N != m->accesses.N) HALT("given and needed #acc in accessConnectRules cmismatch");
    for_list(Access, a, m->accesses) connect(*a, accessConnectRules(a_COUNT));
    return m;
  }

  Module* addModule(const char *dclName, const char *name=NULL, Module::StepMode mode=Module::listenFirst, double beat=0.);
//  Module* addModule(const char *dclName, const char *name, const uintA& accIdxs, Module::StepMode mode=Module::listenFirst, double beat=0.);
  Module* addModule(const char *dclName, const char *name, const StringA& accRenamings, Module::StepMode mode=Module::listenFirst, double beat=0.);

  /** instantiate all the necessary variables for the list of modules, i.e.,
   *  check all accesses they have, match their names and types, create
   *  the necessary variables, and link the accesses to them  */
  void //connect();

  // [sort of private] check if Variable with variable_name and acc.type exists; if not, create one; then connect
  VariableBase* connect(Access& acc, const char *variable_name);

  Graph graph() const;
  void write(ostream& os) const;
};
stdOutPipe(System);

extern System& NoSystem;


//===========================================================================
/**
 * A singleton that can run systems
 */

struct Engine{
  struct EventController *acc;
  enum { none=0, serial, threaded } mode;
  System *system;
  AccessL createdAccesses;
  Signaler shutdown;

  Engine();
  virtual ~Engine();

  void open(System& S, bool waitForOpened=true);
  void step(Module &m, bool threadedOnly=false);
  void step(System& S=NoSystem);
  void test(System& S=NoSystem);
  void close(System& S=NoSystem);
  void cancel(System& S=NoSystem);

  void waitForShutdownSignal(){ moduleShutdown()->waitForStatusGreaterThan(0); }

  /// @name event control
  void enableAccessLog();
  void dumpAccessLog();
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
};

/// returns the singleton
Engine& engine();


//===========================================================================
/**
 * A minimal data structure to hold an 'event record': a description of a read, write, step-begin or step-end event
 */

struct EventRecord{
  const VariableBase *variable;
  const Module *module;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  EventRecord(const VariableBase *v, const Module *m, EventType _type, uint _revision, uint _procStep, double _time):
    variable(v), module(m), type(_type), revision(_revision), procStep(_procStep), time(_time){}
};

typedef mlr::Array<EventRecord*> EventRecordL;


//===========================================================================
/**
 * Logging and controlling events. Controlling mostly means blocking them, thereby enabling low-level controll over the
 * system's scheduling/stepping of modules.
 * This is a member of the singleton engine.
 */

struct EventController{
  bool enableEventLog;
  bool enableDataLog;
  bool enableReplay;

  EventRecordL events;
  RWLock eventsLock;
  Signaler blockMode; //0=all_run, 1=next_runs, 2=none_runs
  EventRecordL blockedEvents;

  ofstream* eventsFile;

  EventController();
  ~EventController();

  struct LoggerVariableData* getVariableData(const VariableBase *v);

  //writing into a file
  void writeEventList(ostream& os, bool blockedEvents, uint max=0, bool clear=false);
  void dumpEventList();

  //methods called during write/read access from WITHIN the Variable
  void queryReadAccess(VariableBase *v, const Module *p);
  void queryWriteAccess(VariableBase *v, const Module *p);
  void logReadAccess(const VariableBase *v, const Module *p);
  void logReadDeAccess(const VariableBase *v, const Module *p);
  void logWriteAccess(const VariableBase *v, const Module *p);
  void logWriteDeAccess(const VariableBase *v, const Module *p);
  void logStepBegin(const Module *p);
  void logStepEnd(const Module *p);

  mlr::Array<Signaler*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};

#endif
