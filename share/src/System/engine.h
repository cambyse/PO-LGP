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
#include <Core/module.h>

struct Variable_SharedMemory;
struct Module_Thread;
typedef MT::Array<Module*> ModuleL;
typedef MT::Array<Module_Thread*> Module_ThreadL;
typedef MT::Array<Variable_SharedMemory*> VariableL;
typedef MT::Array<Access*> AccessL;

//===========================================================================

VariableL createVariables(const ModuleL& ms); ///< same as System::connect();

//===========================================================================
/**
 * Implements a Module as a Thread
 */
struct Module_Thread:Thread{
  enum StepMode { listenFirst=0, listenAll, loopWithBeat, loopFull };
  Module *m;
  uint step_count;
  StepMode mode; double beat;

  /// @name c'tor/d'tor
  Module_Thread(Module* _m, const char* _name=NULL):Thread(_name?_name:_m->name),m(_m),step_count(0){ m->name = _name; }

  virtual void open(){ m->open(); }
  virtual void step();
  virtual void close(){ m->close(); }
};
//inline void operator>>(istream& is, Module_Thread& m){  }
inline void operator<<(ostream& os, const Module_Thread& m){ os <<"Module_Thread " <<m.name <<' ' <<m.step_count; }


//===========================================================================
/**
 * Implements a Variable (something that Modules can access) as mutex shared memory
 */

struct Variable_SharedMemory : Variable {
  struct sVariable *s;        ///< private
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess -- but views may access directly...)
  ConditionVariable revision; ///< revision (= number of write accesses) number
  double revision_time;       ///< clock time of last write access
  ModuleL listeners;          ///< list of modules that are being signaled a threadStep on write access

  /// @name c'tor/d'tor
  Variable_SharedMemory(const char* name);
  virtual ~Variable_SharedMemory();

  /// @name access control
  /// to be called by a processes before access, returns the revision
  int readAccess(Module*);  //might set the caller to sleep
  int writeAccess(Module*); //might set the caller to sleep
  int deAccess(Module*);

  /// @name syncing via a variable
  /// the caller is set to sleep
  int waitForNextRevision();
  int waitForRevisionGreaterThan(int rev); //returns the revision
  double revisionTime();
  int revisionNumber();

  /// @name info (fields are currently not used anymore)
  struct FieldRegistration& get_field(uint i) const;
};
inline void operator<<(ostream& os, const Variable_SharedMemory& v){ os <<"Variable " <<v.name <<' ' <<*v.type; }


//===========================================================================
/**
 * A list of Modules and Variables that can be autoconnected and, as a group, opened, stepped and closed
 */

struct System:Module{
  ModuleL mts;
  VariableL vars;

  System(const char* name=NULL):Module(name){}

  virtual void step(){  for(Module *m: mts) m->step();  }
  virtual void open(){  for(Module *m: mts) m->open();  }
  virtual void close(){  for(Module *m: mts) m->close();  }

  //-- add variables
  Variable_SharedMemory* addVariable(const char *name, Type *type){
    Variable_SharedMemory *v = new Variable_SharedMemory(name);
    v->type = type->clone();
    v->data = type->newInstance();
    vars.append(v);
    return v;
  }

  Variable_SharedMemory* addVariable(Access& acc){ return addVariable(acc.name, acc.type); }
  template<class T> Variable_SharedMemory* addVariable(const char *name){ return addVariable(name, new Type_typed<T, void>()); }

  //-- access vars
  template<class T> T& getVar(uint i){ return *((T*)vars(i)->data); }

  template<class T> Access_typed<T>* getAccess(const char* varName){
    Variable_SharedMemory *v = listFindByName(vars, varName);
    return new Access_typed<T>(varName, v);
  }

  //-- add modules
  template<class T> T* addModule(const char *name=NULL, Module_Thread::StepMode mode=Module_Thread::listenFirst, double beat=0.){
    T *m = new T;
    currentlyCreating=NULL;
    for(Access *a: m->accesses) a->module = m;
    mts.append(m);

    m->thread = new Module_Thread(m, name);
    m->thread->mode = mode;
    m->thread->beat = beat;
    return m;
  }

  //-- add modules
  template<class T> T* addModule(const char *name, const StringA& accessConnectRules, Module_Thread::StepMode mode=Module_Thread::listenFirst, double beat=0.){
    T *m = addModule<T>(name, mode, beat);
    if(accessConnectRules.N != m->accesses.N) HALT("given and needed #acc in accessConnectRules cmismatch");
    for_list(Access, a, m->accesses) connect(*a, accessConnectRules(a_COUNT));
    return m;
  }

  Module* addModule(const char *dclName, const char *name=NULL, Module_Thread::StepMode mode=Module_Thread::listenFirst, double beat=0.);
  Module* addModule(const char *dclName, const char *name, const uintA& accIdxs, Module_Thread::StepMode mode=Module_Thread::listenFirst, double beat=0.);
  Module* addModule(const char *dclName, const char *name, const StringA& accRenamings, Module_Thread::StepMode mode=Module_Thread::listenFirst, double beat=0.);

  /** instantiate all the necessary variables for the list of modules, i.e.,
   *  check all accesses they have, match their names and types, create
   *  the necessary variables, and link the accesses to them  */
  void connect();
  // [sort of private] check if Variable with variable_name and acc.type exists; if not, create one; then connect
  Variable_SharedMemory* connect(Access& acc, const char *variable_name);

  KeyValueGraph graph() const;
  void write(ostream& os) const;
};
stdOutPipe(System);

extern System& NoSystem;


//===========================================================================
/**
 * A singleton that can run systems, here implemented using threads
 */

struct Engine{
  struct EventController *acc;
  enum { none=0, serial, threaded } mode;
  System *system;
  AccessL createdAccesses;
  ConditionVariable shutdown;

  Engine();
  virtual ~Engine();

  void open(System& S);
  void step(Module &m, bool threadedOnly=false);
  void step(System& S=NoSystem);
  void test(System& S=NoSystem);
  void close(System& S=NoSystem);
  void cancel(System& S=NoSystem);

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
  const Variable_SharedMemory *variable;
  const Module_Thread *module;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  EventRecord(const Variable_SharedMemory *v, const Module_Thread *m, EventType _type, uint _revision, uint _procStep, double _time):
    variable(v), module(m), type(_type), revision(_revision), procStep(_procStep), time(_time){}
};

typedef MT::Array<EventRecord*> EventRecordL;


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
  ConditionVariable blockMode; //0=all_run, 1=next_runs, 2=none_runs
  EventRecordL blockedEvents;

  ofstream* eventsFile;

  EventController();
  ~EventController();

  struct LoggerVariableData* getVariableData(const Variable_SharedMemory *v);

  //writing into a file
  void writeEventList(ostream& os, bool blockedEvents, uint max=0, bool clear=false);
  void dumpEventList();

  //methods called during write/read access from WITHIN the Variable
  void queryReadAccess(Variable_SharedMemory *v, const Module_Thread *p);
  void queryWriteAccess(Variable_SharedMemory *v, const Module_Thread *p);
  void logReadAccess(const Variable_SharedMemory *v, const Module_Thread *p);
  void logReadDeAccess(const Variable_SharedMemory *v, const Module_Thread *p);
  void logWriteAccess(const Variable_SharedMemory *v, const Module_Thread *p);
  void logWriteDeAccess(const Variable_SharedMemory *v, const Module_Thread *p);
  void logStepBegin(const Module_Thread *p);
  void logStepEnd(const Module_Thread *p);

  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};

#endif
