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

#ifndef system_engine_h
#define system_engine_h

#include <Core/thread.h>
#include <Core/module.h>

struct Variable;
struct ModuleThread;
typedef MT::Array<Module*> ModuleL;
typedef MT::Array<ModuleThread*> ModuleThreadL;
typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Access*> AccessL;

VariableL createVariables(const ModuleL& ms);


//===========================================================================
/**
 * Implements a Module as a Thread
 */
struct ModuleThread:Thread{
  enum StepMode { listenFirst=0, listenAll, loopWithBeat, loopFull };
  Module *m;
  uint step_count;
  StepMode mode; double beat;

  /// @name c'tor/d'tor
  ModuleThread(Module* _m, const char* _name=NULL):Thread(_name?_name:m->name),m(_m),step_count(0){ m->name = _name; }

  virtual void open(){ m->open(); }
  virtual void step(){ m->step(); step_count++; }
  virtual void close(){ m->close(); }
};
//inline void operator>>(istream& is, ModuleThread& m){  }
inline void operator<<(ostream& os, const ModuleThread& m){ os <<"ModuleThread " <<m.name <<' ' <<m.step_count; }


//===========================================================================
/**
 * Implements a VariableAccess (something that Modules can access) as mutex shared memory
 */

struct Variable : VariableAccess {
  struct sVariable *s;        ///< private
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess -- but views may access directly...)
  ConditionVariable revision; ///< revision (= number of write accesses) number
  double revision_time;       ///< clock time of last write access
  ModuleL listeners;          ///< list of modules that are being signaled a threadStep on write access

  /// @name c'tor/d'tor
  Variable(const char* name);
  virtual ~Variable();

  /// @name access control
  /// to be called by a processes before access, returns the revision
  int readAccess(Module*);  //might set the caller to sleep
  int writeAccess(Module*); //might set the caller to sleep
  int deAccess(Module*);

  /// @name syncing via a variable
  /// the caller is set to sleep
  int waitForNextWriteAccess();
  int waitForRevisionGreaterThan(int rev); //returns the revision
  double revisionTime();

  /// @name info (fields are currently not used anymore)
  struct FieldRegistration& get_field(uint i) const;
};
inline void operator<<(ostream& os, const Variable& v){ os <<"Variable " <<v.name <<' ' <<*v.type; }


//===========================================================================
/**
 * A list of Modules and Variables that can be autoconnected and, as a group, opened, stepped and closed
 */

struct System:Module{
  ModuleL mts;
  VariableL vars;

  System(const char* name=NULL):Module(name){}

  virtual void step(){  for_list_(Module, m, mts) m->step();  }
  virtual void open(){  for_list_(Module, m, mts) m->open();  }
  virtual void close(){  for_list_(Module, m, mts) m->close();  }

  Variable* addVariable(Access& acc){
    Variable *v = new Variable(acc.name);
    v->type = acc.type->clone();
    v->data = v->type->newInstance();
    vars.append(v);
    return v;
  }

  template<class T> Variable* addVariable(const char *name){
    Variable *v = new Variable(name);
    v->type = new Type_typed<T, void>();
    v->data = new T;
    vars.append(v);
    return v;
  }

  template<class T> T& getVar(uint i){ return *((T*)vars(i)->data); }

  template<class T> Access_typed<T>* getAccess(const char* varName){
    Variable *v = listFindByName(vars, varName);
    return new Access_typed<T>(varName, NULL, v);
  }

  template<class T> T* addModule(const char *name=NULL, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.){
    T *m = new T;
    currentlyCreating=NULL;
    for_list_(Access, a, m->accesses) a->module = m;
    mts.append(m);

    m->thread = new ModuleThread(m, name);
    m->thread->mode = mode;
    m->thread->beat = beat;
    return m;
  }

  Module* addModule(const char *dclName, const char *name=NULL, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  void addModule(const char *dclName, const char *name, const uintA& accIdxs, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  void addModule(const char *dclName, const char *name, const StringA& accRenamings, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);


  KeyValueGraph graph() const;
  void write(ostream& os) const;
  void connect();
  Variable* connect(Access& acc, const char *variable_name);
};
stdOutPipe(System);

extern System& NoSystem;


//===========================================================================
/**
 * A singleton that can run (open) systems, here implemented using threads
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

  /// @name event control
  void enableAccessLog();
  void dumpAccessLog();
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
};

Engine& engine();


//===========================================================================
/**
 * A minimal data structure to hold an 'event record': a description of a read, write, step-begin or step-end event
 */

struct EventRecord{
  const Variable *variable;
  const ModuleThread *module;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  EventRecord(const Variable *v, const ModuleThread *m, EventType _type, uint _revision, uint _procStep, double _time):
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

  struct LoggerVariableData* getVariableData(const Variable *v);

  //writing into a file
  void writeEventList(ostream& os, bool blockedEvents, uint max=0, bool clear=false);
  void dumpEventList();

  //methods called during write/read access from WITHIN the Variable
  void queryReadAccess(Variable *v, const ModuleThread *p);
  void queryWriteAccess(Variable *v, const ModuleThread *p);
  void logReadAccess(const Variable *v, const ModuleThread *p);
  void logReadDeAccess(const Variable *v, const ModuleThread *p);
  void logWriteAccess(const Variable *v, const ModuleThread *p);
  void logWriteDeAccess(const Variable *v, const ModuleThread *p);
  void logStepBegin(const ModuleThread *p);
  void logStepEnd(const ModuleThread *p);

  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};

#endif
