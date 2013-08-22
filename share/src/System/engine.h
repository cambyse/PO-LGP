#ifndef system_engine_h
#define system_engine_h

#include <Core/thread.h>
#include "module.h"

struct Variable;
struct ModuleThread;
typedef MT::Array<Module*> ModuleL;
typedef MT::Array<ModuleThread*> ModuleThreadL;
typedef MT::Array<Variable*> VariableL;
typedef MT::Array<Access*> AccessL;


struct ModuleThread:Thread{
  enum StepMode { listenFirst=0, listenAll, loopWithBeat, loopFull };
  Module *m;
  uint step_count;
  StepMode mode; double beat;

  /// @name c'tor/d'tor
  ModuleThread(Module* _m, const char* _name=NULL):Thread(_name),m(_m),step_count(0){ m->name = _name; }

  virtual void open(){ m->open(); }
  virtual void step(){ m->step(); step_count++; }
  virtual void close(){ m->close(); }
};
//inline void operator>>(istream& is, ModuleThread& m){  }
inline void operator<<(ostream& os, const ModuleThread& m){ os <<"ModuleThread " <<m.name <<' ' <<m.step_count; }

//===========================================================================
/**
 * A Variable is a container to hold data, potentially handling concurrent r/w
 * access, which is used to exchange information between processes.
 */
struct Variable : DataAccess {
  struct sVariable *s;        ///< private
  ModuleThreadL listeners;
  ConditionVariable revision; ///< revision (= number of write accesses) number
  RWLock rwlock;              ///< rwLock (usually handled via read/writeAccess -- but views may access directly...)
  Item *reg;

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
  void waitForNextWriteAccess();
  int  waitForRevisionGreaterThan(int rev); //returns the revision

  /// @name info
  struct FieldRegistration& get_field(uint i) const;
};
inline void operator<<(ostream& os, const Variable& v){ os <<"Variable " <<v.name <<' ' <<*v.type; }


//TODO: hide?
struct sVariable {
  virtual ~sVariable(){}
  MT::Array<struct FieldRegistration*> fields; //? make static? not recreating for each variable?
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);

  sVariable():loggerData(NULL){}
};


//===========================================================================
/**
 * A System is an interconnected list of Modules and Variables.
 */

struct System{
  ModuleThreadL mts;
  VariableL vars;
//  KeyValueGraph system;

//  SystemDescription() {}

  Variable* addVariable(Access *a){
    Variable *v = new Variable(a->name);
    v->type = a->type->clone();
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
//  Item* getVariableEntry(const Access& acc);
//  Item* getVariableEntry(const char* name, const Type& typeinfo);

//  Item* getVar(uint i){ ItemL vars = system.getTypedItems<VariableEntry>("Variable"); return vars(i); }
  template<class T> T& getVar(uint i){ return *((T*)vars(i)->data); }
  template<class T> Access_typed<T>* getAccess(const char* varName){
    Variable *v = listFindByName(vars, varName);
    return new Access_typed<T>(varName, NULL, v);
  }

  template<class T> ModuleThread* addModule(const char *name=NULL, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  ModuleThread* addModule(const char *dclName, const char *name=NULL, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  void addModule(const char *dclName, const char *name, const uintA& accIdxs, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  void addModule(const char *dclName, const char *name, const StringA& accNames, ModuleThread::StepMode mode=ModuleThread::listenFirst, double beat=0.);
  KeyValueGraph graph() const;
  void write(ostream& os) const;
  void complete();
};
stdOutPipe(System);

//===========================================================================
/**
 * An Engine runs a system.
 */

struct Engine{
  struct EventController *acc;
  enum { none=0, serial, threaded } mode;
  KeyValueGraph *system;
  AccessL createdAccesses;

  Engine();
  virtual ~Engine();

  void open(System& S);
  void step(ModuleThread &m, bool threadedOnly=false);
  void step(System& S);
  void test(System& S);
  void close(System& S);

  /// @name event control
  void enableAccessLog();
  void dumpAccessLog();
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
};

Engine& engine();

//inline void operator<<(std::ostream& os,const SystemDescription::ModuleEntry& m){ os <<"ModuleEntry"; }
//inline void operator<<(std::ostream& os,const SystemDescription::AccessEntry& a){ os <<"AccessEntry '" <<"'"; }
//inline void operator<<(std::ostream& os,const SystemDescription::VariableEntry& v){ os <<"VariableEntry"; }


//===========================================================================
//
// logging and blocking events
//

struct Event{
  const Variable *variable;
  const ModuleThread *module;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  Event(const Variable *v, const ModuleThread *m, EventType _type, uint _revision, uint _procStep, double _time):
    variable(v), module(m), type(_type), revision(_revision), procStep(_procStep), time(_time){}
};

typedef MT::Array<Event*> BirosEventL;

struct EventController{
  bool enableEventLog;
  bool enableDataLog;
  bool enableReplay;

  BirosEventL events;
  RWLock eventsLock;
  ConditionVariable blockMode; //0=all_run, 1=next_runs, 2=none_runs
  BirosEventL blockedEvents;

  ofstream* eventsFile;

  EventController();
  ~EventController();

  struct LoggerVariableData* getVariableData(const Variable *v);

  //writing into a file
  void writeEventList(ostream& os, bool blockedEvents, uint max=0, bool clear=false);
  void dumpEventList();

  //methods called during write/read access from WITHIN biros
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


template<class T> ModuleThread* System::addModule(const char *name, ModuleThread::StepMode mode, double beat){
  Module *m = new T;
  for_list_(Access, a, m->accesses) a->module = m;
  ModuleThread *mt = new ModuleThread(m, name);
  mt->mode = mode;
  mt->beat = beat;
  mts.append(mt);
  return mt;
}

#endif
