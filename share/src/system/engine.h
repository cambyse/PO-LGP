#ifndef system_engine_h
#define system_engine_h

#include "module.h"

struct SystemDescription{
  struct VariableEntry{ MT::String name; TypeInfo* dcl; Variable *var; };
  struct ModuleEntry{ MT::String name; TypeInfo* dcl; MT::Array<VariableEntry*> accs; Module *mod; };

  typedef MT::Array<VariableEntry*> VariableEntryL;
  VariableEntryL variables;
  MT::Array<ModuleEntry> modules;

  SystemDescription() {}
  template<class T> void newVar(const char *name){
    VariableEntry *v = variables.append(new VariableEntry);
    v->dcl = new TypeInfo_typed<T, void>();
    v->name = name;
  }
  template<class T> T& getVar(uint i){ return *((T*)variables(i)->var->data); }

  void newModule(const char *name, const char *dclName, const VariableEntryL& vars);
  void report();
  void complete();
};


struct Engine{
  struct EventController *acc;
  enum { none=0, serial, threaded } mode;

  Engine();
  ~Engine();

  void create(SystemDescription& S);
  void step(Module &m);
  void step(SystemDescription& S);

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
//
// logging and blocking events
//

struct Event{
  const Variable *variable;
  const Module *module;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  Event(const Variable *v, const Module *m, EventType _type, uint _revision, uint _procStep, double _time):
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
  void queryReadAccess(Variable *v, const Module *p);
  void queryWriteAccess(Variable *v, const Module *p);
  void logReadAccess(const Variable *v, const Module *p);
  void logReadDeAccess(const Variable *v, const Module *p);
  void logWriteAccess(const Variable *v, const Module *p);
  void logWriteDeAccess(const Variable *v, const Module *p);
  void logStepBegin(const Module *p);
  void logStepEnd(const Module *p);

  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};

#endif
