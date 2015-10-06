#ifndef system_engine_h
#define system_engine_h

#include <Core/module.h>

struct SystemDescription{
  enum StepMode { listenAll=0, listenRead, loopWithBeat, loopFull };
  struct VariableEntry{ Type* type; Variable *var; };
  struct AccessEntry{ Node* reg; Type* type; Access *acc; };
  struct ModuleEntry{ Node* reg; Type* type; Module *mod; StepMode mode; double beat; };
  Graph system;

  SystemDescription() {}

  template<class T> void addVar(const char *name){
    VariableEntry *v = new VariableEntry;
    v->type = new Type_typed<T, void>();
    system.append<VariableEntry>({"Variable", name}, v);
  }
  Node* getVariableEntry(const Access& acc);
  Node* getVariableEntry(const char* name, const Type& typeinfo);

  Node* getVar(uint i){ NodeL vars = system.getTypedNodes<VariableEntry>("Variable"); return vars(i); }
  template<class T> T& getValue(uint i){ return *((T*)getVar(i)->value<VariableEntry>()->var->data); }

  void addModule(const char *dclName, const char *name=NULL, const NodeL& vars=NoNodeL, StepMode mode=listenRead, double beat=0.);
  void report();
  void complete();
};


struct Engine{
  struct EventController *acc;
  enum { none=0, serial, threaded } mode;
  Graph *system;

  Engine();
  ~Engine();

  void create(SystemDescription& S);
  void step(Module &m);
  void step(SystemDescription& S);
  void test(SystemDescription& S);

  /// @name event control
  void enableAccessLog();
  void dumpAccessLog();
  void blockAllAccesses();
  void unblockAllAccesses();
  void stepToNextAccess();
  void stepToNextWriteAccess();
};

Engine& engine();

inline void operator<<(std::ostream& os,const SystemDescription::ModuleEntry& m){ os <<"ModuleEntry"; }
inline void operator<<(std::ostream& os,const SystemDescription::AccessEntry& a){ os <<"AccessEntry '" <<"'"; }
inline void operator<<(std::ostream& os,const SystemDescription::VariableEntry& v){ os <<"VariableEntry"; }


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

typedef mlr::Array<Event*> BirosEventL;

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

  mlr::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};

#endif
