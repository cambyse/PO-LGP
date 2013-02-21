#ifndef MT_biros_internal_h
#define MT_biros_internal_h
/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */

#include <pthread.h>
#include <biros/biros.h>

typedef unsigned int uint;

void reportNice();
bool setNice(int);



//===========================================================================
//
// privates
//

//Variable's internal data
struct sVariable {
  MT::Array<struct FieldRegistration*> fields; //? make static? not recreating for each variable?
  ProcessL listeners;
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);

  sVariable():loggerData(NULL){}
};

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsOPENING=-2, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go

//Process' internal data
struct sProcess: Thread{
  Process *proc;
  pid_t tid;                           ///< system thread id
  VariableL listensTo;
  ParameterL dependsOn;
  Metronome *metronome;                ///< used for beat-looping

  sProcess(Process *p): proc(p), tid(0), metronome(NULL) {}

  void main(); //virtual method for Thread
};


//===========================================================================
//
// logging and blocking events
//

struct BirosEvent{
  const Variable *var;
  const Process *proc;
  enum EventType{ read, write, stepBegin, stepEnd } type;
  uint revision;
  uint procStep;
  double time;
  BirosEvent(const Variable *v, const Process *p, EventType _type, uint _revision, uint _procStep, double _time):
    var(v), proc(p), type(_type), revision(_revision), procStep(_procStep), time(_time){}
};

typedef MT::Array<BirosEvent*> BirosEventL;

struct sBirosEventController{
  bool enableEventLog;
  bool enableDataLog;
  bool enableReplay;

  BirosEventL events;
  RWLock eventsLock;
  ConditionVariable blockMode; //0=all_run, 1=next_runs, 2=none_runs
  BirosEventL blockedEvents;

  ofstream* eventsFile;

  sBirosEventController();
  ~sBirosEventController();

  struct LoggerVariableData* getVariableData(const Variable *v);

  //writing into a file
  void writeEventList(ostream& os, bool blockedEvents, uint max=0, bool clear=false);
  void dumpEventList();

  //methods called during write/read access from WITHIN biros
  void queryReadAccess(Variable *v, const Process *p);
  void queryWriteAccess(Variable *v, const Process *p);
  void logReadAccess(const Variable *v, const Process *p);
  void logReadDeAccess(const Variable *v, const Process *p);
  void logWriteAccess(const Variable *v, const Process *p);
  void logWriteDeAccess(const Variable *v, const Process *p);
  void logStepBegin(const Process *p);
  void logStepEnd(const Process *p);

  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};


#endif
/** @} */
