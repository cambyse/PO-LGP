#ifndef MT_biros_internal_h
#define MT_biros_internal_h

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
  ConditionVariable cond; //to broadcast write access to this variable
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable

  //MT bool logValues;
  //MT bool dbDrivenReplay;
  //MT pthread_mutex_t replay_mutex; //TODO: move to sVariable! (that's the point of private..)
  virtual void serializeToString(MT::String &string) const;
  virtual void deSerializeFromString(const MT::String &string);
  
  sVariable():loggerData(NULL){}
};

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go

//Process' internal data
struct sProcess {
  pthread_t thread;                    ///< pthread pointer
  pid_t tid;                           ///< system thread id
  VariableL listensTo;
  ParameterL dependsOn;
  CycleTimer timer;                    ///< measures cycle and busy times
  Metronome *metronome;                ///< used for beat-looping
  uint skips;                          ///< how often a step was requested but (because busy) skipped
  int threadPriority;                  ///< priority of this thread
  
  sProcess(): thread(0), tid(0), metronome(NULL), skips(0), threadPriority(0) {}
  
  static void *staticThreadMain(void *_self); ///< internal use: 'main' routine of the thread
};

//===========================================================================
//
// information about a single access event
//

struct AccessEvent{
  const Variable *var;
  const Process *proc;
  enum AccessType{ read, write } type;
  uint revision;
  uint procStep;
  AccessEvent(const Variable *v, const Process *p, AccessType _type, uint _revision, uint _procStep):
    var(v), proc(p), type(_type), revision(_revision), procStep(_procStep){}
};

typedef MT::Array<AccessEvent*> AccessEventL;

struct sAccessController{
  bool enableAccessLog;
  bool enableDataLog;
  bool replay;
  RWLock eventsLock;

  AccessEventL blockedAccesses;
  AccessEventL events;
  ConditionVariable blockMode;
  ofstream* eventsFile;

  sAccessController();
  ~sAccessController();

  struct LoggerVariableData* getVariableData(const Variable *v);

  //writing into a file
  void dumpEventList();

  //methods called during write/read access from WITHIN biros
  void queryReadAccess(Variable *v, const Process *p);
  void queryWriteAccess(Variable *v, const Process *p);
  void logReadAccess(const Variable *v, const Process *p);
  void logReadDeAccess(const Variable *v, const Process *p);
  void logWriteAccess(const Variable *v, const Process *p);
  void logWriteDeAccess(const Variable *v, const Process *p);
  
  MT::Array<ConditionVariable*> breakpointQueue;
  Mutex breakpointMutex;
  void breakpointSleep(); //the caller goes to sleep
  void breakpointNext(); //first in the queue is being woke up
};


#endif
