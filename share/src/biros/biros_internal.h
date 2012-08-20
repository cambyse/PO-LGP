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
  Lock rwlock;
  ConditionVariable cond; //to broadcast write access to this variable
  struct LoggerVariableData *loggerData; //data that the logger may associate with a variable
  
  sVariable():loggerData(NULL){}
};

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go

//Process' internal data
struct sProcess {
  pthread_t thread;                    ///< pthread pointer
  pid_t tid;                           ///< system thread id
  ConditionVariable threadCondition;   ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  CycleTimer timer;                    ///< measures cycle and busy times
  Metronome *metronome;                ///< used for beat-looping
  uint skips;                          ///< how often a step was requested but (because busy) skipped
  int threadPriority;                  ///< priority of this thread
  
  sProcess(): thread(0), tid(0), threadCondition(tsCLOSE), metronome(NULL), skips(0), threadPriority(0) {}
  
  static void *staticThreadMain(void *_self); ///< internal use: 'main' routine of the thread
};


#endif
