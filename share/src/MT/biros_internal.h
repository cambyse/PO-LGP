#ifndef MT_process_internal_h
#define MT_process_internal_h

#include <pthread.h>
#include "biros.h"

typedef unsigned int uint;


//===========================================================================
//
// simple wrappers of basic threading ingredients
//

void reportNice();
bool setNice(int);

//! a basic mutex lock
struct Mutex {
  const char* msg;
  pthread_mutex_t _lock;
  
  Mutex();
  ~Mutex();
  
  void lock(const char* _msg=NULL);   ///< multiple threads may request 'lock for read'
  void unlock();                          ///< thread must unlock when they're done
};

//! a basic read/write access lock
struct Lock {
  int state; ///< -1==write locked, positive=numer of readers, 0=unlocked
  const char* msg;
  Mutex stateMutex;
  pthread_rwlock_t lock;
  
  Lock();
  ~Lock();
  
  void readLock(const char* _msg=NULL);   ///< multiple threads may request 'lock for read'
  void writeLock(const char* _msg=NULL);  ///< only one thread may request 'lock for write'
  void unlock();                          ///< thread must unlock when they're done
};

//! a basic condition variable
struct ConditionVariable {
  int state;
  pthread_mutex_t mutex;
  pthread_cond_t  cond;
  
  ConditionVariable();
  ~ConditionVariable();
  
  int  getState();
  int  setState(int i);
  void signal();
  void waitForSignal();
  void waitForSignal(double seconds);
  int  waitForStateEq(int i);    ///< return value is the state after the waiting
  int  waitForStateNotEq(int i); ///< return value is the state after the waiting
  void waitUntil(double absTime);
};

//===========================================================================
//
// Timing helpers
//

//! a simple struct to realize a strict tic tac timing (call step() once in a loop)
struct Metronome {
  long targetDt;
  timespec ticTime, lastTime;
  uint tics;
  const char* name;                   ///< name
  
  Metronome(const char* name, long _targetDt); //!< set tic tac time in milli seconds
  ~Metronome();
  
  void reset(long _targetDt);
  void waitForTic();              //!< waits until the next tic
  double getTimeSinceTic();       //!< time since last tic
};

//! a really simple thing to meassure cycle and busy times
struct CycleTimer {
  uint steps;
  double cyclDt, cyclDtMean, cyclDtMax;  ///< internal variables to measure step time
  double busyDt, busyDtMean, busyDtMax;  ///< internal variables to measure step time
  timespec now, lastTime;
  const char* name;                    ///< name
  CycleTimer(const char *_name=NULL);
  ~CycleTimer();
  void reset();
  void cycleStart();
  void cycleDone();
};


//===========================================================================
//
// privates
//

//Variable's internal data
struct sVariable {
  Variable *p;
  Lock lock;
  
  sVariable(Variable *_p) { p = _p; }
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
  
  sProcess() {
    skips=0;
    threadCondition.setState(tsCLOSE);
    tid=0;
    threadPriority=0;
    thread=0;
    metronome=NULL;
  };
  
  static void *staticThreadMain(void *_self); ///< internal use: 'main' routine of the thread
};


#endif
