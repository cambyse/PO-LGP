#ifndef MT_biros_internal_h
#define MT_biros_internal_h

#include <pthread.h>
#include <biros/biros.h>

typedef unsigned int uint;


//===========================================================================
//
// simple wrappers of basic threading ingredients
//

void reportNice();
bool setNice(int);

//! a basic mutex lock
struct Mutex {
  pthread_mutex_t mutex;
  
  Mutex();
  ~Mutex();
  
  void lock();   ///< multiple threads may request 'lock for read'
  void unlock();                          ///< thread must unlock when they're done
};

//! a basic read/write access lock
struct Lock {
  int state; ///< -1==write locked, positive=numer of readers, 0=unlocked
  Mutex stateMutex;
  pthread_rwlock_t lock;
  
  Lock();
  ~Lock();
  
  void readLock();   ///< multiple threads may request 'lock for read'
  void writeLock();  ///< only one thread may request 'lock for write'
  void unlock();     ///< thread must unlock when they're done
};

//! a basic condition variable
struct ConditionVariable {
  int state;
  Mutex stateMutex;
  pthread_cond_t  cond;
  
  ConditionVariable();
  ~ConditionVariable();
  
  int  getState();  //WARNING: be aware that the returned state might be outdated already
  void setState(int i);
  void broadcast();
  //WARNING: by default the following routines will NOT unlock the mutex
  // This is to exclude that another process is changing the state again while it is processed.
  // YOU need to call waitUnlock(); after you've analyzed the new state
  void waitForSignal();
  void waitForSignal(double seconds);
  void waitForStateEq(int i);    ///< return value is the state after the waiting
  void waitForStateNotEq(int i); ///< return value is the state after the waiting
  void waitForStateGreaterThan(int i); ///< return value is the state after the waiting
  void waitUntil(double absTime);
  void waitUnlock();
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
  ConditionVariable cond; //to broadcast write access to this variable
  
  sVariable(Variable *_p) { p = _p; }
};

enum ThreadState { tsIDLE=0, tsCLOSE=-1, tsSTARTUP=-2, tsLOOPING=-3, tsBEATING=-4 }; //positive states indicate steps-to-go

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
    metronome = NULL;
  };
  
  static void *staticThreadMain(void *_self); ///< internal use: 'main' routine of the thread
};


#endif
