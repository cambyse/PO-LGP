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
  int state; ///< 0=unlocked, 1=locked
  
  Mutex();
  ~Mutex();
  
  void lock();
  void unlock();
};

//! a basic read/write access lock
struct Lock {
  pthread_rwlock_t lock;
  int state; ///< -1==write locked, positive=numer of readers, 0=unlocked
  Mutex stateMutex;
  
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
  
  ConditionVariable(int initialState=0);
  ~ConditionVariable();
  
  void setState(int i, bool signalOnlyFirstInQueue=false); ///< sets state and broadcasts
  void broadcast(bool signalOnlyFirstInQueue=false);       ///< just broadcast
  
  void lock();   //the user can manually lock/unlock, if he needs atomic state access for longer -> use userHasLocked=true below!
  void unlock();
  
  int  getState(bool userHasLocked=false);
  void waitForSignal(bool userHasLocked=false);
  void waitForSignal(double seconds, bool userHasLocked=false);
  void waitForStateEq(int i, bool userHasLocked=false);    ///< return value is the state after the waiting
  void waitForStateNotEq(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForStateGreaterThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitForStateSmallerThan(int i, bool userHasLocked=false); ///< return value is the state after the waiting
  void waitUntil(double absTime, bool userHasLocked=false);
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
