#ifndef MT_threads_h
#define MT_threads_h

#include <pthread.h>
#include <iostream>

typedef unsigned int uint;


//===========================================================================
//
// simple wrappers of basic threading ingredients
//

void reportNice();
bool setNice(int);

//! a basic read/write access lock
struct RWLock{
  int state;
  const char* msg;
  pthread_rwlock_t lock;
    
  RWLock();
  ~RWLock();

  void readLock(const char* _msg=NULL);   ///< multiple threads may request 'lock for read'
  void writeLock(const char* _msg=NULL);  ///< only one thread may request 'lock for write' 
  void unlock();                          ///< thread must unlock when they're done
};

//! a basic condition variable
struct ConditionVariable{
  int state;
  pthread_mutex_t mutex;
  pthread_cond_t  cond;

  ConditionVariable();
  ~ConditionVariable();

  int  getState();
  void setState(int i);
  void signal();
  void waitForSignal();
  void waitForStateEq(int i);
  void waitForStateNotEq(int i);
  void waitUntil(double absTime);
};

//! a simple struct to realized a strict tic tac timing (call step() once in a loop)
struct Metronome{
  long targetDt;
  timespec ticTime, lastTime;
  uint tics;
  const char* name;                   ///< name

  Metronome(const char* name, long _targetDt); ///< set tic tac time in milli seconds
  ~Metronome();

  void reset();
  void waitForTic();              ///< waits until the next tic
  double getTimeSinceTic();       ///< time since last tic
};

//! a really simple thing to meassure cycle and busy times
struct CycleTimer{
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
  
/** \brief a threading template: classes should derived from this can easily be threaded.
   Every thread needs to implement an open, step and close method -- nothing more. The thread
   can be set to loop endlessly and with maximum speed, or be stepped (strict synchronization with
   the calling process). Threads should have public variables for input/output/parameters; the step() method
   computes the output from the input; when step is done (and the thread idle) the main process
   may readout the results and set new inputs. When the thread is permanently looping you should
   use the lock to coordinates read/write access to all public variables. */
struct StepThread{
  pthread_t thread;                    ///< pthread pointer
  pid_t tid;                           ///< system thread id
  ConditionVariable threadCondition;   ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  CycleTimer timer;                    ///< measures cycle and busy times
  Metronome *metronome;
  uint skips;                          ///< how often a step was requested but (because busy) skipped
  int threadPriority;                  ///< priority of this thread
  const char* threadName;              ///< name of the thread
  //RWLock lock;                           ///< default mutex lock - to coordinate access
  bool broadCastDone;
  ConditionVariable *syncCondition;

  enum ThreadState { tsOPEN=-1, tsCLOSE=-2, tsLOOPING=-3, tsBEATING=-4, tsSYNCLOOPING=-5, tsIDLE=0 }; //positive states indicate 2*steps-to-go

  virtual void open() = 0;             ///< is called within the thread when the thread is created
  virtual void close() = 0;            ///< is called within the thread when the thread is destroyed
  virtual void step() = 0;             ///< is called within the thread when trigerring a step from outside (or when permanently looping)

  StepThread(const char* name);
  ~StepThread();
  void threadOpen(int priority=0);      ///< start the thread (in idle mode) (should be positive for changes)
  void threadClose();                   ///< close the thread (stops looping and waits for idle mode before joining the thread)
  
  void threadStepOrSkip(uint maxSkips); ///< trigger a step (idle -> working mode) or skip if still busy (counts skips.., maxSkip=0 -> no warnings)
  void threadStep(bool wait=false);     ///< trigger a step (idle -> working mode)
  void threadSteps(uint steps);         ///< trigger multiple steps (idle -> working mode)
  
  void threadWait();                    ///< wait until step is done (working -> idle mode)
  bool threadIsReady();                 ///< check if in idle mode
  void threadLoop();                    ///< loop stepping forever
  void threadLoopWithBeat(double sec);
  void threadLoopSyncWithDone(StepThread& thread);
  void threadLoopStop();                ///< stop looping
  static void *staticThreadMain(void *_this); ///< internal use: 'main' routine of the thread
};


#ifdef  MT_IMPLEMENTATION
#  include "threads.cpp"
#endif

#endif
