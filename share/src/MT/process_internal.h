#ifndef MT_process_internal_h
#define MT_process_internal_h

#include <pthread.h>

typedef unsigned int uint;


//===========================================================================
//
// simple wrappers of basic threading ingredients
//

void reportNice();
bool setNice(int);

//! a basic read/write access lock
struct Lock {
  int state;
  const char* msg;
  pthread_rwlock_t lock;
  
  Lock();
  ~Lock();
  
  void readLock(const char* _msg=NULL);   ///< multiple threads may request 'lock for read'
  void writeLock(const char* _msg=NULL);  ///< only one thread may request 'lock for write'
  void unlock();                          ///< thread must unlock when they're done
};

//! a basic mutex lock
struct Mutex {
  int state;
  const char* msg;
  pthread_mutex_t _lock;
  
  Mutex();
  ~Mutex();
  
  void lock(const char* _msg=NULL);   ///< multiple threads may request 'lock for read'
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
  void setState(int i);
  void signal();
  void waitForSignal();
  void waitForStateEq(int i);
  void waitForStateNotEq(int i);
  void waitUntil(double absTime);
};

//Variable's internal data
struct sVariable {
  Variable *p;
  //ofstream os;
  Lock lock;
  ConditionVariable cond;
  
  sVariable(Variable *_p){ p = _p; }
};

enum ThreadState { tsOPEN=-1, tsCLOSE=-2, tsLOOPING=-3, tsBEATING=-4, tsSYNCLOOPING=-5, tsIDLE=0 }; //positive states indicate steps-to-go

//Process' internal data
struct sProcess {
  pthread_t thread;                    ///< pthread pointer
  pid_t tid;                           ///< system thread id
  ConditionVariable threadCondition;   ///< the condition variable indicates the state of the thread: positive=steps-to-go, otherwise it is a ThreadState
  CycleTimer timer;                    ///< measures cycle and busy times
  Metronome *metronome;                ///< used for beat-looping
  uint skips;                          ///< how often a step was requested but (because busy) skipped
  int threadPriority;                  ///< priority of this thread
  
  bool broadcastDone;
  ConditionVariable *syncCondition;
  
  sProcess(){
    skips=0;
    threadCondition.setState(tsCLOSE);
    tid=0;
    threadPriority=0;
    thread=NULL;
    broadcastDone=false;
    syncCondition=NULL;
  };
  
  static void *staticThreadMain(void *_self); ///< internal use: 'main' routine of the thread
};




#endif
