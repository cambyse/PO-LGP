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


#endif
