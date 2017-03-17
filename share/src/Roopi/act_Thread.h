#pragma once

#include "act.h"

struct Act_Thread : Act{
  ThreadL threads;
  Act_Thread(Roopi *r, Thread* th): Act(r), threads({th}){}
  Act_Thread(Roopi *r, const ThreadL& th): Act(r), threads(th){}
  ~Act_Thread(){ listDelete(threads); }

  template<class T> T* get(uint i=0){ return dynamic_cast<T*>(threads(0)); }

  typedef std::shared_ptr<Act_Thread> Ptr;
};

template<class T>
struct Act_Th : Act{
  shared_ptr<T> thread;
  Act_Th(Roopi *r, T* th): Act(r), thread(th){}
};

template<class T>
struct Act_Th2 : Act{
  T *thread;
  Act_Th2(Roopi *r, T* th): Act(r), thread(th){}
  ~Act_Th2(){
    Thread *th=dynamic_cast<Thread*>(thread);
    CHECK(th,"this is not a thread! I can't delete it!");
    thread=NULL;
    delete th;
  }
};
