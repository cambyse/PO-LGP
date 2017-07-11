#pragma once

#include "act.h"

#if 0
struct Act_Thread : Act{
  ThreadL threads;
  Act_Thread(Roopi *r, Thread* th): Act(r), threads({th}){}
  Act_Thread(Roopi *r, const ThreadL& th): Act(r), threads(th){}
  ~Act_Thread(){ listDelete(threads); }

  template<class T> T* get(uint i=0){ return dynamic_cast<T*>(threads(0)); }

  typedef std::shared_ptr<Act_Thread> Ptr;
};
#endif

struct Act_Thread : Act{
  Thread *thread;
  Act_Thread(Roopi *r, Thread* th): Act(r), thread(th){}
  ~Act_Thread(){ delete thread; }

  template<class T> T* get(){ T* th = dynamic_cast<T*>(thread); CHECK(th,""); return th; }

  void write(ostream& os){ Act::write(os); os <<thread->name <<"' " <<thread->timer.report();  }

  typedef std::shared_ptr<Act_Thread> Ptr;
};

template<class T>
struct Act_Th : Act{
  shared_ptr<T> thread;
  Act_Th(Roopi *r, T* th): Act(r), thread(th){}
};
