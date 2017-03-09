#pragma once

#include "act.h"

struct Act_Thread : Act{
  ThreadL threads;
  Act_Thread(Roopi *r, Thread* th): Act(r), threads({th}){}
  Act_Thread(Roopi *r, const ThreadL& th): Act(r), threads(th){}
  ~Act_Thread(){ listDelete(threads); }

  template<class T> T* get(uint i=0){ return dynamic_cast<T*>(threads(0)); }
};
