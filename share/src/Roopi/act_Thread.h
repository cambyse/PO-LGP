#include "act.h"

template<class T> struct Act_Thread : Act{
  T* thread;
  Act_Thread(Roopi *r, bool loop=true)
    : Act(r){
    thread = new T;
    if(loop) thread->threadLoop();
    else thread->threadOpen();
  }
  ~Act_Thread(){
    thread->close();
    delete thread;
  }
};
