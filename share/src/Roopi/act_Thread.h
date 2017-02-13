#include "act.h"

template<class T> struct Act_Thread : Act{
  T* thread;
  Act_Thread(Roopi *r, T* th, bool loop=true)
    : Act(r), thread(th){
    if(loop) thread->threadLoop();
    else thread->threadOpen();
  }
  ~Act_Thread(){
    thread->threadClose();
    delete thread;
  }
};
