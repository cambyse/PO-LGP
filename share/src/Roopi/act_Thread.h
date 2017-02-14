#include "act.h"

struct Act_Thread : Act{
  Thread* th;
  Act_Thread(Roopi *r, Thread* th, bool loop=true);
  ~Act_Thread();

  template<class T> T* get(){ return dynamic_cast<T*>(th); }
};
