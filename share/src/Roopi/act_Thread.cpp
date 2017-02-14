#include "act_Thread.h"

Act_Thread::Act_Thread(Roopi* r, Thread* th, bool loop)
  : Act(r), th(th){
  if(loop) th->threadLoop();
  else th->threadOpen();
}


Act_Thread::~Act_Thread(){
//  th->threadClose();
  delete th;
}
