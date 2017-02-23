#include "act_Thread.h"

Act_Thread::Act_Thread(Roopi* r, Thread* th)
  : Act(r), th(th){
//  if(loop) th->threadLoop(); //the derived Thread class must do this itself!
//  else th->threadOpen();
}


Act_Thread::~Act_Thread(){
//  th->threadClose(); //the derived Thread class must do this itself!
  delete th;
}
