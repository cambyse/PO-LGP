#include "act_AtEvent.h"

Act_AtEvent::Act_AtEvent(Roopi* r, shared_ptr<Act_Event>& E, const Script& S)
  : Act(r), Thread("Act_AtEvent", -1.), event(E), script(S){
  threadStep();
}

Act_AtEvent::~Act_AtEvent(){
  Act::setStatus(AS_kill);
  event->broadcast(); //triggers the for(;;)-loop below..
  threadClose();
}

void Act_AtEvent::step(){
  event->mutex.lock();
  for(;;){
    if(event->status==AS_true){
      event->mutex.unlock();
      int r=script();
      Act::setStatus(r);
      return;
    }
    if(event->status==AS_kill || Act::getStatus()==AS_kill){
      event->mutex.unlock();
      Act::setStatus(AS_kill);
      return;
    }
    event->waitForSignal(true);
  }
  HALT("should never be here");
  event->mutex.unlock();
}
