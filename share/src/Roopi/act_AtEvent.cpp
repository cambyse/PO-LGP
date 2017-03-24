#include "act_AtEvent.h"

Act_AtEvent::Act_AtEvent(Roopi* r, const Act::Ptr& _event, const Script& _script)
  : Act(r), Thread("Act_AtEvent", -1.), event(_event), script(_script){
  threadStep();
}

Act_AtEvent::~Act_AtEvent(){
  Act::setStatus(AS_kill);
  event->broadcast(); //triggers the for(;;)-loop below..
  threadClose();
}

void Act_AtEvent::step(){
  event->statusMutex.lock();
  for(;;){
    if(event->status==AS_true){
      event->statusMutex.unlock();
      int r=script();
      Act::setStatus(r);
      return;
    }
    if(event->status==AS_kill || Act::getStatus()==AS_kill){
      event->statusMutex.unlock();
      Act::setStatus(AS_kill);
      return;
    }
    event->waitForSignal(true);
  }
  HALT("should never be here");
  event->statusMutex.unlock();
}
