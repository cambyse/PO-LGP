#include "act_Event.h"

Act_Event::Act_Event(Roopi* r, const SignalerL& signalers, const EventFunction& eventFct)
  : Act(r), signalers(signalers), event(eventFct){
  CHECK(!signalers.containsDoubles(),"an event may not listen twice to the same signaler");
  std::function<void(Signaler*,int)> f = std::bind(&Act_Event::selfCallback, this, std::placeholders::_1, std::placeholders::_2);
  Act::callbacks.append(new Callback<void(Signaler*,int)>(this, f));
  signalersStates.resize(signalers.N);
  for(uint i=0;i<signalers.N;i++){
    signalersStates(i) = signalers(i)->getStatus();
    signalers(i)->callbacks.append(new Callback<void(Signaler*,int)>(this, std::bind(&Act_Event::callback, this, std::placeholders::_1, std::placeholders::_2)));
  }
  statusMutex.lock();
  setStatus(eventFct(signalers, signalersStates, -1));
  statusMutex.unlock();
}

Act_Event::~Act_Event(){
  Act::callbacks.memMove=true;
  Act::callbacks.delRemove(this);
  for(Signaler *s:signalers) s->callbacks.delRemove(this);
}

void Act_Event::callback(Signaler* s, int status){
  int i = signalers.findValue(s);
  CHECK(i>=0, "signaler " <<s <<" was not registered with this event!");
  signalersStates(i) = status;
  int newEventStatus = event(signalers, signalersStates, i);
//  cout <<"event callback: BOOL=" <<eventStatus <<' ' <<s <<' ' <<status <<" statuses=" <<statuses <<endl;
  statusMutex.lock();
//  if(eventStatus){
//    cout <<"event callback: STATUS TRUE" <<endl;
  if(this->status!=newEventStatus) setStatus(newEventStatus);
//  }else{
//    if(this->status!=AS_init && this->status!=AS_false) setStatus(AS_false);
//  }
  statusMutex.unlock();
}

void Act_Event::selfCallback(Signaler* s, int status){
  CHECK_EQ(s, dynamic_cast<Signaler*>(this), "");
  if(status==AS_kill) cout <<"so what??" <<endl;
}
