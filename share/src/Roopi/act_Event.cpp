#include "act_Event.h"

Act_Event::Act_Event(Roopi* r, const ConditionVariableL& signalers, const EventBoolean& event)
  : Act(r), signalers(signalers), event(event){
  std::function<void(ConditionVariable*,int)> f = std::bind(&Act_Event::selfCallback, this, std::placeholders::_1, std::placeholders::_2);
  Act::callbacks.append(new Callback<void(ConditionVariable*,int)>(this, f));
  statuses.resize(signalers.N);
  for(uint i=0;i<signalers.N;i++){
    statuses(i) = signalers(i)->getStatus();
    signalers(i)->callbacks.append(new Callback<void(ConditionVariable*,int)>(this, std::bind(&Act_Event::callback, this, std::placeholders::_1, std::placeholders::_2)));
  }
  bool eventBoolean = event(signalers, statuses, -1);
  if(eventBoolean) setStatus(AS_true);
}

Act_Event::~Act_Event(){
  Act::callbacks.memMove=true;
  Act::callbacks.delRemove(this);
  for(ConditionVariable *s:signalers) s->callbacks.delRemove(this);
}

void Act_Event::callback(ConditionVariable* s, int status){
  int i = signalers.findValue(s);
  CHECK(i>=0, "signaler " <<s <<" was not registered with this event!");
  statuses(i) = status;
  bool eventBoolean = event(signalers, statuses, i);
//  cout <<"event callback: BOOL=" <<eventBoolean <<' ' <<s <<' ' <<status <<" statuses=" <<statuses <<endl;
  mutex.lock();
  if(eventBoolean){
//    cout <<"event callback: STATUS TRUE" <<endl;
    if(this->status!=AS_true) setStatus(AS_true);
  }else{
    if(this->status!=AS_init && this->status!=AS_false) setStatus(AS_false);
  }
  mutex.unlock();
}

void Act_Event::selfCallback(ConditionVariable* s, int status){
  CHECK_EQ(s, dynamic_cast<ConditionVariable*>(this), "");
  if(status==AS_kill) cout <<"so what??" <<endl;
}
