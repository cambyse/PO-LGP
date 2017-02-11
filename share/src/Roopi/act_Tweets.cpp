#include "act_Tweets.h"
#include "roopi.h"
#include <Core/thread.h>

struct sAct_Tweets : Thread{
  ACCESSlisten(ActL, acts)

  sAct_Tweets() : Thread("Act_Tweets", -1.){}
  virtual void open(){}
  virtual void close(){}
  virtual void step(){
    cout <<"TWEET " <<step_count <<" messengers: ";
    for(ConditionVariable *c:status.messengers) cout <<typeid(*c).name() <<"sends " <<c->getValue();
    cout <<endl;
    status.messengers.clear();
  }
};

Act_Tweets::Act_Tweets(Roopi *r)
  : Act(r), s(new sAct_Tweets()){
  s->threadOpen();
}

Act_Tweets::~Act_Tweets(){
  s->threadClose();
  delete s;
}

void Act_Tweets::registerAct(Act* a){
  if(a!=this) s->status.listenTo(a);
}

void Act_Tweets::deregisterAct(Act* a){
  if(a!=this) s->status.stopListenTo(a);
}
