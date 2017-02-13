#include "act_Tweets.h"
#include "roopi.h"
#include <Core/thread.h>

#include <Control/taskControl.h>


struct sAct_Tweets : Thread{

  struct Callbacks : GraphEditCallback {
    sAct_Tweets *T;
    Callbacks(sAct_Tweets *T) : T(T){}

    virtual void cb_new(Node* n){
      if(n->isOfType<Act*>()) T->reg(n->get<Act*>());
//      if(n->isOfType<Thread*>()) T->reg(n->get<Thread*>());
    }
    virtual void cb_delete(Node* n){
      if(n->isOfType<Act*>()) T->dereg(n->get<Act*>());
//      if(n->isOfType<Thread*>()) T->dereg(n->get<Thread*>());
    }
  } callbacks;

  void reg(ConditionVariable* c){
    if(c!=this){
      stepMutex.lock();
      listenTo(c);
      stepMutex.unlock();
    }
  }

  void dereg(ConditionVariable* c){
    if(c!=this){
      stepMutex.lock();
      stopListenTo(c);
      stepMutex.unlock();
    }
  }

  sAct_Tweets(Act_Tweets *T) : Thread("Act_Tweets", -1.), callbacks(this){}
  virtual void open(){}
  virtual void close(){}
  virtual void step(){
    cout <<"TWEETs #" <<step_count <<' ';
    for(ConditionVariable *c:messengers){
      Act *a = dynamic_cast<Act*>(c);
      if(a){
        Act_CtrlTask *t = dynamic_cast<Act_CtrlTask*>(c);
        if(t && t->task){
          cout <<"Act_CtrlTask " <<t->get()->name <<" sends " <<mlr::Enum<ActStatus>((ActStatus)t->getStatus()) <<' ';
        }else{
          cout <<typeid(*a).name() <<" sends " <<mlr::Enum<ActStatus>((ActStatus)a->getStatus()) <<' ';
        }
      }else{
        Thread *th = dynamic_cast<Thread*>(c);
        if(th){
          cout <<"Thread " <<th->name <<" hat status " <<th->getStatus() <<' ';
        }else{
//        cout <<typeid(*c).name() <<" sends " <<c->getStatus() <<' ';
        }
      }
    }
    cout <<endl;
    messengers.clear();
  }
};

Act_Tweets::Act_Tweets(Roopi *r)
  : Act(r), s(new sAct_Tweets(this)){
  s->threadOpen();
  registry().callbacks.append(&s->callbacks);
}

Act_Tweets::~Act_Tweets(){
  registry().callbacks.removeValue(&s->callbacks);
  s->threadClose();
  delete s;
}

