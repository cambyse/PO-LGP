#include "act_Tweets.h"
#include "roopi.h"
#include <Core/thread.h>

#include <Control/taskControl.h>


struct sAct_Tweets : GraphEditCallback {

  virtual void cb_new(Node* n){
    if(n->isOfType<Act*>()) reg(n->get<Act*>());
    //      if(n->isOfType<Thread*>()) T->reg(n->get<Thread*>());
  }
  virtual void cb_delete(Node* n){
    if(n->isOfType<Act*>()) dereg(n->get<Act*>());
    //      if(n->isOfType<Thread*>()) T->dereg(n->get<Thread*>());
  }

  void reg(ConditionVariable* c){
//    if(c!=this){
//      stepMutex.lock();
#if 1
      c->callbacks.append(Callback<void(ConditionVariable*,int)>(c,
        [this](ConditionVariable* c,int s){
          this->tweet(c,s);
        } )
      );
#else
      listenTo(c);
#endif
//      stepMutex.unlock();
//    }
  }

  void dereg(ConditionVariable* c){
//    if(c!=this){
//      stepMutex.lock();
#if 1
      c->callbacks.memMove=true;
      c->callbacks.removeValue(Callback<void(ConditionVariable*,int)>(c));
#else
      if(listensTo.contains(c)) stopListenTo(c);
#endif
//      stepMutex.unlock();
//    }
  }

  void tweet(ConditionVariable* c, int s){
      cout <<"TWEETs #" <<' ' <<std::setprecision(3) <<mlr::realTime() <<' ';
      Act *a = dynamic_cast<Act*>(c);
      if(a){
        Act_CtrlTask *t = dynamic_cast<Act_CtrlTask*>(c);
        if(t && t->task){
          cout <<"Act_CtrlTask " <<t->task->name <<" sends " <<mlr::Enum<ActStatus>((ActStatus)s) <<' ';
        }else{
          cout <<typeid(*a).name() <<" sends " <<mlr::Enum<ActStatus>((ActStatus)s) <<' ';
        }
      }else{
        Thread *th = dynamic_cast<Thread*>(c);
        if(th){
          cout <<"Thread " <<th->name <<" hat status " <<s <<' ';
        }else{
          //        cout <<typeid(*c).name() <<" sends " <<c->getStatus() <<' ';
        }
      }
      cout <<endl;
  }

  sAct_Tweets(Act_Tweets *T){}
};

Act_Tweets::Act_Tweets(Roopi *r)
  : Act(r), s(new sAct_Tweets(this)){
  registry().callbacks.append(s);
}

Act_Tweets::~Act_Tweets(){
  registry().callbacks.removeValue(s);
  delete s;
}
