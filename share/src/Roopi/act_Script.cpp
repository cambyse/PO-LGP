#include "act_Script.h"

struct sAct_Script : Thread{
  Act *act;
  Script script;
  sAct_Script(Act *a, const Script& S)
    : Thread("Act_Script", -1.), act(a), script(S){
    threadStep();
  }
  ~sAct_Script(){
//    bool closed = Thread::waitForStatusEq(tsCLOSE, false, .1);
//    if(!closed){
//      LOG(-1) <<"AtScript act is being killed (destroyed from outside without finishing)";
//      threadCancel();
//    }
    threadClose();
  }
  virtual void open(){}
  virtual void step(){
    int r = script();
    act->setStatus(r);
  }
  virtual void close(){}
};

Act_Script::Act_Script(Roopi* r, const Script& S)
  : Act(r), s(new sAct_Script(this, S)) {
}

Act_Script::~Act_Script(){
  delete s;
}
