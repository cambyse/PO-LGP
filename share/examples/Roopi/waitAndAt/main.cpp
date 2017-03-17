#include <Roopi/roopi.h>

//===============================================================================

int Script_setRnd_a(){
  Access_typed<int> a("a");
  Access_typed<int> b("b");
  b.set() = 0;
  for(;;){
    a.set() = rnd(10);
    b.set() += a.get();
    LOG(0) <<"SET: a=" <<a.get() <<" b=" <<b.get();
    mlr::wait(1.);
    if(b.get()>30.) break;
  }
  return 1;
}

int Script_SayHello(){
  LOG(0) <<"HELLO";
  return 1;
}

int Event_b_ge_10(){
  Access_typed<int> b("b");
  for(;;){
    if(b.get()>=10) return 1;
    b.waitForNextRevision();
  }
  return 2;
}

int Event_never(){
  mlr::wait();
  return 2;
}

bool Event2_b_ge_10(const ConditionVariableL& signalers, const intA& statuses, int whoChanged){
  AccessData<int> *b = dynamic_cast<AccessData<int>*>(signalers(0));
  CHECK(b,"");
  if(whoChanged==-1 && b->get()>=10) return true;
  if(whoChanged==0 && b->lockedGet()>=10) return true;
  return false;
}

//===============================================================================

void timedWait(){
  Roopi R;

  auto rndSetter = R.runScript(Script_setRnd_a);

  bool r = R.wait({&rndSetter}, 3.);
  LOG(0) <<"timed wait returned " <<r;

  r = R.wait({&rndSetter});
  LOG(0) <<"wait returned " <<r;
}

//===============================================================================

void atDemo(){
  Roopi R;
  R.startTweets();

  auto rndSetter = R.runScript(Script_setRnd_a);

  Access_typed<int> b("b");
//  Act_Event event(&R, {b.data}, Event2_b_ge_10);

//  event.waitForStatusEq(AS_true);
//  cout <<"*******************" <<endl;

//  {
    auto hello = R.atEvent({b.data}, Event2_b_ge_10, Script_SayHello);
//    mlr::wait(1.);
    //now kill 'hello' without it ever triggering...
//  }

  R.wait({&rndSetter});
}

//===============================================================================

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  timedWait();
  atDemo();

  return 0;
}
