#include <Roopi/roopi.h>

//===============================================================================

int Script_setRnd_a(){
  Access<int> a("a");
  Access<int> b("b");
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
  Access<int> b("b");
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

int Event2_b_ge_10(const SignalerL& signalers, const intA& statuses, int whoChanged){
  VariableData<int> *b = dynamic_cast<VariableData<int>*>(signalers(0));
  CHECK(b,"");
  if(whoChanged==-1 && b->get()>=10)
    return AS_true;
  if(whoChanged==0 && b->lockedGet()>=10)
    return AS_true;
  return AS_false;
}

//===============================================================================

void timedWait(){
  Roopi R;

  auto rndSetter = R.run(Script_setRnd_a);

  bool r = R.wait(+rndSetter, 3.);
  LOG(0) <<"timed wait returned " <<r;

  r = R.wait(+rndSetter);
  LOG(0) <<"wait returned " <<r;
}

//===============================================================================

void atDemo(){
  Roopi R;
  R.startTweets();

  auto rndSetter = R.run(Script_setRnd_a);

  Access<int> b("b");
//  Act_Event event(&R, {b.data}, Event2_b_ge_10);

//  event.waitForStatusEq(AS_true);
//  cout <<"*******************" <<endl;

//  {
//  auto ev = R.event(+b.data, Event2_b_ge_10);
    auto hello = R.at(R.event(+b.data, Event2_b_ge_10), Script_SayHello);
//    mlr::wait(1.);
    //now kill 'hello' without it ever triggering...
//  }

  R.wait(+rndSetter);
}

//===============================================================================

//int todo(){
//  R.at(+grasp, comp_EQ, {AS_conv}, [](){
//    //do something
//    return 1;
//  });
//  R.at(+access, comp_GE, revision_number, [](){
//  });
//  R.at<arr>("q_ref", [](Access<T>& x){
//    return x->N>10;
//  }, [](){
//    //do something
//    return 0;
//  });
//}

int main(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

//  timedWait();
  atDemo();

  return 0;
}
