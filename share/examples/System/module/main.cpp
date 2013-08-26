#include <System/engine.h>
#include <Gui/graphview.h>

//NOTE: ComputeSum does not need to be included when using registry only!
#include "ComputeSum_Module.h"

//===========================================================================
//
// direct execution - way1
//

void way1(){
  System S;

//  Module *m = S.addModule<ComputeSum> ("funnyName");
  Module *m = S.addModule("ComputeSum", "funnyName");
  S.complete(); //this will create the respective variables!
  cout <<S <<endl;

  Access_typed<arr> *x = S.getAccess<arr>("x");
  Access_typed<double> *s = S.getAccess<double>("s");

#if 0
  ComputeSum C;
  arr x;
  double s;
  S.addModule(&C,"funnyName");
  S.addVariable(&x, "x");
  S.addVariable(&s, "s");
#endif

  x->set() = ARRAY(1., 2., 3.);

#if 1 //serial
  m->open();
  m->step();
  m->close();
#else //threaded
  m->thread->threadOpen();
  m->thread->threadStep();
  MT::wait(.001); //give it a tiny tiny bit of time to at least step once (it could happen that the condition variable has not waken up to detect the step BEFORE threadClose chanes the state again)
  mt->threadClose();
#endif

  cout <<"result = " <<s->get() <<endl;

}

//===========================================================================
//
// direct execution - way2
//

struct MySystem:System{
  ACCESS(arr, x);
  ACCESS(double, s);
  MySystem():System("hallo"){
    addModule<ComputeSum> ("funnyName");
    complete(); //this will create the respective variables!
  }
};

void way2(){
  MySystem S;

  cout <<S <<endl;

  S.x.set() = ARRAY(1., 2., 3.);

  S.open();
  S.step();
  S.close();

  cout <<"result = " <<S.s.get() <<endl;
};

//===========================================================================
//
// testing
//

void basicTesting(){
  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  System S;
  S.addModule("ComputeSum", "funnyName");

  cout <<S <<endl;

  S.complete();
  cout <<S <<endl;

  engine().test(S);

  cout <<registry() <<endl;
  KeyValueGraph g = S.graph();
  GraphView gv(g);
  gv.watch();
}


//===========================================================================

//-- this is how the top-level manager should get access
int main(int argc, char** argv){
//  way1();
  way2();
//  basicTesting();
  return 0;
}
