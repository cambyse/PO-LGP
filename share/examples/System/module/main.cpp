#include <System/engine.h>
#include <Gui/graphview.h>

//NOTE: ComputeSum does not need to be included when using registry only!
#include "ComputeSum_Module.h"

//===========================================================================
//
// direct execution
//

void testExec(){
  System S;

#if 1
  ModuleThread *mt = S.addModule<ComputeSum> ("funnyName");
  S.complete(); //this will create the respective variables!

  Access_typed<arr> *x = S.getAccess<arr>("x");
  Access_typed<double> *s = S.getAccess<double>("s");

#else
  ComputeSum C;
  arr x;
  double s;
  S.addModule(&C,"funnyName");
  S.addVariable(&x, "x");
  S.addVariable(&s, "s");
#endif

  cout <<S <<endl;

  x->set() = ARRAY(1., 2., 3.);

#if 1 //serial
  mt->open();
  mt->step();
  mt->close();
#else //threaded
  mt->threadOpen();
  mt->threadStep();
  MT::wait(.001); //give it a tiny tiny bit of time to at least step once (it could happen that the condition variable has not waken up to detect the step BEFORE threadClose chanes the state again)
  mt->threadClose();
#endif

  cout <<"result = " <<s->get() <<endl;

}

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
//  testExec();
  basicTesting();
  return 0;
}
