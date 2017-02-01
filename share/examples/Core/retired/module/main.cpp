//#include <System/engine.h>
#include <Gui/graphview.h>
#include <Core/thread.h>

//NOTE: ComputeSum does not need to be included when using registry only!
#include "ComputeSum_Module.h"


//===========================================================================
//
// not using any 'system' code, directly creating the modules and 'completing'
// the variables

void way0(){
  ComputeSum C;
  C.x.set() = {1., 2., 3.};
  C.open();
  C.step();
  C.close();
  cout <<C.s.get() <<endl;
}

//===========================================================================
//
// direct execution - way1
//

void way1(){

//  Module *m = S.addModule<ComputeSum> ("funnyName");
  Thread *m = new ComputeSum;
  //S.connect(); //this will create the respective variables!

  Access_typed<arr> x(NULL, "x");
  Access_typed<double> s(NULL, "s");

#if 0
  ComputeSum C;
  arr x;
  double s;
  S.addModule(&C,"funnyName");
  S.addVariable(&x, "x");
  S.addVariable(&s, "s");
#endif

  x.set() = {1., 2., 3.};

#if 1 //serial
  m->open();
  m->step();
  m->close();
#else //threaded
  m->thread->threadOpen();
  m->thread->threadStep();
  mlr::wait(.001); //give it a tiny tiny bit of time to at least step once (it could happen that the condition variable has not waken up to detect the step BEFORE threadClose chanes the state again)
  mt->threadClose();
#endif

  cout <<"result = " <<s.get() <<endl;
  delete m;

}

//===========================================================================
//
// direct execution - way2
//

struct MySystem{
  ACCESSname(arr, x)
  ACCESSname(double, s)
  ComputeSum cs;
};

void way2(){
  cout <<registry() <<endl;

  MySystem S;

  S.x.set() = {1., 2., 3.};

  openModules();
  stepModules();
  closeModules();

  cout <<"result = " <<S.s.get() <<endl;
};

//===========================================================================
//
// testing
//

void TEST(SystemConnect) {
  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  ComputeSum CS;

  cout <<registry() <<endl;

  registry().displayDot();

//  GraphView gv(registry());
//  gv.watch();
}


//===========================================================================

int MAIN(int argc, char** argv){
  int mode=0;
  if(argc>1) mode=atoi(argv[1]);
  switch(mode){
    case 0: way0(); way1(); way2(); testSystemConnect(); break;
    case 1: way0(); break;
    case 2: way1(); break;
    case 3: way2(); break;
    case 4: testSystemConnect(); break;
  }

  return 0;
}
