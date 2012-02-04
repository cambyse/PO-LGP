#define MT_IMPLEMENT_TEMPLATES
#include <MT/process.h>
#include <MT/util.h>

#include "process_monitor.h"

//===========================================================================
//
// test looping multiple threads on beat or sync
//

struct TestThread:public Process{
  const char* name;
  double sec;
  int x;

  TestThread(const char* _name,double _sec):Process(_name){
    name=_name;
    sec=_sec;
    x=0;
    threadOpen();
  }
  ~TestThread(){ threadClose(); }

  void open (){ cout <<name <<" is opening,  x=" <<x <<endl; }
  void close(){ cout <<name <<" is closing,  x=" <<x <<endl; }
  void step (){ if(sec) MT::wait(sec); x++; cout <<name <<" has stepped, x=" <<x <<endl; }
};


void testLoop(){
  //Fl::lock();
  ThreadInfoWin win;
  win.threadLoopWithBeat(.1);

  TestThread A("A loop (self=.08)",.08);
  TestThread E("E slave of B (self=.0)",.0);
  TestThread B("B loop beat (self=.02)",.02);
  TestThread C("C step (self=.07)",.07);
  TestThread D("D step (self=.03)",.03);
  //TestThread E("E loop slave of A (self=.0)",.0);
  A.threadLoop();
  B.threadLoopWithBeat(.11);
  E.threadLoopSyncWithDone(B);

  Metronome ticcer("ticcer (self=.1)",100);

  for(uint t=0;t<20;t++){
    C.threadStep();
    D.threadStep();
    cout <<"*** main process loop iteration " <<t <<endl;
    //win.threadStep();//globalThreadInfoWin.step();
    ticcer.waitForTic();
  }

  A.threadClose();
  B.threadClose();
  C.threadClose();
  D.threadClose();
  E.threadClose();

  MT::wait();

  win.threadClose();
}



//===========================================================================
//
// test excessive access to Variables
//

struct IntVar:public Variable{
  //BIR_VARIABLE;
  
  FIELD(int, x);

  //BIR_FIELD(bool, mybool);
  
  IntVar():Variable("IntVar"){ x=rnd(1000); reg_x(); }
};

//int IntVar::bir_typeId=-1;

uint PC=0;

struct Maxxer:public Process{
  IntVar *a,*b;

  Maxxer():Process("Maxxer"){};
  
  void open (){}
  void close(){}
  void step (){
    int xa=a->get_x(this);
    int xb=b->get_x(this);
    if(xa>xb) b->set_x(xa, this);
    else a->set_x(xb, this);
  }
};

void testMultiAccess(){
  uint n=MT::getParameter<uint>("n",100);
  MT::Array<IntVar> vars(n);
  MT::Array<Maxxer> procs(2*n);

  for(uint i=0;i<procs.N;i++){
    procs(i).a = &vars.rndElem();
    procs(i).b = &vars.rndElem();
  }

  for(uint i=0;i<procs.N;i++) procs(i).threadLoopWithBeat(rnd.uni(.001,.01));
  MT::wait(1.);
  for(uint i=0;i<procs.N;i++) procs(i).threadClose();

  dumpInfo();

  
  
  for(uint i=0;i<vars.N;i++) cout <<vars(i).x <<' ';
  cout <<endl;
}

int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);
  //testLoop();
  testMultiAccess();
  
  return 0;
}

