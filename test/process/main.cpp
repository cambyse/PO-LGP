#define MT_IMPLEMENTATION

#include <MT/process.h>
#include <MT/util.h>

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

int main(int argc, char *argv[]){
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
    //C.threadStep();
    //D.threadStep();
    cout <<"*** main process loop iteration " <<t <<endl;
    //win.threadStep();//globalThreadInfoWin.step();
    ticcer.waitForTic();
  }

  A.threadClose();
  B.threadClose();
  C.threadClose();
  D.threadClose();
  E.threadClose();
  win.threadClose();
  
  return 0;
}

