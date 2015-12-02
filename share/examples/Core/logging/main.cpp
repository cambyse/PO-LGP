#include <Core/thread.h>

//===========================================================================

// Normal Thread struct
struct MyThread: Thread{
  uint n;
  MyThread(uint n, double beatIntervalSec=0.):Thread(STRING("MyThread_"<<n), beatIntervalSec), n(n){}
  void open(){}
  void close(){}
  void step(){
    LOG(0) <<mlr::realTime() <<"sec Thread " <<n;
  }
};

void TEST(Logging){
  MyThread t1(1, .5), t2(2, .25);

  t1.threadLoop();
  t2.threadLoop();

  LOG(0) <<"starting to wait";
  mlr::wait(3.);

  LOG(0) <<"done with wait";

  t1.threadClose();
  t2.threadClose();
}


//===========================================================================

int MAIN(int argc,char** argv){
  mlr::initCmdLine(argc, argv);
  testLogging();

  return 0;
}
