#include <Core/thread.h>

//===========================================================================

// Normal Thread struct
struct MyThread: Thread{
  uint n;
  MyThread(uint n):Thread(STRING("MyThread_"<<n)), n(n){}
  void open(){}
  void close(){}
  void step(){
    LOG(0) <<MT::realTime() <<"sec Thread " <<n;
  }
};

void TEST(Logging){
  MyThread t1(1), t2(2);

  t1.threadLoopWithBeat(.5);
  t2.threadLoopWithBeat(.25);

  LOG(0) <<"starting to wait";
  MT::wait(3.);

  LOG(0) <<"done with wait";

  t1.threadClose();
  t2.threadClose();
}


//===========================================================================

int MAIN(int argc,char** argv){
  MT::initCmdLine(argc, argv);
  testLogging();

  return 0;
}
