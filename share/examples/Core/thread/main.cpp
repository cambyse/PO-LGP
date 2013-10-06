#include <Core/thread.h>

struct MyThread: Thread{
  uint n,i;
  MyThread(uint _n):Thread(STRING("MyThread_"<<n)), n(_n), i(0) {}
  void open(){}
  void close(){}
  void step(){
    cout <<"Thread " <<n <<" is counting:" <<i++ <<endl;
  }
};


void TEST(Thread){
  MyThread t1(1), t2(2);

  t1.threadLoopWithBeat(.1);
  t2.threadLoopWithBeat(1.);
  
  MT::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(t1.i>=29 && t1.i<=31,"");
  CHECK(t2.i>=2 && t2.i<=4,"");
}

int MAIN(int argn,char** argv){
  testThread();

  return 0;
}
