#include <Core/thread.h>

struct MyThread: Thread{
  uint n,i;
  MyThread(uint _n):Thread(STRING("MyThread_"<<n)), n(_n), i(0){}
  void open(){}
  void close(){}
  void step(){
    cout <<"Thread " <<n <<" is counting:" <<i++ <<endl;
  }
};


int main(int argn,char** argv){
  MyThread t1(1), t2(2);

  t1.threadLoopWithBeat(1.);
  t2.threadLoopWithBeat(1.5);
  
  MT::wait(10.);

  t1.threadClose();
  t2.threadClose();

  return 0;
}
