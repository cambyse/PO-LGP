#include <Core/thread.h>

// Normal Thread struct
struct MyThread: Thread{
  uint n,i;
  MyThread(uint _n):Thread(STRING("MyThread_"<<n)), n(_n), i(0){}
  void open(){}
  void close(){}
  void step(){
    cout <<"Thread " <<n <<" is counting:" <<i++ <<endl;
  }
};

// Thread struct with throut
struct MyOtherThread: Thread {
  uint n, i;
  MyOtherThread(uint _n):Thread(STRING("MyOtherThread_"<<n)), n(_n), i(0) {
    throut::throutRegHeading(this, STRING("MyOtherThread(" << n << "): "));
  }
  ~MyOtherThread() {
    throut::throutUnregHeading(this);
  }
  void open(){}
  void close(){}
  void step(){
    throut::throut(this, STRING("iteration " << i++));
  }
};

int main(int argn,char** argv){
  // Thread example code
  MyThread t1(1), t2(2);

  t1.threadLoopWithBeat(1.);
  t2.threadLoopWithBeat(1.5);
  
  MT::wait(10.);

  t1.threadClose();
  t2.threadClose();

  // throut example code
  int nThreads = 2;
  MyOtherThread *tp[nThreads];

  for(int i = 0; i < nThreads; i++)
    tp[i] = new MyOtherThread(i);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadLoop();

  MT::wait(2.);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadClose();

  for(int i = 0; i < nThreads; i++)
    delete tp[i];

  return 0;
}
