#include <Core/thread.h>

// Normal Thread struct
struct MyThread: Thread{
  uint n,i;
  MyThread(uint _n):Thread(STRING("MyThread_"<<n)), n(_n), i(0) {}
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

void TEST(Thread){
  MyThread t1(1), t2(2);

  t1.threadLoopWithBeat(.1);
  t2.threadLoopWithBeat(1.);
  
  MT::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(t1.i>=29 && t1.i<=31,"");
  CHECK(t2.i>=2 && t2.i<=4,"");

#if 0 //TODO - left over of a merge conflict
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
#endif
}

int MAIN(int argc,char** argv){
  testThread();

  return 0;
}
