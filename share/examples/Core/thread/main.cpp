#include <Core/thread.h>

TStream tout(cout);

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
    tout.reg(this) << "MyOtherThread(" << n << "): ";
  }
  ~MyOtherThread() {
    tout.unreg(this);
  }
  void open(){}
  void close(){}
  void step(){
    tout(this) << "iteration " << i++ << endl;
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

  // tout example
  int nThreads = 2;
  MyOtherThread *tp[nThreads];

  for(int i = 0; i < nThreads; i++)
    tp[i] = new MyOtherThread(i);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadLoopWithBeat(.01);

  MT::wait(1.);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadClose();

  for(int i = 0; i < nThreads; i++)
    delete tp[i];

  MT::wait(1.);

  // tout usage examples
  char i = 'i';
  char j = 'j';
  
  tout() << "test without object" << endl;

  tout(&i) << "test with unregistered object" << endl;

  tout.reg(&i) << "Head " << i << ": ";
  tout(&i) << "test with registered object" << endl;

  tout.unreg(&i);
  tout(&i) << "test after unregistering object" << endl;

  tout.reg(&i) << "Head " << i << " v2.0: ";
  tout(&i) << "test after re-registering object" << endl;

  tout.reg(&i) << "Head " << i << " v3.0: ";
  tout(&i) << "test after re-registering object" << endl;

  tout.reg(&j) << "Head " << j << ": ";
  tout(&j) << "test with new object" << endl;

  tout.unreg_all();
  tout() << "tests after unregistering all:" << endl;
  tout(&i) << "test " << i << endl;
  tout(&j) << "test " << j << endl;
}

int MAIN(int argc,char** argv){
  testThread();

  return 0;
}
