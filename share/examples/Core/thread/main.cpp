#include <Core/thread.h>

TStream tout(cout);
Mutex m;
//===========================================================================

// Normal Thread struct
struct MyThread: Thread{
  Variable<double>& x;
  uint n;
  MyThread(Variable<double>& x, uint n):Thread(STRING("MyThread_"<<n)), x(x), n(n){}
  void open(){}
  void close(){}
  void step(){
    x.set(this)++;
    COUT <<mlr::realTime() <<"sec Thread " <<n <<" is counting:" <<x.get() <<endl;
  }
};

void TEST(Thread){
  Variable<double> x(0.);
  MyThread t1(x, 1), t2(x, 2);

  t1.threadLoopWithBeat(.5);
  t2.listenTo(x); //whenever t1 modifies x, t2 is stepped
  
  mlr::wait(3.);

  t1.threadClose();
  t2.threadClose();

  CHECK(x.get()>=11. && x.get()<=15.,"");
}

//===========================================================================

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

void TEST(Throut){
  // tout example
  int nThreads = 2;
  MyOtherThread *tp[nThreads];

  for(int i = 0; i < nThreads; i++)
    tp[i] = new MyOtherThread(i);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadLoopWithBeat(.01);

  mlr::wait(1.);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadClose();

  for(int i = 0; i < nThreads; i++)
    delete tp[i];

  mlr::wait(1.);

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

//===========================================================================

int MAIN(int argc,char** argv){
  mlr::initCmdLine(argc, argv);
  testThread();
//  testVariable();
//  testThrout();

  return 0;
}
