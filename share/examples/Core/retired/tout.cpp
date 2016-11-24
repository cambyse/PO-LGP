//==============================================================================

// Thread struct with throut
struct MyOtherThread: Thread {
  uint n, i;
  MyOtherThread(uint _n, double beat):Thread(STRING("MyOtherThread_"<<n), beat), n(_n), i(0) {
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
    tp[i] = new MyOtherThread(i, .01);

  for(int i = 0; i < nThreads; i++)
    tp[i]->threadLoop();

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
