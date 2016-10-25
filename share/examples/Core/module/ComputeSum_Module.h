#include <Core/thread.h>

struct MyType{
  int i;
};

struct ComputeSum : Thread {
  ACCESS(arr, x);    //input
  ACCESS(double, s); //output
  ACCESS(MyType, i);

  ComputeSum():Thread("ComputeSum"){}          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  void open(){}
  void close(){}
  void step();
  bool test();
};
