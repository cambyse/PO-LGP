#include <Core/module.h>

struct MyType{
  int i;
};

struct ComputeSum : Module {
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
