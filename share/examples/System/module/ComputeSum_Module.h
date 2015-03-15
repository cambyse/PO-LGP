#include <Core/module.h>

struct MyType{
  int i;
};

struct ComputeSum:Module {
  ACCESS(arr, x);    //input
  ACCESS(double, s); //output
  ACCESS(MyType, i);

  ComputeSum(){}          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  virtual void step();
  virtual bool test();
};
