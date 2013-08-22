#include <System/module.h>

struct ComputeSum:Module {
  ACCESS(arr, x);    //input
  ACCESS(double, s); //output

  ComputeSum(){}          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  virtual void step();
  virtual bool test();
};
