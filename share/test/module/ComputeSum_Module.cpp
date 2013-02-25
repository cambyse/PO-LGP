#include "ComputeSum_Module.h"

DECLARE_MODULE(ComputeSum);

struct ComputeSum:ComputeSum_Base {
  ACCESS(arr, x);    //input
  ACCESS(double, s); //output

  ComputeSum(){}          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  virtual void step();
  virtual bool test();
};

void ComputeSum::step(){
  set_s( sum(get_x()) );
}

bool ComputeSum::test(){
  set_x( ARR(1., 2., 3.) );
  step();
  double S = get_s();
  CHECK(S==6.,"");
  cout <<"*** TEST SUCCESS ***" <<endl;
  return true;
}

void ComputeSum2::step(){}
bool ComputeSum2::test(){ return false; }

