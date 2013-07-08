#include <System/module.h>

BEGIN_MODULE(ComputeSum)
ACCESS(arr, x);    //input
ACCESS(double, s); //output
END_MODULE()

struct ComputeSum:ComputeSum_Base {
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


