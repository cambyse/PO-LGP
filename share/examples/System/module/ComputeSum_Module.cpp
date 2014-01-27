
//BEGIN_MODULE(ComputeSum)
//END_MODULE()

#include "ComputeSum_Module.h"

void ComputeSum::step(){
  s.set() = sum(x.get()());
}

bool ComputeSum::test(){
  x.set() = ARR(1., 2., 3.);
  step();
  double S = s.get();
  CHECK(S==6.,"");
  cout <<"*** TEST SUCCESS ***" <<endl;
  return true;
}

REGISTER_MODULE(ComputeSum);
