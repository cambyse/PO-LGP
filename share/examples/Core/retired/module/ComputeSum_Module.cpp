
//BEGIN_MODULE(ComputeSum)
//END_MODULE()

#include "ComputeSum_Module.h"

void ComputeSum::step(){
  s.set() = sum(x.get()());
}

bool ComputeSum::test(){
  x.set() = {1., 2., 3.};
  step();
  double S = s.get();
  CHECK_EQ(S,6.,"");
  cout <<"*** TEST SUCCESS ***" <<endl;
  return true;
}

//REGISTER_MODULE(ComputeSum);
