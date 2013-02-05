#include "modules.h"
#include <MT/mapGraph.h>

//===========================================================================
//
// declaration of a module
//

struct ArrRepresentation:arr,TypeBase{};
struct DoubleRepresentation:arr,TypeBase{};

//-- this is how the developer should provide a module
struct ComputeSum: MODULE_BASE(ComputeSum){
  VAR(arr, x);    //input
  VAR(double, s); //output

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
}

//===========================================================================
//
// testing
//

template<class T>
void testModule(){
  Item *modit = registry().getItem("module", typeid(T).name());
  //create the module
  T *mod = (T*)modit->newInstance();
  //create the variables
  for_list_(VariableAccess, var, mod->accesses) var->createOwnVariable();
  //test
  mod->test();
}

//-- this is how the top-level manager should get access
int main(int argc, char** argv){

  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  testModule<ComputeSum>();

  return 0;
}


