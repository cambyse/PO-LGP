#include <MT/module.h>
#include <MT/keyValueGraph.h>
#include <MT/graphview.h>

//===========================================================================
//
// declaration of a module -- FIRST STYLE
//

//-- this is how the developer should provide a module
DECLARE_MODULE(ComputeSum);

struct ComputeSum:ComputeSum_Base {
  VAR(arr, x);    //input
  VAR(double, s); //output

  ComputeSum(){
    int i = 5-3;
    i++;
  }          //replaces old 'open'
  virtual ~ComputeSum(){} //replaces old 'close'

  virtual void step(){
    set_s( sum(get_x()) );
  }

  virtual bool test(){
    set_x( ARR(1., 2., 3.) );
    step();
    double S = get_s();
    CHECK(S==6.,"");
    cout <<"*** TEST SUCCESS ***" <<endl;
    return true;
  }
};


//===========================================================================
//
// testing
//

template<class T>
void testModule(){
  Item *modit = registry().getItem("moduledecl", typeid(T).name());
  CHECK(modit,"");

  //create the module
  T *mod = (T*)modit->value<KeyValueGraph>()->getItem("type")->value<TypeInfo>()->newInstance();
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

  cout <<registry() <<endl;
  GraphView gv(registry());
  gv.watch();

  return 0;
}


