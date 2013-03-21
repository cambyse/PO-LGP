#include <MT/module.h>
#include <MT/graphview.h>

//===========================================================================
//
// testing
//

void testModule(const char* name){
  Item *modReg = registry().getItem("Decl_Module", name);
  CHECK(modReg,"");

  //create the module
  Module *mod = (Module*)modReg->value<TypeInfo>()->newInstance();
  //create the variables
  for_list_(Access, var, mod->accesses) var->createOwnData();
  //test
  mod->test();
}

void basicTesting(){
  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  testModule("10ComputeSum");

  cout <<registry() <<endl;
  GraphView gv(registry());
  gv.watch();
}


//===========================================================================

//-- this is how the top-level manager should get access
int main(int argc, char** argv){
  basicTesting();
  return 0;
}


