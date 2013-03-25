#include <system/engine.h>
#include <MT/graphview.h>

//NOTE: ComputeSum is in no way included, still we can create it

//===========================================================================
//
// testing
//

void testModule(const char* name){
  Item *modReg = registry().getItem("Decl_Module", name);
  CHECK(modReg,"");

  //create the module
  Module *mod = (Module*)modReg->value<Type>()->newInstance();
  //create the variables
  for_list_(Access, var, mod->accesses) var->createOwnData();
  //test
  mod->test();
}

void basicTesting(){
  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  SystemDescription S;
  S.addModule("ComputeSum");

  engine().test(S);

  cout <<S.system <<endl;

  cout <<registry() <<endl;
  GraphView gv(S.system);
  gv.watch();
}


//===========================================================================

//-- this is how the top-level manager should get access
int main(int argc, char** argv){
  basicTesting();
  return 0;
}
