#include <dummy/dummy.h>
#include <dummy/ComputeSum_Module.h>
#include <perception/perception.h>

//===========================================================================
//
// testing
//

void testModule(const char* name){
  Item *modReg = registry().getItem("moduledecl", name);
  CHECK(modReg,"");

  //create the module
  Module *mod = (Module*)modReg->value<KeyValueGraph>()->getItem("type")->value<TypeInfo>()->newInstance();
  //create the variables
  for_list_(VariableAccess, var, mod->accesses) var->createOwnData();
  //test
  mod->test();
}

//-- this is how the top-level manager should get access
int main(int argc, char** argv){
  Image a("A"),b("B");  newCvtGray(a,b); //only if this is done the OpencvCamera module is registered...

  cout <<"**** ENTER_MAIN" <<endl;

  cout <<registry() <<endl;

  testModule("10ComputeSum");

//  cout <<registry() <<endl;
//  GraphView gv(registry());
//  gv.watch();

  return 0;
}


