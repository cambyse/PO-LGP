#include "smallWorld.h"


//-- this is how the developer should provide a module

struct ComputeSum:Module{
  VAR(arr, x);
  VAR(double, s);
  //PARAM(double, y, 2.);

  ComputeSum():x_access(this), s_access(this) {}

  void step(){
    set_s(sum(get_x()));
  }

  void read(std::istream& is){}
  void write(std::ostream& os) const{}
};
stdPipes(ComputeSum);

REGISTER_DERIVED_TYPE(ComputeSum, Module);



//-- this is how the top-level manager should get access

int main(int argc, char** argv){

  dumpAllRegisteredModules();

  return 0;
}
