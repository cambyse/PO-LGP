#include "smallWorld.h"


//-- this is how the developer should provide a module

struct ComputeSum:Module{
  VAR(arr, x);   //input
  VAR(double, s); //output

  ComputeSum(): VARc(x), VARc(s) {} //constructors of access objects

  void step(){
    set_s(sum(get_x()));
  }
};

REGISTER_MODULE(ComputeSum);


//-- this is how the top-level manager should get access

int main(int argc, char** argv){

  cout <<registry() <<endl;

  return 0;
}
