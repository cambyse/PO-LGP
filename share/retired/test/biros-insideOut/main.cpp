#include <system/engine.h>
#include <views/views.h>

//-- standard AccessData containing only an integer
struct Integer:public AccessData {
  int x;
  
  Integer():AccessData("IntVar") { x=rnd(100); }
};


int main(int argc, char **argv) {

  //  biros().dump();

  Integer Int;

  new InsideOut();


  for(uint t=0; t<1000; t++){
    mlr::wait(.1);
    Int.set_x(t, NULL);
  }
  
  return 0;
}

