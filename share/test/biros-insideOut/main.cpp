#include <system/engine.h>
#include <views/views.h>

//-- standard Variable containing only an integer
struct Integer:public Variable {
  FIELD(int, x);
  
  Integer():Variable("IntVar") { x=rnd(100); }
};


int main(int argn, char **argv) {

  //  biros().dump();

  Integer Int;

  new InsideOut();


  for(uint t=0; t<1000; t++){
    MT::wait(.1);
    Int.set_x(t, NULL);
  }
  
  return 0;
}

