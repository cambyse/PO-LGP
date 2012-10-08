#include <biros/biros.h>
#include <biros/control.h>

//-- standard Variable containing only an integer
struct Integer:public Variable {
  FIELD(int, x);
  
  Integer():Variable("IntVar") { reg_x(); x=rnd(100); }
};


int main(int argn, char **argv) {

  b::dump();

  Integer Int;

  b::openInsideOut();

  for(uint t=0; t<1000; t++){
    MT::wait(.1);
    Int.set_x(t, NULL);
  }
  
  return 0;
}

