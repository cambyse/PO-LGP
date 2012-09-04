#include "brachiation.h"

int main (const int argc, char ** argv){

  BrachiationSystem sys;

  arr A,a,B,Q;
  arr x(sys.get_xDim());
  x.setZero();
  sys.setx(x);
  sys.getDynamics(A,a,B,Q,0);
  
  return 0;
}

