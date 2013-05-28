#include <Core/array.h>
#include <MT/util.h>
#include <MT/optimization.h>

struct ExampleFunction:public VectorFunction{
  double q;
  ExampleFunction(){ q = MT::getParameter<double>("q",10.); }
  void fv(arr& phi, arr& J,const arr& x){
    CHECK(x.N==1,"");
    phi.resize(1);
    if(&J) J.resize(1);
#if 0
    phi(0) = sin(x(0));
    if(&J) J  (0) = cos(x(0));
#else //gnuplot: plot 1-1/(x**10+1)
    double y=1./(pow(x(0),q)+1);
    phi(0) = 1.-y;
    if(&J) J  (0) = y*y * q * pow(x(0),q-1.);
#endif
  }
};


void testGaussNewton(){
  cout <<"For this function (gnuplot: plot 1-1/(x**10+1)) the initial step size is\n\
  much too large (because of the plateau at the starting point x=10). Therefore the initial\n\
  steps are rejected. Later, convergence is fast." <<endl;
  ExampleFunction f;
  arr x(1);
  x=1.;
  x=10.;
  optGaussNewton(x, f, OPT2(stopTolerance=1e-5, verbose=3));
}

int main(int argc, char *argv[]){
  MT::initCmdLine(argc,argv);
  MT::verboseLevel=2;
  
  testGaussNewton();
  return 0;
}

