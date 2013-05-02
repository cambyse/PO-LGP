#include <MT/optimization.h>
#include <MT/optimization_benchmarks.h>
#include <MT/plot.h>

void displayFunction(ScalarFunction& F){
  arr X, Y;
  X.setGrid(2,-1.,1.,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++) Y(i) = F.fs(NoArr, NoArr, X[i]);
  Y.reshape(101,101);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("splot [-1:1][-1:1] 'z.fct' matrix us ($1/50-1):($2/50-1):3 w l", false, true);
}

void testGradDescent(ScalarFunction& F){
  arr x(2),x0;
  rnd.clockSeed();
  for(uint k=0;k<3;k++){
    rndUniform(x, -1., 1.);
    x0=x;
    cout <<"x0=" <<x0 <<endl;
    checkGradient(F, x, 1e-4);

    optGradDescent(x, F, OPT2(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();

    x=x0;
    optRprop(x, F, OPT2(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();
  }
}

int main(int argn,char** argv){
  ChoiceFunction F;
  displayFunction(F);
  MT::wait();

  testGradDescent(F);

  return 0;
}
