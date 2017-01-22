#include <Net/net.h>
#include <Net/functions.h>


//===========================================================================

int main(int argc, char **argv){
  mlr::initCmdLine(argc, argv);

  //-- specify layer sizes of a neural net
  uintA h = { 3, 100, 20, 50, 10, 4 };
  uint L = h.N-2;

  //-- construct the NN
  Net N;
  Variable *x = N.newConstant("x_0", TUP(h(0)), true );
  Variable *z;
  for(uint l=1;l<=L+1;l++){
    Variable *W = N.newConstant(STRING("W_"<<l-1), TUP(h(l), h(l-1)), true );
    Variable *b = N.newConstant(STRING("b_"<<l-1), TUP(h(l)), true );
    z = N.newFunction(STRING("z_"<<l), {x, W, b}, new Linear(), TUP(h(l)) );
    if(l<=L)
      x = N.newFunction(STRING("x_"<<l), {z}, new Sigmoid(), TUP(h(l)) );
  }
  Variable *out=z;
  N.G.displayDot();

  //-- set random weights
  N.randParameters();

  //-- define vector-valued function
  VectorFunction f =
      [&N,out](arr& y, arr& J, const arr& x)->void{
    N.setAllParameters(x);
    N.fwdCompute();
    y = out->value;
    if(&J){
      N.zeroAllPartialDerivatives(y.N);
      out->del.setId();
      N.bwdCompute();
      J = N.getAllParameterJacobians(y.N, x.N);
    }
  };

  //-- check gradient w.r.t. all constants (input and weights
  arr w = N.getAllParameters();
  checkJacobian(f, w, 1e-4);

  return 0;
}

