#include "constrained.h"

#include <Optim/optimization.h>
#include <Optim/optimization_benchmarks.h>


void displayFunction(ScalarFunction& F){
  arr X, Y;
  X.setGrid(2,-1.2,1.2,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++) Y(i) = F.fs(NoArr, NoArr, X[i]);
  Y.reshape(101,101);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us (1.2*($1/50-1)):(1.2*($2/50-1)):3 w l", false, true);
}

void displayFunction(VectorFunction& F){
  arr phi;
  arr X, Y;
  X.setGrid(2,-1.2,1.2,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++){ F.fv(phi, NoArr, X[i]); Y(i) = sumOfSqr(phi); } //phi(0); }
  Y.reshape(101,101);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us (1.2*($1/50-1)):(1.2*($2/50-1)):3 w l", false, true);
}



//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint(VectorFunction& f, arr& x_start=NoArr, uint iters=10){
  enum MethodType { squaredPenalty=1, augmentedLag, logBarrier };

  MethodType method = (MethodType)MT::getParameter<int>("method");

  UnconstrainedProblem F(f);

  //switch on penalty terms
  switch(method){
  case squaredPenalty: F.mu=10.;  break;
  case augmentedLag:   F.mu=10.;  break;
  case logBarrier:     F.muLB=1.;  break;
  }

  uint d=MT::getParameter<uint>("dim", 2);
  arr x(d);
  if(&x_start) x=x_start;
  else{
    if(method==logBarrier) x = ARRAY(.3, .3); //log barrier needs a starting point
    else rndUniform(x, -1., 1.);
  }
  cout <<"x0=" <<x <<endl;


  system("rm -f z.grad_all");

  for(uint k=0;k<iters;k++){
    checkJacobian(f, x, 1e-4);
    //checkGradient(F, x, 1e-4); //very convenient: check numerically whether the gradient is correctly implemented
    //checkHessian (F, x, 1e-4);
    checkJacobian(F, x, 1e-4);

    //optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
//    optNewton(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    optGaussNewton(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));

    displayFunction((VectorFunction&)F);
    MT::wait();
    gnuplot("load 'plt'", false, true);
    MT::wait();

    //upate unconstraint problem parameters
    switch(method){
    case squaredPenalty: F.mu *= 10;  break;
    case augmentedLag:   F.augmentedLagrangian_LambdaUpdate(x);  break;
    case logBarrier:     F.muLB /= 2;  break;
    }

    system("cat z.grad >>z.grad_all");
    cout <<"x_opt=" <<x <<" mu=" <<F.mu <<" lambda=" <<F.lambda <<endl;
  }

  system("mv z.grad_all z.grad");
  gnuplot("load 'plt'", false, true);

  if(&x_start) x_start = x;
}



//==============================================================================
//
// test the phase one optimization
//

void testPhaseOne(VectorFunction& f){
  PhaseOneProblem metaF(f);

  arr x;
  x = ARRAY(1., 1., 10.);

  testConstraint(metaF, x, 1);
  //one iteration of phase one should be enough
  //properly done: check in each step if constraints are fulfilled and exit phase one then
  //no need to really minimize

  x=x.sub(0,-2);
  testConstraint(f, x);
}

//==============================================================================

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

//  ChoiceConstraintFunction F;
  SimpleConstraintFunction F;
  testConstraint(F);

  return 0;
}
