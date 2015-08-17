#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

#include <iomanip>

//this code is only for demo in the lecture -- a bit messy!

void displayFunction(const VectorFunction& f){
  arr phi;
  arr X, Y;
  X.setGrid(2,-1.2,1.2,100);
  Y.resize(X.d0);
  for(uint i=0;i<X.d0;i++){ f(phi, NoArr, X[i]); Y(i) = sumOfSqr(phi); } //phi(0); }
  Y.reshape(101,101);
  write(LIST<arr>(Y),"z.fct");
  gnuplot("reset; splot [-1:1][-1:1] 'z.fct' matrix us (1.2*($1/50-1)):(1.2*($2/50-1)):3 w l", false, true);
}



//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint(const ConstrainedProblemMix& p, uint dim_x, arr& x_start=NoArr, uint iters=20){

  ConstrainedMethodType method = (ConstrainedMethodType)MT::getParameter<int>("opt/constrainedMethod");

  UnconstrainedProblemMix UCP(p, method);

  //-- choose constrained method
  switch(method){
  case squaredPenalty: UCP.mu=10.; UCP.nu=10.;  break;
  case augmentedLag:   UCP.mu=1.;  UCP.nu=1.;   break;
  case logBarrier:     UCP.muLB=1.;  UCP.nu=1.;   break;
  default: NIY;
  }

  //-- initial x
  arr x(dim_x);
  if(&x_start) x=x_start;
  else{
    x.setZero();
    if(method==logBarrier){ } //log barrier needs a feasible starting point
    else rndUniform(x, -1., 1.);
  }
//  cout <<std::setprecision(2);
  cout <<"x0=" <<x <<endl;

  rnd.seed(0);

  system("rm -f z.grad_all");

  uint evals=0;
  for(uint k=0;k<iters;k++){
//    checkGradient(UCP, x, 1e-4);
//    checkHessian (UCP, x, 1e-4); //will throw errors: no Hessians for g!
//    checkAllGradients(p, x, 1e-4);

    if(x.N==2){
      displayFunction(UCP);
      MT::wait();
      gnuplot("load 'plt'", false, true);
      MT::wait();
    }

    //optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    OptNewton opt(x, UCP, OPT(verbose=1, damping=.1, stopTolerance=1e-2));
    opt.run();
    evals+=opt.evals;

//    arr lambda_ = UCP.lambda;

    //upate unconstraint problem parameters
    switch(method){
    case squaredPenalty: UCP.mu *= 10;  UCP.nu *= 10;  break;
    case augmentedLag:
      //        UCP.anyTimeAulaUpdate(1., 2.0, &opt.fx, opt.gx, opt.Hx);
      UCP.aulaUpdate(1.);//   UCP.mu *= 2.;
        break;
    case logBarrier:     UCP.muLB *=.5;  UCP.nu *= 10;  break;
    default: NIY;
    }

//    if(method==augmentedLag){
//      arr zz(lambda_.N); zz.setZero();
//      for(uint i=0;i<z.N;i++) zz(i) = (lambda_(i)<=1e-10 || UCP.lambda(i)>0.)?0.:1.;
//      cout <<" \tremain_active_cond="<< sum(zz) <<" \tlin_indep_cond=" <<sum(z) <<endl;
//    }

    system("cat z.grad >>z.grad_all");
    cout <<k <<' ' <<evals <<" f(x)=" <<UCP.get_sumOfSquares()
	 <<" \tg_compl=" <<UCP.get_sumOfGviolations()
	 <<" \th_compl=" <<UCP.get_sumOfHviolations()
      <<" \tmu=" <<UCP.mu <<" \tnu=" <<UCP.nu <<" \tmuLB=" <<UCP.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda /*<<" \tg=" <<UCP.g_x <<" \th=" <<UCP.h_x*/;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<UCP.get_sumOfSquares() <<"\nx_opt=" <<x <<"\nlambda=" <<UCP.lambda <<endl;

  system("mv z.grad_all z.grad");
  if(x.N==2) gnuplot("load 'plt'", false, true);

  if(&x_start) x_start = x;
}
