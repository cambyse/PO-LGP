#include <Optim/constrained.h>

#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

#include <iomanip>

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

void testConstraint(ConstrainedProblem& p, arr& x_start=NoArr, uint iters=20){
  enum MethodType { squaredPenalty=1, augmentedLag, logBarrier };

  MethodType method = (MethodType)MT::getParameter<int>("method");

  UnconstrainedProblem UCP(p);

  //-- choose constrained method
  switch(method){
  case squaredPenalty: UCP.mu=10.;  break;
  case augmentedLag:   UCP.mu=1.;  break;
  case logBarrier:     UCP.muLB=1.;  break;
  }

  //-- initial x
  arr x(p.dim_x());
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
//    cout <<"x_start=" <<x <<flush; //<<" mu=" <<UCP.mu <<" \nlambda=" <<UCP.lambda <<" \ng=" <<elemWiseMax(UCP.g_x,0.) <<endl;
//    checkGradient(UCP, x, 1e-4);
    //checkHessian (UCP, x, 1e-4); //will throw errors: no Hessians for g!
//    checkAllGradients(p, x, 1e-4);
    //checkJacobian(p, x, 1e-4);

//    optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    OptNewton opt(x, UCP, OPT(verbose=1, damping=.1, stopTolerance=1e-2));
    opt.run();
    evals+=opt.evals;

    if(x.N==2){
      displayFunction((ScalarFunction&)UCP);
      MT::wait();
      gnuplot("load 'plt'", false, true);
      MT::wait();
    }

//    arr lambda_ = UCP.lambda;
//    arr z(lambda_.N); z.setZero();
//    for(uint i=0;i<z.N;i++) z(i) = (lambda_(i)>0. || UCP.g_x(i)>0.)?1.:0.;
//    cout <<"old lambda=" <<lambda <<endl;
//    cout <<"current g =" <<elemWiseMax(UCP.g_x,0.) <<endl;
//    cout <<"I_lambda  =" <<z <<"  --  " <<sum(z) <<endl;

    //upate unconstraint problem parameters
    switch(method){
    case squaredPenalty: UCP.mu *= 10;  break;
    case augmentedLag:
      //        UCP.anyTimeAulaUpdate(1., 2.0, &opt.fx, opt.gx, opt.Hx);
      UCP.aulaUpdate(1., x);//   UCP.mu *= 2.;
        break;
    case logBarrier:     UCP.muLB *=.5;  break;
    }
//    cout <<"current g =" <<UCP.g_x <<endl;

//    if(method==augmentedLag){
//      arr zz(lambda_.N); zz.setZero();
//      for(uint i=0;i<z.N;i++) zz(i) = (lambda_(i)<=1e-10 || UCP.lambda(i)>0.)?0.:1.;
//      cout <<" \tremain_active_cond="<< sum(zz) <<" \tlin_indep_cond=" <<sum(z) <<endl;
//    }

    system("cat z.grad >>z.grad_all");
    cout <<k <<' ' <<evals <<' ' <<"f(x)=" <<UCP.f_x <<" \tcompl=" <<sum(elemWiseMax(UCP.g_x,zeros(UCP.g_x.N,1))) <<" \tmu=" <<UCP.mu <<" \tmuLB=" <<UCP.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda;
    cout <<endl;
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<UCP.f_x <<"\nx_opt=" <<x <<"\nlambda=" <<UCP.lambda <<endl;

  system("mv z.grad_all z.grad");
  if(x.N==2) gnuplot("load 'plt'", false, true);

  if(&x_start) x_start = x;
}


//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint2(ConstrainedProblem& p, arr& x_start=NoArr, uint iters=20){
  ConstrainedMethodType method = (ConstrainedMethodType)MT::getParameter<int>("method");

  //-- initial x
  arr x(p.dim_x());
  if(&x_start) x=x_start;
  else{
    x.setZero();
  }
  cout <<"x0=" <<x <<endl;

  rnd.seed(0);

  optConstrained(x, NoArr, p, OPT(verbose=1, damping=1., stopTolerance=1e-4, stepDec=.5, constrainedMethod=method));

  if(&x_start) x_start = x;
}

//==============================================================================
//
// test the phase one optimization
//

void testPhaseOne(ConstrainedProblem& f){
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

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ChoiceConstraintFunction F;
//  SimpleConstraintFunction F;
  testConstraint(F);
//    testConstraint2(F);

  return 0;
}
