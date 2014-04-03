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

void testAula(ConstrainedProblem& p){

  UnconstrainedProblem UCP(p);

  UCP.mu=1.;

  uint d=MT::getParameter<uint>("dim", 2);
  arr x(d);
  rndUniform(x, -1., 1.);
  cout <<"x0=" <<x <<endl;

  rnd.seed(0);

  system("rm -f z.grad_all");

  OptNewton opt(x, UCP, OPT(verbose=2, damping=1., stopTolerance=1e-6, stepInc=1.));

  for(uint k=0;k<100;k++){
//    cout <<"x_start=" <<x <<" mu=" <<UCP.mu <<" lambda=" <<UCP.lambda <<endl;
//    checkGradient(UCP, x, 1e-4);
//    checkAllGradients(p, x, 1e-4);

//    optNewton(x, UCP, OPT(verbose=2));
#if 0
    arr g,H;
    double f = UCP.fs(g, H, x);
    H += 1e-0 *eye(H.d0);
    cout <<"f(x)=" <<f-UCP.f0  <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda <<" \tf0=" <<UCP.f0  <<endl;
    arr Delta = -inverse_SymPosDef(H)*g;
    x += 1. * Delta;
#else
//    OptNewton(x, UCP, OPT(verbose=1, damping=1., stopTolerance=1e-6, stepInc=1.)).step();
    OptNewton::StopCriterion res = opt.step();
    cout <<"f(x)=" <<UCP.f_x <<" \tmu=" <<UCP.mu <<" \tmuLB=" <<UCP.muLB;
    if(x.N<5) cout <<" \tx=" <<x <<" \tlambda=" <<UCP.lambda;
    cout <<endl;
#endif

    if(res) break;
    UCP.aula_update(x, .1, &opt.fx, opt.gx, opt.Hx);
  }
  cout <<std::setprecision(6) <<"\nf(x)=" <<UCP.f_x <<"\nx_opt=" <<x <<"\nlambda=" <<UCP.lambda <<endl;
}


//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  ChoiceConstraintFunction F;
  testAula(F);

  return 0;
}
