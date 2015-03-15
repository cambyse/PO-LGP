#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <stdlib.h>

const char* USE="\n\
Choose\n\
  1) the cost function\n\
  2) the conditioning\n\
  3) exercise task\n\
  4) if constrained, which method is used\n\
in the MT.cfg (or command line)\n\
\n\
The constraint is always as in exercise 3\n";


//==============================================================================
//
// plain grad descent
//

void testGradDescent(const ScalarFunction& F){
  uint d=MT::getParameter<uint>("dim", 2);
  arr x(d),x0;
  rnd.clockSeed();
  for(uint k=0;k<3;k++){
    rndUniform(x, -1., 1.);
    x0=x;
    cout <<"x0=" <<x0 <<endl;
    checkGradient(F, x, 1e-4);
    checkHessian(F, x, 1e-4);

    optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();

    x=x0;
    optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();

    x=x0;
    optNewton(x, F, OPT(verbose=3, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();
  }
}


//==============================================================================
//
// we define an unconstraint optimization problem from a constrained one
// that can include penalties, log barriers, and augmented lagrangian terms
//

// struct UnconstrainedProblem:ScalarFunction{
//   VectorFunction &f; // see below for the meaning of the VectorFunction
//   double muLB;   // log barrier weight
//   double mu;     // squared penalty weight
//   arr lambda;    // lagrange multiplier in augmented lagrangian

//   UnconstrainedProblem(VectorFunction &_f):f(_f), muLB(0.), mu(0.) {}

//   virtual double fs(arr& g, arr& H, const arr& x){
//     //the VectorFunction F describes the cost function f(x) as well as the constraints g(x)
//     //concatenated to one vector:
//     // phi(0) = cost,   phi(1,..,phi.N-1) = constraints

//     arr phi, J;
//     f.fv(phi, (&g?J:NoArr), x);

//     //in log barrier case, check feasibility
//     if(muLB)     for(uint i=1;i<phi.N;i++) if(phi(i)>0.) return NAN; //CHECK(phi(i)<=0., "log barrier: constraints must be fulfiled!");

//     double f = phi(0); //costs
//     if(muLB)     for(uint i=1;i<phi.N;i++) f -= muLB * ::log(-phi(i));  //log barrier
//     if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) f += mu * MT::sqr(phi(i));  //penalty
//     if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) f += lambda(i) * phi(i);  //augments

//     if(&g){
//       g = J[0]; //costs
//       if(muLB)     for(uint i=1;i<phi.N;i++) g -= (muLB/phi(i))*J[i];  //log barrier
//       if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) g += (mu*2.*phi(i))*J[i];  //penalty
//       if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) g += lambda(i)*J[i];  //augments
//       g.reshape(x.N);
//     }

//     if(&H){
//       ///TODO: Here we assume the hessian of phi(0) and all phi(i) ZERO!!! Only the J^T J terms are considered (as in Gauss-Newton type)
//       H.resize(x.N,x.N);
//       H.setZero();
//       if(muLB)     for(uint i=1;i<phi.N;i++) H += (muLB/MT::sqr(phi(i)))*(J[i]^J[i]);  //log barrier
//       if(mu)       for(uint i=1;i<phi.N;i++) if(phi(i)>0. || (lambda.N && lambda(i)>0.)) H += (mu*2.)*(J[i]^J[i]);  //penalty
//       if(lambda.N) for(uint i=1;i<phi.N;i++) if(lambda(i)>0.) H += 0.; //augments
//       H.reshape(x.N,x.N);
//     }

//     return f;
//   }

//   void augmentedLagrangian_LambdaUpdate(const arr& x){
//     arr phi;
//     f.fv(phi, NoArr, x);

//     if(!lambda.N){ lambda.resize(phi.N); lambda.setZero(); }

//     for(uint i=1;i<phi.N;i++) if(phi(i)>0. || lambda(i)>0.) lambda(i) += mu * 2.*phi(i);

//     for(uint i=1;i<phi.N;i++) if(lambda(i)<0.) lambda(i)=0.;

//     cout <<"Update Lambda: phi=" <<phi <<" lambda=" <<lambda <<endl;
//   }
// };


//==============================================================================
//
// test standard constrained optimizers
//

void testConstraint(ConstrainedProblem& f, arr& x_start=NoArr, uint iters=10){
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

  system("rm z.grad_all");

  for(uint k=0;k<iters;k++){
    checkAllGradients(f, x, 1e-4);
    checkGradient(F.Lag, x, 1e-4); //very convenient: check numerically whether the gradient is correctly implemented
    checkHessian (F.Lag, x, 1e-4);

    //optRprop(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    //optGradDescent(x, F, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));
    optNewton(x, F.Lag, OPT(verbose=2, stopTolerance=1e-3, initStep=1e-1));

    displayFunction(F.Lag);
    MT::wait();
    gnuplot("load 'plt'", false, true);
    MT::wait();

    //upate unconstraint problem parameters
    switch(method){
    case squaredPenalty: F.mu *= 10;  break;
    case augmentedLag:   F.aulaUpdate();  break;
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
// we define a constraint optimization problem that corresponds
// to the phase one problem of another constraint problem
//

// struct PhaseOneProblem:VectorFunction{
//   VectorFunction &f;

//   PhaseOneProblem(VectorFunction &_f):f(_f) {}

//   virtual void fv(arr& metaPhi, arr& metaJ, const arr& x){
//     arr phi, J;
//     f.fv(phi, (&metaJ?J:NoArr), x.sub(0,-2)); //the underlying problem only receives a x.N-1 dimensional x

//     metaPhi.resize(phi.N+1);
//     metaPhi(0) = x.last();                                     //cost
//     for(uint i=1;i<phi.N;i++) metaPhi(i) = phi(i)-x.last();    //slack constraints
//     metaPhi.last() = -x.last();                                //last constraint

//     if(&metaJ){
//       metaJ.resize(metaPhi.N, x.N);  metaJ.setZero();
//       metaJ(0,x.N-1) = 1.; //cost
//       for(uint i=1;i<phi.N;i++) for(uint j=0;j<x.N-1;j++) metaJ(i,j) = J(i,j);
//       for(uint i=1;i<phi.N;i++) metaJ(i,x.N-1) = -1.;
//       metaJ(phi.N, x.N-1) = -1.;
//     }
//   }
// };


//==============================================================================
//
// test the phase one optimization
//

void testPhaseOne(ConstrainedProblem& f){
  PhaseOneProblem metaF(f);

  arr x;
  x = ARRAY(1., 1., 10.);

  testConstraint(metaF.f_phaseOne, x, 1);
  //one iteration of phase one should be enough
  //properly done: check in each step if constraints are fulfilled and exit phase one then
  //no need to really minimize

  x=x.sub(0,-2);
  testConstraint(f, x);
}


//==============================================================================

void testGaussNewton(VectorFunction& F){
  uint d=2;
  arr x(d),x0;
  rnd.clockSeed();
  for(uint k=0;k<3;k++){
    rndUniform(x, -1., 1.);
    x0=x;
    cout <<"x0=" <<x0 <<endl;
    checkJacobian(F, x, 1e-4);

    optNewton(x, Convert(F), OPT(verbose=2, stopTolerance=1e-3));
    cout <<"x_opt=" <<x <<endl;
    gnuplot("load 'plt'", false, true);
    MT::wait();
  }
}


//==============================================================================

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);

  cout <<USE <<endl;

  enum TestType { unconstrained=1, constrained, phaseOne, gaussNewton };

  switch((TestType)MT::getParameter<int>("exercise")){
  case unconstrained: {
    displayFunction(ChoiceFunction());
    MT::wait();
    testGradDescent(ChoiceFunction());
  } break;
  case constrained: {
    ChoiceConstraintFunction F;
    testConstraint(F);
  } break;
  case phaseOne: {
    ChoiceConstraintFunction F;
    testPhaseOne(F);
  } break;
  case gaussNewton: {
    SinusesFunction F;
    displayFunction(Convert(F));
    MT::wait();
    testGaussNewton(F);
  }
  }

  return 0;
}
