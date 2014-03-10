#include <Core/util.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>
#include <Optim/constrained.h>
#include <Gui/plot.h>

arr buildKernelMatrix(KOrderMarkovFunction& P){
  CHECK(P.hasKernel(),"");
  uint T = P.get_T();
  uint n = P.dim_x();
  arr K((T+1)*n,(T+1)*n);
  K.setZero();
  for(uint t=0;t<=T;t++){
    for(uint s=t;s<=T;s++){
      double kts=P.kernel(t,s);
      for(uint i=0;i<n;i++){
        K(t*n+i,s*n+i) = kts;
        K(s*n+i,t*n+i) = kts;
      }
    }
  }
  for(uint i=0;i<K.d0;i++) K(i,i) += 1e-10;
  arr Kinv;
  inverse_SymPosDef(Kinv, K);
  return Kinv;
}

void TEST(KOrderMarkov) {
  ParticleAroundWalls P;
  P.k=1;
  P.kern = false; //true;
  P.constrained = true;

  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uint n=P.dim_x();
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check
  arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(P), x, 1e-4);
  }
  
#if 0
  //-- print some example output
  arr phi,J;
  P.fv(phi, J, x);
  cout <<"x=" <<x <<"\nphi=" <<phi <<"\nJ=" <<J <<endl;
  return 0.;
#endif
  
#if 0
  //-- test cost on a simple deterministic trajectory
  for(uint t=0;t<x.d0;t++){ x(t,0) = double(t)/T; x(t,1)=1.; }
  for(uint t=0;t<x.d0;t++){ double tt=double(t)/T;  x(t,1) = 2.*tt; x(t,0) = tt*tt; }
  //analyzeTrajectory(sys, x, true, &cout);
  //return 0;
#endif

  //-- optimize
  rndUniform(x,-10.,-1.);
  arr K;
  if(P.hasKernel()) K = buildKernelMatrix(P);
  if(P.isConstrained()){
    Convert CP(P);
    UnconstrainedProblem UCP(CP);
    UCP.mu=1.;
    for(uint k=0;k<10;k++){
//      checkJacobian(CP, x, 1e-4);
//      checkAll(CP, x, 1e-4);
      //checkGradient(UCP, x, 1e-4);
      cout <<" mu=" <<UCP.mu <<" lambda=" <<UCP.lambda <<endl;
      //optNewton(x, UCP, OPT(verbose=2, useAdaptiveDamping=false, damping=1., stopIters=20), (K.N? &K : NULL));
      OptNewton opt(x, UCP, OPT(verbose=2,  useAdaptiveDamping=false, damping=1., stopIters=20));
      if(K.N) opt.additionalRegularizer=&K;
      opt.run();

      UCP.augmentedLagrangian_LambdaUpdate(x);
//      UCP.mu *= 10;
      write(LIST<arr>(x),"z.output");
      gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", false, true);
      MT::wait();
    }
  }else{
    OptNewton opt(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=true));
    if(K.N) opt.additionalRegularizer=&K;
    opt.run();
  }

  //analyzeTrajectory(sys, x, true, &cout);
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
}
  
int MAIN(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  testKOrderMarkov();

  return 0;
}


