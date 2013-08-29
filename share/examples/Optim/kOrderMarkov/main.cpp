#include <Core/util.h>
#include <Optim/optimization.h>
#include "exampleProblem.h"
#include <Gui/plot.h>

arr buildKernelMatrix(KOrderMarkovFunction& P){
  CHECK(P.hasKernel(),"");
  uint T = P.get_T();
  uint n = P.get_n();
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

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  ParticleAroundWalls P;
  P.k=2;
  P.kern=false; //true;
  
  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uint n=P.get_n();
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check
  arr x(T+1,n);
  for(uint k=0;k<2;k++){
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
  if(P.hasKernel()){
    arr K=buildKernelMatrix(P);
//    createRandom(x, K);  //return 0;
    optGaussNewton(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=1), &K);
  }else{
//    optGaussNewton(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=1));
    optNewton(x, Convert(P), OPT(verbose=2, useAdaptiveDamping=1));
  }

  //analyzeTrajectory(sys, x, true, &cout);
  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
  
  return 0;
}


