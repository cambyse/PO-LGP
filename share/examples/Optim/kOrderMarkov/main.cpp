#include <Core/util.h>
#include <Optim/optimization.h>
#include <Optim/benchmarks.h>

// the kernel stuff is preliminary -- please igore everything related to kernels so far
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
  //see the implemention of ParticleAroundWalls::phi_t for an example on how to specify constrained k-order-Markov optimization problems
  ParticleAroundWalls P;

  //-- print some info on the P
  uint T=P.get_T();
  uint k=P.get_k();
  uint n=P.dim_x();
  cout <<"P parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  //-- gradient check: this is slow!
  arr x(T+1,n);
  for(uint k=0;k<0;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(Convert(P), x, 1e-4);
  }
  
  //-- optimize
  rndUniform(x,-1.,1.);
  arr K;
  if(P.hasKernel()) K = buildKernelMatrix(P);
  if(P.isConstrained()){
    optConstrained(x, NoArr, Convert(P) );
  }else{
    OptNewton opt(x, Convert(P));
    if(K.N) opt.additionalRegularizer=&K;
    opt.run();
  }

  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3", true, true);
}
  
int MAIN(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  testKOrderMarkov();

  return 0;
}


