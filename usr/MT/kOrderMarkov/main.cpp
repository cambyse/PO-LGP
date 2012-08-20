#include <MT/util.h>
#include <MT/optimization.h>
#include "kOrderMarkovProblem.h"
#include "exampleProblem.h"

struct conv_KOrderMarkovFunction:VectorFunction {
  KOrderMarkovFunction *f;
  conv_KOrderMarkovFunction(KOrderMarkovFunction& _f):f(&_f) {}
  void   fv(arr& y, arr& J, const arr& x);
};

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);

  ParticleAroundWalls problem;

  conv_KOrderMarkovFunction P(problem);

  uint T=problem.get_T();
  uint k=problem.get_k();
  uint n=problem.get_n();

  cout <<"Problem parameters:"
       <<"\n T=" <<T
       <<"\n k=" <<k
       <<"\n n=" <<n
       <<endl;

  arr x(T+1,n);
  rndUniform(x,-1.,1.);
  
  for(uint k=0;k<10;k++){
    rndUniform(x,-1.,1.);
    checkJacobian(P, x, 1e-6);
  }
  
  rndUniform(x,-10.,-1.);
  optGaussNewton(x, P, OPT1(verbose=2));

  write(LIST<arr>(x),"z.output");
  gnuplot("plot 'z.output' us 1,'z.output' us 2,'z.output' us 3");
  
  return 0;
}


void conv_KOrderMarkovFunction::fv(arr& phi, arr& J, const arr& x) {
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0;t<=T-k;t++) M+=f->get_m(t);
  CHECK((T+1)*n==x.N,"");

  //resizing things:
  phi.resize(M);   phi.setZero();
  if(&J){ J  .resize(M,x.N); J  .setZero(); }
  M=0;
  uint m_t;
  for(uint t=0;t<=T-k;t++){
    m_t = f->get_m(t);
    arr phi_t,J_t;
    f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t, t+k) );
    J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
    phi.setVectorBlock(phi_t, M);
    if(&J) J  .setMatrixBlock(J_t, M, t*n);
    M += m_t;
  }
}
