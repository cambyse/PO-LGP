#include "kOrderMarkovProblem.h"

void conv_KOrderMarkovFunction::fv(arr& phi, arr& J, const arr& x) {
  //probing dimensionality
  uint T=f->get_T();
  uint k=f->get_k();
  uint n=f->get_n();
  uint M=0;
  for(uint t=0;t<=T-k;t++) M+=f->get_m(t);
  CHECK(x.nd==2 && x.d1==n && x.d0==(T+1),"");
  
  //resizing things:
  phi.resize(M);   phi.setZero();
  if(&J){ J  .resize(M,x.N); J  .setZero(); }
  M=0;
  uint m_t;
  for(uint t=0;t<=T-k;t++){
    m_t = f->get_m(t);
    arr phi_t,J_t;
    f->phi_t(phi_t, (&J?J_t:NoArr), t, x.subRange(t, t+k) );
    CHECK(phi_t.N==m_t,"");
    phi.setVectorBlock(phi_t, M);
    if(&J){
      if(J_t.nd==3) J_t.reshape(J_t.d0,J_t.d1*J_t.d2);
      CHECK(J_t.d0==m_t && J_t.d1==(k+1)*n,"");
      J.setMatrixBlock(J_t, M, t*n);
    }
    M += m_t;
  }
}
