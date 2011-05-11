#include "soc_inverseKinematics.h"

/*! \brief compute a single control step from current state to target of time t.
    qv=output, qv_1=state at time t-1 */
void soc::bayesianDynamicControl(SocSystemAbstraction& sys, arr& qv, const arr& qv_1, uint t, arr *v, arr *Vinv){
  CHECK(sys.dynamic,"assumed dynamic SOC abstraction");
  uint n=sys.qDim();
  
  //-- access necessary information
  arr A,a,B,tB;
  sys.getProcess(A,a,B,t);
  transpose(tB,B);
  
  arr Q,H,Hinv;
  sys.getQ(Q,t);
  sys.getH(H,t);
  inverse_SymPosDef(Hinv,H);
  
  //fwd message
  arr s(2*n),S(2*n,2*n),Sinv(2*n,2*n);
  S = Q;
  S += B*Hinv*tB;
  s = a + A*qv_1;
  inverse_SymPosDef(Sinv,S);

  //task message
  arr R,r,q_1;
  q_1.referToSubRange(qv_1,0,n-1);
  sys.getCosts(R,r,q_1,t);

  //v,Vinv are optional bwd messages!
  
  //belief
  arr Binv,b;
  if(!v){
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }else{
    if(v->N==qv.N){ //bwd msg given as fully dynamic
      Binv = Sinv + (*Vinv) + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + (*Vinv)*(*v) + r);
    }else{
      arr _Vinv(2*n,2*n),_v(2*n);
      _Vinv.setZero();  _Vinv.setMatrixBlock(*Vinv,0,0);
      _v   .setZero();  _v   .setVectorBlock(*v,0);
      Binv = Sinv + _Vinv + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + _Vinv*_v + r);
    }
  }
  
  qv=b;
}
