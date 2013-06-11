/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
/**
 * @file
 * @ingroup group_soc
 */

#include "soc_inverseKinematics.h"
#include "truncatedGaussian.h"

void soc::bayesianIKControl2(SocSystemAbstraction& sys,
                             arr& q, const arr& q_1, uint t, arr *v, arr *Vinv){
  CHECK(!sys.dynamic, "assumed non-dynamic SOC abstraction");
  uint n=sys.qDim();

  //-- access necessary information
  arr W;
  sys.getControlCosts(W, NoArr, t);

  //fwd message
  arr s(n), Sinv(n, n);
  s = q_1;
  Sinv = W;

  //task message
  arr R, r;
  sys.getTaskCosts(R, r, q_1, t);

  //v, Vinv are optional bwd messages!

  //belief
  arr Binv, b;
  if(!v){
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }else{
    Binv = Sinv + (*Vinv) + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + (*Vinv)*(*v) + r);
  }

  //constraints
  arr cdir, coff;
  sys.getConstraints(cdir, coff, q_1, t);
  if(cdir.d0){
    arr __b, __B, __Binv;
    inverse_SymPosDef(__B, Binv);
    __b=b;
    //plotClear();  plotCovariance(__b, __B);
    for(uint i=0; i<cdir.d0; i++){ //one-by-one truncate the constraint from the belief
      TruncateGaussian(__b, __B, cdir[i], coff(i));
      //plotTruncationLine(cdir[i], coff[i]);  plotCovariance(__b, __B);  plot();
    }
    //compute the EP message and 'add' it to the task message
    inverse_SymPosDef(__Binv, __B);
    R += __Binv - Binv;
    r += __Binv * __b - Binv*b;

    //recompute (b, B);
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }
  q=b;
}


/** @brief compute a single control step from current state to target of time t.
    x=output, x_1=state at time t-1 */
void soc::bayesianDynamicControl(SocSystemAbstraction& sys, arr& x, const arr& x_1, uint t, arr *v, arr *Vinv){
  CHECK(sys.dynamic, "assumed dynamic SOC abstraction");
  uint n=sys.qDim();

  //-- access necessary information
  arr A, a, B, tB, Q;
  sys.getDynamics(A, a, B, Q, t);
  transpose(tB, B);

  arr H, Hinv;
  sys.getControlCosts(H, Hinv, t);

  //fwd message
  arr s(2*n), S(2*n, 2*n), Sinv(2*n, 2*n);
  S = Q;
  S += B*Hinv*tB;
  s = a + A* x_1;
  inverse_SymPosDef(Sinv, S);

  //task message
  arr R, r;
  //q_1.referToSubRange(x_1, 0, n-1);
  sys.getTaskCosts(R, r, x_1, t);

  //v, Vinv are optional bwd messages!

  //belief
  arr Binv, b;
  if(!v){
    Binv = Sinv + R;
    lapack_Ainv_b_sym(b, Binv, Sinv*s + r);
  }else{
    if(v->N== x.N){ //bwd msg given as fully dynamic
      Binv = Sinv + (*Vinv) + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + (*Vinv)*(*v) + r);
    }else{
      arr _Vinv(2*n, 2*n), _v(2*n);
      _Vinv.setZero();  _Vinv.setMatrixBlock(*Vinv, 0, 0);
      _v   .setZero();  _v   .setVectorBlock(*v, 0);
      Binv = Sinv + _Vinv + R;
      lapack_Ainv_b_sym(b, Binv, Sinv*s + _Vinv*_v + r);
    }
  }

  x=b;
}
