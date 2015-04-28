#include "scene.h"

void Scene::initCosts(bool _optNonlinearParam){
  optNonlinearParam = _optNonlinearParam;

  // precompute some terms
  MotionProblemFunction MPF(*MP);
  ConstrainedProblemMix v = Convert(MPF);

  arr PHI_T, J_T; // total PHI and J
  arr lambdaRef;
  TermTypeA tt;
  v(PHI_T,J_T,tt,xDem);

  // split up PHI_T and J_T into costs and constraints
  if (!optConstraintsParam) {
    JxP = J_T;
    PHI = PHI_T;
  } else {
    PHI.clear(); lambdaRef.clear();JxP.clear(); JgP.clear();
    RowShiftedPackedMatrix *J_T_aux = (RowShiftedPackedMatrix*)J_T.aux;

    uintA f;
    tt.findValues(f,sumOfSqrTT);
    uint xN = f.N;
    tt.findValues(f,ineqTT);
    // count active inequality constraints
    uint gN = 0;
    for (uint i=0;i<f.N;i++){
      if (PHI_T(f(i))>0.) {
        gN++;
      }
    }
    tt.findValues(f,eqTT);
    gN = gN + f.N;

    uint xC = 0;
    uint gC = 0;
    RowShiftedPackedMatrix *Jx_aux = auxRowShifted(JxP, xN, J_T.d1, J_T_aux->real_d1);
    RowShiftedPackedMatrix *Jg_aux = auxRowShifted(JgP, gN, J_T.d1, J_T_aux->real_d1);

    for (uint i= 0;i<tt.d0;i++){
      switch (tt(i)) {
        case sumOfSqrTT:
          JxP[xC] = J_T[i];
          Jx_aux->rowShift(xC) = J_T_aux->rowShift(i);
          PHI.append(PHI_T(i));
          xC++;
          break;
        case ineqTT:
          if (PHI_T(i)>0.){ // include only active inequality constraints
            JgP[gC] = J_T[i];
            Jg_aux->rowShift(gC) = J_T_aux->rowShift(i);
            G.append(PHI_T(i));
            lambdaRef.append(1.);
            gC++;
          }
          break;
        case eqTT:
          JgP[gC] = J_T[i];
          Jg_aux->rowShift(gC) = J_T_aux->rowShift(i);
          G.append(PHI_T(i));
          lambdaRef.append(1.);
          gC++;
          break;
        case noTT:
          HALT("TaskMap of type noTT!");
          break;
      }
    }
  }

//  cout << PHI << endl;
//  cout << JxP << endl;

  arr J_Jt = comp_A_At(JxP);
  Jx = unpack(JxP); Jx.special = arr::noneST;
  dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_Jt);


  if (lambdaRef.N==0 || max(lambdaRef) == 0.) {
    /// Unconstrained case
    optConstraintsParam = false;
  } else {
    /// Constrained case
    optConstraintsParam = true;
    Jg = unpack(JgP); Jg.special = arr::noneST;
    Jg_JgtP = comp_A_At(JgP);
    Jg_Jgt = unpack(Jg_JgtP); Jg_Jgt.special = arr::noneST;
    arr Jg_Jgt_I = lapack_inverseSymPosDef(Jg_Jgt);
    Jgt_JgJgtI_Jg = ~Jg*Jg_Jgt_I*Jg;
    arr tmp = Jg*~Jx;
    J_Jgt = ~tmp;
  }
}

double Scene::compCosts(arr &df, arr &Hf, arr &g, arr &Jg, const arr &w, const arr &dw, const arr &Hw) {
  arr PHIw = PHI%w;
  arr JP_PHIw = comp_At_x(JxP,PHIw);
  arr J_Jt_PHIw = comp_A_x(JxP,JP_PHIw);
  arr J_G_Jt_PHIw = J_Jt_PHIw;

  arr JgJgtI_Jg_Jt_PHIw;

  if (optConstraintsParam) {
    JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
    J_G_Jt_PHIw = J_G_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);
    lambda = -2.*JgJgtI_Jg_Jt_PHIw;
  }

  arr f = 4.*(~PHIw)*J_G_Jt_PHIw;
  double y = f(0);
  arr h;
  if (&df) {
    h = 8.*(PHI%J_G_Jt_PHIw);
//    cout << h << endl;
//    cout << dw << endl;
    df = ~h*dw ;
    df.flatten();
  }
  if (&Hf) {
    if (optNonlinearParam) { // recompute Hessian
      Hf = 8.*~dw*dPHI_J_Jt_dPHI*dw;
      arr tmp = ~h*Hw;
      tmp.reshapeAs(Hf);
      Hf += ~tmp;
    } else {
      if (dWdx_dPHI_J_G_Jt_dPHI_dWdx.N < 1) { // compute Hessian only once
        if (optConstraintsParam) {
          arr tmp = ~Jx*(diag(PHI)*dw);
          dWdx_dPHI_J_G_Jt_dPHI_dWdx = 8.*(~dw*dPHI_J_Jt_dPHI*dw - (~tmp*Jgt_JgJgtI_Jg*tmp));
        }else {
          dWdx_dPHI_J_G_Jt_dPHI_dWdx = 8.*~dw*dPHI_J_Jt_dPHI*dw;
        }
      }
      Hf = dWdx_dPHI_J_G_Jt_dPHI_dWdx;
    }
  }
  if (&g && optConstraintsParam) {
//    g = 2.*JgJgtI_Jg_Jt_PHIw;
  }
  if (&Jg && optConstraintsParam) {
//    Jg = 2.*JgJgtI_Jg_J_dPHI*dw ;
  }
  return y;
}
