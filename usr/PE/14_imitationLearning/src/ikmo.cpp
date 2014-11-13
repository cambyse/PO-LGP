#include "ikmo.h"

CostWeight::CostWeight(WeightType _type, uint _numParam, arr _fixedParam, uint _T, uint _nY, arr _limits) {
  type=_type;
  numParam=_numParam;
  fixedParam=_fixedParam;
  limits=_limits;
  T=_T+1;
  nY=_nY;
}

void CostWeight::compWeights(arr &w, arr &dw, arr &Hw, const arr &param, bool optPrec){
  uint D = optPrec ? 1 : nY;

  switch(type) {
    case Dirac:
      // param: [weight]
      w = ARR(param(0));
      if (&dw){
        dw = ARR(1.);
        Hw = ARR(0.);
        dw = repmat(dw,1,D); dw.flatten();
        Hw = repmat(Hw,1,D);Hw.flatten();
      }
      break;
    case Constant: case Transition: {
      // param: [weight]
      w = ones(T,1)*param(0);
      if (&dw) {
        dw = ones(T,1);
        Hw = zeros(T,1);
        dw = repmat(dw,1,D); dw.flatten();
        Hw = repmat(Hw,1,D);Hw.flatten();
      }
      break;
    }
    case Gaussian: {
      // param: [weight,mean,std] (mean and std can be fixed if numParam is 1 or 2)
      GaussianCosts gc;
      arr t = linspace(0,T-1,T-1);
      if (numParam==1){
        gc.w = param(0);
        gc.mu = fixedParam(0);
        gc.std = fixedParam(1);
        gc.f(t,w);
        if(&dw){
          gc.dfdw(t,dw,Hw);
          dw = repmat(dw,1,D); dw.flatten();
          Hw = repmat(Hw,1,D); Hw.flatten();
        }
      }else if (numParam==2){
        gc.w = param(0);
        gc.mu = param(1);
        gc.std = fixedParam(0);
        gc.f(t,w);
        arr g_w,g_mu,H_w,H_mu,H_w_mu;
        if (&dw){
          //gradient
          gc.dfdw(t,g_w,H_w);
          gc.dfdmu(t,g_mu,H_mu);
          dw.append(g_w);
          dw.append(g_mu);
          dw = repmat(dw,1,D); dw.flatten();
          //Hessian
          gc.dfdwdmu(t,H_w_mu);
          H_w = repmat(H_w,1,D); H_w.flatten();
          H_mu = repmat(H_mu,1,D); H_mu.flatten();
          H_w_mu = repmat(H_w_mu,1,D); H_w_mu.flatten();
          Hw.append(catCol(H_w,H_w_mu));
          Hw.append(catCol(H_w_mu,H_mu));
        }
      } else if (numParam==3){
        gc.w = param(0);
        gc.mu = param(1);
        gc.std = param(2);
        gc.f(t,w);
        if (&dw){
          arr g_w,g_mu,g_std,H_w,H_mu,H_std,H_w_mu,H_w_std,H_mu_std;
          gc.dfdw(t,g_w,H_w);
          gc.dfdmu(t,g_mu,H_mu);
          gc.dfdstd(t,g_std,H_std);
          gc.dfdmudstd(t,H_mu_std);
          gc.dfdwdstd(t,H_w_std);
          gc.dfdwdmu(t,H_w_mu);
          // gradient
          dw.append(g_w);
          dw.append(g_mu);
          dw.append(g_std);
          dw = repmat(dw,1,D); dw.flatten();
          //Hessian
          H_w = repmat(H_w,1,D); H_w.flatten();
          H_mu = repmat(H_mu,1,D); H_mu.flatten();
          H_std = repmat(H_std,1,D); H_std.flatten();
          H_w_mu = repmat(H_w_mu,1,D); H_w_mu.flatten();
          H_w_std = repmat(H_w_std,1,D); H_w_std.flatten();
          H_mu_std = repmat(H_mu_std,1,D); H_mu_std.flatten();
          Hw.append(catCol(H_w,H_w_mu,H_w_std));
          Hw.append(catCol(H_w_mu,H_mu,H_mu_std));
          Hw.append(catCol(H_w_std,H_mu_std,H_std));
        }
      }
      break;
    }
    case RBF:
      RBFCosts rc;
      rc.W = param;
      rc.nB = fixedParam(0);
      //      rc.M = linspace(0.,T,rc.nB-1); rc.M.flatten();
      rc.M = linspace(fixedParam(1),fixedParam(2),rc.nB-1); rc.M.flatten();
      rc.C = repmat(ARR(fixedParam(3)),rc.nB,1); rc.C.flatten();
      arr t = linspace(0,T-1,T-1);
      rc.f(t,w);
      if (&dw) {
        rc.dfdM(t,dw,Hw);
        dw = repmat(dw,1,D); dw.flatten();
        Hw = repmat(Hw,1,D); Hw.flatten();
      }
      break;
  }

  w = repmat(w,1,D); w.flatten();
}

void CostWeight::compConstraints(arr &gL, arr &gU, arr &gS, arr &JgL, arr &JgU, arr &JgS, const arr &param) {
  // upper and lower limit for weight
  gL = ARR(limits(0)-param(0));
  gU = ARR(param(0)-limits(1));
  gS = ARR(param(0)*param(0));

  if (&JgL){
    JgL = ARR(-1.);
    JgU = ARR(1.);
    JgS = ARR(-2.*param(0));
  }
  switch(type) {
    case Gaussian:
      if (numParam > 1){
        // mean should be between 0 and T
        gL.append(ARR(-param(1)));
        gU.append(ARR(param(1)-T));
        if (&JgL){
          JgL.append(ARR(-1.));
          JgU.append(ARR(1.));
          JgS.append(ARR(0.));
        }
      }
      if (numParam > 2){
        // std should be between 0.02 and 0.5*T
        gL.append(ARR(0.02-param(2)));
        gU.append(ARR(param(2)-round(T*0.5)));
        if (&JgL){
          JgL.append(ARR(-1.));
          JgU.append(ARR(1.));
          JgS.append(ARR(0.));
        }
      }
      break;
    case Constant:
      // TODO: should not become too high
      break;
    case Transition:
      // TODO: should not become 0, should not become too high
      break;
    case RBF:
      //
      for (uint i = 1;i<numParam;i++) {
        gL.append(ARR(limits(0)-param(i)));
        gU.append(ARR(param(i)-limits(1)));
        gS.append(param(i)*param(i));
        if (&JgL){
          JgL.append(ARR(-1.));
          JgU.append(ARR(1.));
          JgS.append(-2.*param(i));
        }
      }
      break;
  }

}

void Scene::initCosts(uintA &_phi_perm, bool _optConstraintsParam, bool _optNonlinearParam) {
  optConstraintsParam = _optConstraintsParam;
  optNonlinearParam = _optNonlinearParam;

  phi_perm = _phi_perm;

  // precompute some terms
  MotionProblemFunction MPF(*MP);
  ConstrainedProblemMix v = Convert(MPF);

  arr PHI_T, J_T; // total PHI and J
  TermTypeA tt;
  v(PHI_T,J_T,tt,xDem);

  // split up PHI_T and J_T into costs and constraints
  if (!optConstraintsParam) {
    JxP = J_T;
    PHI = PHI_T;
  } else {
    PHI.clear(); JxP.clear(); JgP.clear(); JhP.clear();
    RowShiftedPackedMatrix *J_T_aux = (RowShiftedPackedMatrix*)J_T.aux;

    uintA f;
    tt.findValues(f,sumOfSqrTT);
    uint xN = f.N;
    tt.findValues(f,ineqTT);
    uint gN = f.N;
    tt.findValues(f,eqTT);
    uint hN = f.N;

    uint xC = 0;
    uint gC = 0;
    uint hC = 0;
    RowShiftedPackedMatrix *Jx_aux = auxRowShifted(JxP, xN, J_T.d1, J_T_aux->real_d1);
    RowShiftedPackedMatrix *Jg_aux = auxRowShifted(JgP, gN, J_T.d1, J_T_aux->real_d1);
    RowShiftedPackedMatrix *Jh_aux = auxRowShifted(JhP, hN, J_T.d1, J_T_aux->real_d1);

    for (uint i= 0;i<tt.d0;i++){
      switch (tt(i)) {
        case sumOfSqrTT:
          JxP[xC] = J_T[i];
          Jx_aux->rowShift(xC) = J_T_aux->rowShift(i);
          PHI.append(PHI_T(i));
          xC++;
          break;
        case ineqTT: case eqTT: {
          JgP[gC] = J_T[i];
          Jg_aux->rowShift(gC) = J_T_aux->rowShift(i);
          g.append(PHI_T(i));
          gC++;
          break;
        }
//        case eqTT:
//          JhP[gC] = J_T[i];
//          Jh_aux->rowShift(hC) = J_T_aux->rowShift(i);
//          h.append(PHI_T(i));
//          hC++;
//          break;
      }
    }
  }

  for (uint i =0;i<g.d0;i++){
    lambdaDem.append(g(i)>=0.);
  }
  cout << lambdaDem << endl;

  T = xDem.d0;

  arr J_Jt = comp_A_At(JxP);
  Jx = unpack(JxP); Jx.special = arr::noneST;
  dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_Jt);

  if (lambdaDem.N==0 || max(lambdaDem) == 0.) {
    /// Unconstrained case
    optConstraintsParam = false;
  } else {
    /// Constrained case
    optConstraintsParam = true;
    // reduce Jg to only active part (lambda !=0)
    MT::Array<uint> idx;
    lambdaDem.findValues(idx,0.);
    lambdaDem.removeAllValues(0.);
    Jg = unpack(JgP);
    cout << lambdaDem << endl;
    arr Jgr;
    uint j = 0;
    for (uint i =0;i<Jg.d0;i++) {
      if(!idx.contains(i)) {
        Jgr.append(~Jg[i]);
      }
      if(idx.contains(i)) {
        JgP.delRows(j);
        ((RowShiftedPackedMatrix*)JgP.aux)->rowShift.remove(j);
        j--;
      }
      j++;
    }

    Jg = Jgr;
    Jg_JgtP = comp_A_At(JgP);
    Jg_Jgt = unpack(Jg_JgtP); Jg_Jgt.special = arr::noneST;
    Jgt_JgJgtI_Jg = ~Jg*inverse_SymPosDef(Jg_Jgt)*Jg;
    JgJgtI_Jg_J_dPHI = inverse_SymPosDef(Jg_Jgt)*Jg*~Jx*diag(PHI);
    J_Jgt = Jx*~Jg;
  }

}

double Scene::compCosts(arr &df, arr &Hf, arr &g, arr &Jg, const arr &w, const arr &dw, const arr &Hw) {
  arr PHIw = PHI%w;
  arr JP_PHIw = comp_At_x(JxP,PHIw);
  arr J_Jt_PHIw = comp_A_x(JxP,JP_PHIw);
  arr J_G_Jt_PHIw = J_Jt_PHIw;

  arr JgJgtI_Jg_Jt_PHIw;
  arr lambda;
  if (optConstraintsParam) {
    JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
    J_G_Jt_PHIw = J_G_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);
    lambda = -JgJgtI_Jg_Jt_PHIw;
  }

  arr f = 4.*(~PHIw)*J_G_Jt_PHIw;

  double y = f(0);
  arr h;
  if (&df) {
    h = 8.*(PHI%J_G_Jt_PHIw);
    df = ~h*dw ;
    df.flatten();
    //    df = 2.*~w.subRange(0,1);
  }
  if (&Hf) {
    if (optNonlinearParam) {
      Hf = 8.*~dw*dPHI_J_Jt_dPHI*dw;
      arr tmp = ~h*Hw;
      tmp.reshapeAs(Hf);
      Hf += ~tmp;
    } else {
      if (dWdx_dPHI_J_G_Jt_dPHI_dWdx.N < 1) {
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
    g = 2.*JgJgtI_Jg_Jt_PHIw;
  }
  if (&Jg && optConstraintsParam) {
    Jg = 2.*JgJgtI_Jg_J_dPHI*dw ;
  }
  return y;
}


IKMO::IKMO(MT::Array<Scene> &_scenes, MT::Array<CostWeight> &_weights,uint _nP):
  scenes(_scenes),
  weights(_weights),
  nP(_nP)
{

  ConstrainedProblem::operator=( [this](arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) -> double{
                                 return this->fc(df, Hf, g, Jg, h, Jh, x);} );

nX = scenes(0).MP->world.getJointStateDimension();
nT = scenes(0).MP->T;

/// optimization parameter
optLearnTransParam = MT::getParameter<bool>("optLearnTransParam");
optNormParam = MT::getParameter<bool>("optNormParam");
optNonlinearParam = MT::getParameter<bool>("optNonlinearParam");
optConstraintsParam = MT::getParameter<bool>("optConstraintsParam");

/// precompute matrices for permutation of PHI
uintA cost_counts;
arr counts=zeros(weights.N);
arr Dpdp;
for (uint c=0;c<scenes(0).MP->taskCosts.N;c++) { // task costs;
  if (scenes(0).MP->taskCosts(c)->map.type==sumOfSqrTT){
    cost_counts.append(0.);
    for (uint t=0;t<=nT;t++) {
      uint dim = scenes(0).MP->taskCosts(c)->dim_phi(*scenes(0).world,t);
      cost_counts.last() += dim;
    }
  }
}

// precompute DWdx
for (uint t=0;t<= nT;t++) {
  // add transition cost elements
  //    if (optLearnTransParam){
  //      Dpdp.append(linspace(counts(0),counts(0)+nX-1,nX-1));
  //      counts(0) += nX;
  //    } else {
  //      NIY;
  //    }

  // add task cost elements
  for (uint c=0;c<scenes(0).MP->taskCosts.N;c++) {
    if ( scenes(0).MP->taskCosts(c)->prec.N >t && (scenes(0).MP->taskCosts(c)->prec(t) > 0) && scenes(0).MP->taskCosts(c)->active && scenes(0).MP->taskCosts(c)->map.type==sumOfSqrTT) {
      uint m;
      m = scenes(0).MP->taskCosts(c)->dim_phi(*scenes(0).world,t);
      double b = (c==0)?0.:sum(cost_counts.subRange(0,c-1));
      arr tmp = linspace(counts(c),counts(c)+m-1,m-1);
      if (tmp.N==1) {tmp = 0.;}
      Dpdp.append(b + tmp);
      counts(c) += m;
    }
  }
}

// copy into uint array
Dpdp.flatten();
for (uint i=0;i<Dpdp.d0;i++){
  phi_perm.append(int(Dpdp(i)));
}

numLambda = 0;
// initialize cost functions for demonstrations
for (uint i=0;i<scenes.d0;i++) {
  scenes(i).initCosts(phi_perm,optConstraintsParam,optNonlinearParam);
  if ( scenes(i).optConstraintsParam ) {
    numLambda += scenes(i).lambdaDem.N;
  }
}
}

void IKMO::compWeights(arr &w, arr &dw, arr &Hw, const arr &param){
  uint c = 0;

  for (uint i=0;i<weights.d0;i++) {
    arr wi,gi,Hi;
    uint nPi = weights(i).numParam;

    // compute weight vector; R(O)
    weights(i).compWeights(wi,(&dw)?gi:NoArr,(&Hw)?Hi:NoArr,param.subRange(c,c+nPi-1) );
    w.append(wi);

    uint m = gi.N/double(nPi);

    if (&dw) {
      // compute gradient; R(O x np)
      if (dw.N==0) {
        dw=gi;
      } else {
        if (nPi==1) {
          dw = catCol(dw,zeros(dw.d0,nPi));
          dw.append(catCol(zeros(m,dw.d1-nPi),gi));
        } if (nPi==2) {
          dw = catCol(dw,zeros(dw.d0,nPi));
          dw.append(catCol(zeros(m,dw.d1-nPi),gi.subRange(0,m-1),gi.subRange(m,m+m-1)));
        } if (nPi==3) {
          dw = catCol(dw,zeros(dw.d0,nPi));
          dw.append(catCol(zeros(m,dw.d1-nPi),gi.subRange(0,m-1),gi.subRange(m,m+m-1),gi.subRange(2*m,2*m+m-1)));
        }if (nPi>3) {
          dw = catCol(dw,zeros(dw.d0,nPi));
          arr tmp = zeros(m,dw.d1-nPi);
          for (uint i=0;i<nPi;i++){
            tmp=catCol(tmp,gi.subRange(i*m,i*m+m-1));
          }
          dw.append(tmp);
        }
      }

      // compute Hessian; R(O x np*np)
      if (optNonlinearParam) {
        if (Hw.N==0) {
          Hw = catCol(Hi,zeros(Hi.d0,nP*nP-1));
        } else {
          if (nPi==3) {
            arr tmp = catCol(zeros(m,nP*c+c),Hi.subRange(0,m-1),zeros(m,nP-nPi));
            tmp = catCol(tmp,Hi.subRange(m,m+m-1),zeros(m,nP-nPi),Hi.subRange(2*m,2*m+m-1),zeros(m,(nP+1)*(nP-nPi-c)));
            Hw.append(tmp);
          }

          if (nPi==2) {
            Hw.append(catCol(zeros(m,nP*c+c),Hi.subRange(0,m-1),zeros(m,nP-nPi),Hi.subRange(m,m+m-1),zeros(m,(nP+1)*(nP-nPi-c))));
          }
          if (nPi==1) {
            Hw.append(catCol(zeros(Hi.d0,param.d0*c+c),Hi,zeros(Hi.d0,param.d0*(param.d0-c)-c-1)));
          }
        }
      } else {
        Hw.append(zeros(m,nP*nP-1));
      }
    }
    c += nPi;
  }
  /// permutate weights correctly
  w.permute(phi_perm);
  if (&dw) {
    dw.permuteRows(phi_perm);
    Hw.permuteRows(phi_perm);
  }
}

void IKMO::compParamConstraints(arr &g, arr &Jg, const arr &param) {
  uint c =0;
  arr gU,gL,gS,JgL,JgU,JgS;
  for (uint i =0;i<weights.d0;i++) {
    arr gLi,gUi,JgLi,JgUi,gSi,JgSi;
    uint nPi = weights(i).numParam;
    weights(i).compConstraints(gLi,gUi,gSi, &Jg?JgLi:NoArr, &Jg?JgUi:NoArr,&Jg?JgSi:NoArr, param.subRange(c,c+nPi-1));
    gL.append(gLi);
    gU.append(gUi);
    gS.append(gSi);
    if (&Jg) {
      JgL.append(JgLi);
      JgU.append(JgUi);
      JgS.append(JgSi);
    }
    c += nPi;
  }

  g=gL;
  g.append(gU);
  g.append(1.-sum(gS));
  if (&Jg) {
    Jg=diag(JgL);
    Jg.append(diag(JgU));
    Jg.append(~JgS);
  }
}

void IKMO::setParam(MotionProblem &MP, const arr &param)
{
  //  if (optLearnTransParam) {
  //    CHECK(weights(0).type==CostWeight::Transition, "The first weight should be transition costs!");
  //    MP.H_rate_diag = fabs(param(0));
  //  }

  for (uint c=0;c<MP.taskCosts.N;c++) {
    if (MP.taskCosts(c)->map.type == sumOfSqrTT) {
      arr w;
      weights(c).compWeights(w,NoArr,NoArr,param.subRange(c,c+weights(c).numParam - 1),true);
      if (weights(c).type==CostWeight::Dirac){
        MP.taskCosts(c)->prec(weights(c).fixedParam(0)) = w(0);
      }else {
        MP.taskCosts(c)->prec = fabs(w);
      }
      //      cout << MP.taskCosts(c)->prec << endl;
    }
  }
}

void IKMO::costReport(arr param) {
  cout << "\n############################################################################\n *** IKMO -- CostReport" << endl;
  cout << "Number of parameters: " << nP << endl;
  cout << "Number of time steps: " << nT << endl;

  arr paramRef = scenes(0).paramRef;

  // normalize parameter
  arr paramRefNorm = 1e4*paramRef /sqrt(sumOfSqr(paramRef));
  arr paramNorm = 1e4*param/sqrt(sumOfSqr(param));

  arr gSol;
  double cost = 0.;// fc(gSol,NoArr,NoArr,NoArr,NoArr,NoArr,param);
  cout << "Cost at solution: " << cost << endl;
  cout << "Gradient at solution: " << gSol << endl;


  cout << "\nReference parameter | Learned parameter | Learned parameter (unnormalized)" << endl;
  uint c =0;
  for (uint i=0;i<scenes(0).MP->taskCosts.N;i++) {
    if (scenes(0).MP->taskCosts(i)->map.type==sumOfSqrTT) {
      if (weights(i).numParam>1){
        //        cout << "-- Task " << scenes(0).MP->taskCosts(i)->name << " : " << paramNorm.subRange(c,c+weights(i).numParam-1) << " | \n" << paramRefNorm.subRange(c,c+weights(i).numParam-1) <<  " | \n" << paramRef.subRange(c,c+weights(i).numParam-1) << endl;
      }else {
        cout << "-- Task " << scenes(0).MP->taskCosts(i)->name << " : " << paramRefNorm(c) <<  " | " << paramNorm(c) << " | " <<  param(c) << endl;
      }
      c = c+weights(i).numParam;
    }
  }
  cout << "############################################################################" << endl;

  // plotting
  arr t = linspace(0,scenes(0).MP->T,scenes(0).MP->T);
  c = 0;

  for (uint i=0;i<scenes(0).MP->taskCosts.N;i++) {
    if (scenes(0).MP->taskCosts(i)->map.type==sumOfSqrTT) {

      arr w;
      if (weights(i).type == CostWeight::Dirac) {
        w = zeros(t.d0);
        w(weights(i).fixedParam(0)) = param(c);
      } else {
        weights(i).compWeights(w,NoArr,NoArr,param.subRange(c,c+weights(i).numParam-1),true);
      }
      plotFunctionPoints(t,log(w),scenes(0).MP->taskCosts(i)->name);
      c = c+weights(i).numParam;
    }
  }

  plot();
}
