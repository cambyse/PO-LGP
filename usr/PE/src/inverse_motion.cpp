#include "inverse_motion.h"
#include <Core/util.h>

#include <Gui/plot.h>

InverseMotionProblem::InverseMotionProblem(Scenario &_scenario):
  scenario(_scenario),
  nP(_scenario.paramGT.d0)
{
//  ConstrainedProblem::operator=( [this](arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) -> double{
//                                 return this->fc(df, Hf, g, Jg, h, Jh, x);} );
  ConstrainedProblem::operator=( [this](arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) -> void {
                                 return this->fc(phi,J,H,tt,x);} );

    nX = scenario.scenes(0).MP->world.getJointStateDimension();
    nT = scenario.scenes(0).MP->T;

    /// optimization parameter
    optNonlinearParam = mlr::getParameter<bool>("IMP/optNonlinearParam");

    /// precompute matrices for permutation of PHI
    uintA cost_counts;
    arr counts=zeros(scenario.weights.N);
    arr Dpdp;
    for (uint c=0;c<scenario.scenes(0).MP->tasks.N;c++) { // task costs;
      if (scenario.scenes(0).MP->tasks(c)->map.type==sumOfSqrTT){
        cost_counts.append(0.);
        for (uint t=0;t<=nT;t++) {
          uint dim = scenario.scenes(0).MP->tasks(c)->dim_phi(*scenario.scenes(0).world,t);
          cost_counts.last() += dim;
        }
      }
    }

    // precompute DWdx
    for (uint t=0;t<= nT;t++) {
      // add task cost elements
      for (uint c=0;c<scenario.scenes(0).MP->tasks.N;c++) {
        if ( scenario.scenes(0).MP->tasks(c)->prec.N >t && (scenario.scenes(0).MP->tasks(c)->prec(t) > 0) && scenario.scenes(0).MP->tasks(c)->active && scenario.scenes(0).MP->tasks(c)->map.type==sumOfSqrTT) {
          uint m;
          m = scenario.scenes(0).MP->tasks(c)->dim_phi(*scenario.scenes(0).world,t);
          double b = (c==0)?0.:sum(cost_counts.subRange(0,c-1));
          arr tmp = linspace(counts(c),counts(c)+m-1,m-1);
          if (tmp.N==1) {tmp = counts(c);}
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
    for (uint i=0;i<scenario.scenes.d0;i++) {
      scenario.scenes(i).initCosts(optNonlinearParam);
      if ( scenario.scenes(i).optConstraintsParam ) {
        numLambda += scenario.scenes(i).lambdaDem.N;
      }
    }
}

void InverseMotionProblem::compWeights(arr &w, arr &dw, arr &Hw, const arr &param){
  uint c = 0;

  for (uint i=0;i<scenario.weights.d0;i++) {
    arr wi,gi,Hi;
    uint nPi = scenario.weights(i).numParam;

    // compute weight vector; R(O)
    scenario.weights(i).compWeights(wi,(&dw)?gi:NoArr,(&Hw)?Hi:NoArr,param.subRange(c,c+nPi-1) );
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

void InverseMotionProblem::compParamConstraints(arr &g, arr &Jg, const arr &param) {
  uint c =0;
  g.clear();
  arr gU,gL,gS,JgL,JgU,JgS;
  for (uint i =0;i<scenario.weights.d0;i++) {
    arr gLi,gUi,JgLi,JgUi,gSi,JgSi;
    uint nPi = scenario.weights(i).numParam;
    scenario.weights(i).compConstraints(gLi,gUi,gSi, &Jg?JgLi:NoArr, &Jg?JgUi:NoArr,&Jg?JgSi:NoArr, param.subRange(c,c+nPi-1));
    gL.append(gLi);
    gS.append(gSi);
    if (&Jg) {
      JgL.append(JgLi);
      JgS.append(JgSi);
    }
    c += nPi;
  }

  g=gL;
//  g.append(scenario.costScale-sum(gS));
  g.append(scenario.costScale-sum(gS%gS));
//  g.append(scenario.costScale-sum(gS%gS%gS%gS));

  if (&Jg) {
    Jg=diag(JgL);
//    Jg.append(-~JgS);
//    arr tmp = catCol(~gS,zeros(1,2));
    Jg.append(-2.*~(gS));
//    Jg.append(-2.*tmp);
//        Jg.append(-4.*~(gS%gS%gS));
  }
}


arr InverseMotionProblem::initParam(PARAM_INIT mode, const arr &param) {
  arr param0;
  switch (mode) {
    case ONES:
      param0 = ones(nP,1);
      break;
    case RAND:
      param0 = fabs(randn(nP,1));
      break;
    case VEC:
      param0 = param;
      break;
  }
//  param0 = param0/sum(param0)*scenario.costScale;
  param0 = param0/sqrt(sum(param0%param0)/scenario.costScale);

  param0.flatten();
  return param0;
}

void InverseMotionProblem::setParam(MotionProblem &MP, const arr &param)
{
  arr paramNorm = scenario.costScale*param/sum(param);

  for (uint c=0;c<MP.tasks.N;c++) {
    if (MP.tasks(c)->map.type == sumOfSqrTT) {
      arr w;
      scenario.weights(c).compWeights(w,NoArr,NoArr,paramNorm.subRange(c,c+scenario.weights(c).numParam - 1),true);
      if (scenario.weights(c).type==CostWeight::Block){
        MP.tasks(c)->prec.subRange(scenario.weights(c).fixedParam(0),scenario.weights(c).fixedParam(1)) = w;//fabs(w(0));
      }else {
        MP.tasks(c)->prec = w;//fabs(w);
      }
    }
  }
}

void InverseMotionProblem::costReport(arr param,arr param0) {
  cout << "\n\n\n################################################################################\n *** InverseMotionProblem -- CostReport" << endl;
  cout << "# of parameters: " << nP << endl;
  cout << "# of time steps: " << nT << endl;
  if (scenario.scenes(0).optConstraintsParam) {
    cout << "# of active constraints: " << scenario.scenes(0).lambda.N << endl;
  }

  arr paramRef = scenario.paramGT;

  // normalize parameter
  arr paramRefNorm = 1e4*paramRef /sqrt(sumOfSqr(paramRef));
  arr paramNorm = 1e4*param/sqrt(sumOfSqr(param));
  arr param0Norm = 1e4*param0/sqrt(sumOfSqr(param0));

  arr gSol,conSol;
  arr cost;
  arr phiSol,JSol,HSol;
  fc(phiSol,JSol,HSol,NoTermTypeA,param);
  cout << "-IOC cost at solution: " << phiSol(0) << endl;
  cout << "-IOC cost gradient at solution: " << JSol[0]  << endl;
  cout << "-IOC constraints at solution: " << phiSol.subRange(1,phiSol.d0-1) << endl;


  cout << "################################################################################" << endl;
  cout << "Learned param | Ref param | Init param | Learned param (norm) | Ref param (norm)" << endl;
  uint c =0;
  for (uint i=0;i<scenario.scenes(0).MP->tasks.N;i++) {
    if (scenario.scenes(0).MP->tasks(i)->map.type==sumOfSqrTT) {
      if (scenario.weights(i).numParam>1){
        //        cout << "-- Task " << scenes(0).MP->tasks(i)->name << " : " << paramNorm.subRange(c,c+weights(i).numParam-1) << " | \n" << paramRefNorm.subRange(c,c+weights(i).numParam-1) <<  " | \n" << paramRef.subRange(c,c+weights(i).numParam-1) << endl;
        cout << "-- Task " << scenario.scenes(0).MP->tasks(i)->name << " : " << param.subRange(c,c+scenario.weights(i).numParam-1) << endl;
      } else {
        cout << "-- Task " << scenario.scenes(0).MP->tasks(i)->name << " : " << param(c) <<" | " << paramRef(c)<<" | " << param0(c) <<" | "<< paramNorm(c) <<  " | " << paramRefNorm(c) << " | " << endl;
      }
      c = c+scenario.weights(i).numParam;
    }
  }
  write(LIST<arr>(paramRefNorm,paramNorm),"ioc.log");
  cout << "################################################################################" << endl;
  cout << "param0 " <<param0 << endl;
  cout << "param " <<param << endl;
  cout << "paramNorm " <<paramNorm << endl;
  cout << "paramRefNorm " <<paramRefNorm << endl;
  cout << "################################################################################" << endl;

  // lambda
  if (scenario.scenes(0).optConstraintsParam){
    for (uint i=0;i<scenario.scenes.d0;i++) {
      // lambda
//      cout << "lambda: "<< scenario.scenes(i).lambda << endl;
//      cout << "lambdaDem: "<< scenario.scenes(i).lambdaDem << endl;
    }
  }
  cout << "################################################################################\n\n" << endl;


  // plotting
  arr t = linspace(0,scenario.scenes(0).MP->T,scenario.scenes(0).MP->T);
  c = 0;

  for (uint i=0;i<scenario.scenes(0).MP->tasks.N;i++) {
    if (scenario.scenes(0).MP->tasks(i)->map.type==sumOfSqrTT) {
      arr w;
      if (scenario.weights(i).type == CostWeight::Block) {
        w = zeros(t.d0);
        w.subRange(scenario.weights(i).fixedParam(0),scenario.weights(i).fixedParam(1)) = param(c);
      } else {
        scenario.weights(i).compWeights(w,NoArr,NoArr,param.subRange(c,c+scenario.weights(i).numParam-1),true);
      }
      plotFunctionPoints(t,log(w+1.));//,scenario.scenes(0).MP->tasks(i)->name);
//      plotFunctionPoints(t,w,scenario.scenes(0).MP->tasks(i)->name);
      c = c+scenario.weights(i).numParam;
    }
  }

  plot();
}
