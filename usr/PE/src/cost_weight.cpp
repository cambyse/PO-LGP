#include "cost_weight.h"

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
    case Block: {
      // param: [weight] fixedParam: [fromTime (toTime)]
      double fromTime = fixedParam(0);
      if (fixedParam.d0==1) fixedParam.append(ARR(fixedParam(0)));
      double toTime = fixedParam(1);
      w.resize(toTime+1-fromTime).setZero();
      w = param(0);
      if (&dw){
        dw = w/param(0);
        Hw.resizeAs(w).setZero();
        dw = repmat(dw,1,D); dw.flatten();
        Hw = repmat(Hw,1,D);Hw.flatten();
      }
      break;
    }
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
    case RBF: {
      RBFCosts rc;
      rc.W = param;
      rc.nB = fixedParam(0);
      rc.M = linspace(fixedParam(1),fixedParam(2),rc.nB-1); rc.M.flatten();
      rc.C = repmat(ARR(fixedParam(3)),rc.nB,1); rc.C.flatten();
      arr t = linspace(0,T-1,T-1);
      rc.f(t,w);
      if (&dw) {
        rc.dfdM(t,dw,Hw);
        dw = repmat(dw,1,D); dw.flatten();
        Hw = repmat(Hw,1,D); Hw.flatten();
      }
//      rc.plotBasis(t);
      break;
    }
  }

  w = repmat(w,1,D); w.flatten();
}


/// compute for each parameter an upper and a lower constraint (gL,gU)
/// also compute the some of square (gSq) for the constraints that prevents w=0
void CostWeight::compConstraints(arr &gL, arr &gU, arr &gS, arr &JgL, arr &JgU, arr &JgS, const arr &param) {
  gL = ARR(limits(0)-param(0));
  gU = ARR(param(0)-limits(1));
  if (&JgL){
    JgL = ARR(-1.);
    JgU = ARR(1.);
  }

  switch(type) {
    case Block:
      gS = ARR(param(0));
      if (&JgL){
        JgS = ARR(1.);
      }
      break;
    case Gaussian:
      gS = ARR(param(0));
      if (&JgL){
        JgS = ARR(1.);
      }

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
      gS = ARR(param(0));
      if (&JgL){
        JgS = ARR(1.);
      }/*
        gS = ARR(0.);
        if (&JgL){
          JgS = ARR(0.);
        }*/
      break;
    case RBF:
      //
      gS.append(param(0));
      if (&JgL){
        JgS.append(1.);
      }
      for (uint i = 1;i<numParam;i++) {
        gL.append(ARR(limits(0)-param(i)));
        gU.append(ARR(param(i)-limits(1)));
        gS.append(param(i));
        if (&JgL){
          JgL.append(ARR(-1.));
          JgU.append(ARR(1.));
          JgS.append(1.);
        }
      }
      break;
  }
}

void RBFCosts::f(const arr &x, arr &y) {
  y = zeros(x.d0);
  for (uint i = 0;i<nB; i++) {
    y = y+W(i)*exp(-(x-M(i))%(x-M(i))/(2*pow(C(i),2)));
  }
}

void RBFCosts::dfdM(const arr &x, arr &g, arr &H) {
  g.clear();
  for (uint i = 0;i<nB; i++) {
    g.append(exp(-(x-M(i))%(x-M(i))/(2*pow(C(i),2))));
  }

  if (&H) {
    H = zeros(x.d0,nB*nB);
  }
}

void RBFCosts::plotBasis(const arr &x) {
  arr y = zeros(x.d0);
  plotClear();
  for (uint i = 0;i<nB; i++) {
    y = exp(-(x-M(i))%(x-M(i))/(2*pow(C(i),2)));
    plotFunction(y);
//    plotPoints(y);
  }
  plot(true);
}



void GaussianCosts::f(const arr& x, arr& y) {
  y = w*exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  y.flatten();
  // remove small values
  for (uint i = 0;i<y.d0;i++) {
    if (y(i)<1e-3) {
      y(i)=0.;
    }
  }
}

void GaussianCosts::dfdmu(const arr& x, arr& g, arr &H) {
  arr y;
  f(x,y);
  g = y%(x-mu)/(pow(std,2));
  if (&H) {
    H = y%((x-mu)%(x-mu)/pow(std,4) - 1./pow(std,2));
    H.flatten();
  }
  g.flatten();
}

void GaussianCosts::dfdw(const arr& x, arr& g, arr &H) {
  g = exp(-(x-mu)%(x-mu)/(2*pow(std,2)));
  if (&H) {
    H = .0*g;
    H.flatten();
  }
  g.flatten();
}

void GaussianCosts::dfdstd(const arr& x, arr& g, arr &H) {
  arr y;
  f(x,y);
  g = y%(x-mu)%(x-mu)/pow(std,3.);
  if (&H) {
    H = y%((x-mu)%(x-mu)%(x-mu)%(x-mu)/pow(std,6.) - 3.*(x-mu)%(x-mu)/pow(std,4.));
    H.flatten();
  }
  g.flatten();
}


void GaussianCosts::dfdwdmu(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))%(x-mu)/pow(std,2);
  H.flatten();
}

void GaussianCosts::dfdwdstd(const arr &x, arr &H)
{
  H = exp(-(x-mu)%(x-mu)/(2*pow(std,2)))% (x-mu)%(x-mu)/pow(std,3);
  H.flatten();
}

void GaussianCosts::dfdmudstd(const arr &x, arr &H)
{
  arr y;
  f(x,y);
  H = y%(x-mu)/pow(std,2)%(x-mu)%(x-mu)/pow(std,3) + -2.*y%(x-mu)/pow(std,3);
  H.flatten();
}
