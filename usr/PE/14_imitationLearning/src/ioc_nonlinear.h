#ifndef IOC_H
#define IOC_H

#include <Ors/ors.h>
#include <Optim/benchmarks.h>
#include <Motion/motion.h>
#include <Optim/optimization.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <Motion/feedbackControl.h>
#include <vector>
#include <future>
#include <GL/glu.h>
#include <Gui/opengl.h>

#include "gaussian_costs.h"

//#include "scene.h"

struct CostWeight {
  enum WeightType {Dirac=0,Constant=1,Gaussian=2};
  WeightType type;
  uint numParam;  // number of learned parameter
  arr fixedParam; // fixed parameter
  uint T;         // number of time steps
  arr limits;     // lower and upper limit for parameter (will be included as constraint)
  uint D;         // size of the task map \phi

  CostWeight(WeightType _type, uint _numParam, arr _fixedParam, uint _T, uint _D) {
    type=_type;
    numParam=_numParam;
    fixedParam=_fixedParam;
    T=_T;
    D=_D;
  }

  void getWeight(arr &w, arr &g, arr &H, const arr &param){
    switch(type) {
      case Dirac:
        // param: [time[f],weight]
        w = zeros(T,1);
        w[fixedParam(0)] = param(0);
        g = zeros(T,1);
        g[fixedParam(0)] = 1;
        H = zeros(T,1);
        H = repmat(H,1,D);
        H.flatten();
        break;
      case Constant:
        // param: [weight]
        w = ones(T,1)*param(0);
        g = ones(T,1);
        H = zeros(T,1);
        H = repmat(H,1,D);
        H.flatten();
        break;
      case Gaussian:
        // param: [weight,mean,std] (mean and std can be fixed if numParam is 1 or 2)
        GaussianCosts gc;
        arr t = linspace(0,T-1,T-1);
        if (numParam==1){
          gc.w = param(0);
          gc.mu = fixedParam(0);
          gc.std = fixedParam(1);
          gc.f(t,w);
          gc.dfdw(t,g,H);

        }else if (numParam==2){
          gc.w = param(0);
          gc.mu = param(1);
          gc.std = fixedParam(0);
          gc.f(t,w);
          arr g_w,g_mu,H_w,H_mu,H_w_mu;
          gc.dfdw(t,g_w,H_w);
          gc.dfdmu(t,g_mu,H_mu);
          //gradient
          g.append(g_w);
          g.append(g_mu);
          //Hessian
          gc.dfdwdmu(t,H_w_mu);
          H_w = repmat(H_w,1,D); H_w.flatten();
          H_mu = repmat(H_mu,1,D); H_mu.flatten();
          H_w_mu = repmat(H_w_mu,1,D); H_w_mu.flatten();
          H.append(catCol(H_w,H_w_mu));
          H.append(catCol(H_w_mu,H_mu));
        }else if (numParam==3){
          gc.w = param(0);
          gc.mu = param(1);
          gc.std = param(2);
          gc.f(t,w);
          arr g_w,g_mu,g_std,H_w,H_mu,H_std,H_w_mu,H_w_std,H_mu_std;
          gc.dfdw(t,g_w,H_w);
          gc.dfdmu(t,g_mu,H_mu);
          gc.dfdstd(t,g_std,H_std);
          gc.dfdmudstd(t,H_mu_std);
          gc.dfdwdstd(t,H_w_std);
          gc.dfdwdmu(t,H_w_mu);
          // gradient
          g.append(g_w);
          g.append(g_mu);
          g.append(g_std);
          //Hessian
          H_w = repmat(H_w,1,D); H_w.flatten();
          H_mu = repmat(H_mu,1,D); H_mu.flatten();
          H_std = repmat(H_std,1,D); H_std.flatten();
          H_w_mu = repmat(H_w_mu,1,D); H_w_mu.flatten();
          H_w_std = repmat(H_w_std,1,D); H_w_std.flatten();
          H_mu_std = repmat(H_mu_std,1,D); H_mu_std.flatten();
          H.append(catCol(H_w,H_w_mu,H_w_std));
          H.append(catCol(H_w_mu,H_mu,H_mu_std));
          H.append(catCol(H_w_std,H_mu_std,H_std));
        }
        break;
    }
    w = repmat(w,1,D);
    g = repmat(g,1,D);
    w.flatten();
    g.flatten();
  }
};

/*struct IKMO {
  MT::Array<CostWeight*> weights;
  MT::Array<Scene> scenes;

  IKMO(){};

  void computeWeights(arr &w, arr &g, arr &H, const arr &param) {
    uint c = 0;
    for (uint i=0;i<weights.d0;i++) {
      arr wi,gi,Hi;
      uint np = weights(i)->numParam;

      // compute weight vector; R(O)
      weights(i)->getWeight(wi,gi,Hi,param.subRange(c,c+np-1) );
      w.append(wi);

      // compute gradient; R(O x np)
      if (g.N==0) {
        g=gi;
      } else {
        g = catCol(g,zeros(g.d0,np));
        for (uint j=0;j<np;j++){
          g.append(catCol(zeros(gi.N/double(np),g.d1-np+j),gi.subRange(j*np,j*np+gi.N/double(np)-1),zeros(gi.N/double(np),np-1-j)));
        }
      }

      // compute Hessian; R(O x np*np)
      if (H.N==0) {
        H = catCol(Hi,zeros(Hi.d0,param.d0*param.d0-1));
      } else {
        if (np==3) {
          H.append(catCol(zeros(Hi.d0/double(np),param.d0*c+c),Hi.subRange(0,Hi.d0/double(np)-1),zeros(Hi.d0/double(np),param.d0*(param.d0-c-1))));
          H.append(catCol(zeros(Hi.d0/double(np),param.d0*(c+1)+c),Hi.subRange(Hi.d0/double(np),Hi.d0/double(np)+Hi.d0/double(np)-1),zeros(Hi.d0/double(np),(param.d0)*(param.d0-c-2))));
          H.append(catCol(zeros(Hi.d0/double(np),param.d0*(c+2)+c),Hi.subRange(2.*Hi.d0/double(np),2*Hi.d0/double(np)+Hi.d0/double(np)-1),zeros(Hi.d0/double(np),(param.d0)*(param.d0-c-3))));
        }
        if (np==2) {
          H.append(catCol(zeros(Hi.d0/double(np),param.d0*c+c),Hi.subRange(0,Hi.d0/double(np)-1),zeros(Hi.d0/double(np),param.d0*(param.d0-c-1))));
          H.append(catCol(zeros(Hi.d0/double(np),param.d0*(c+1)+c),Hi.subRange(Hi.d0/double(np),Hi.d0/double(np)+Hi.d0/double(np)-1),zeros(Hi.d0/double(np),(param.d0)*(param.d0-c-2))));
        }
        if (np==1) {
          H.append(catCol(zeros(Hi.d0,param.d0*c+c),Hi,zeros(Hi.d0,param.d0*(param.d0-c)-c-1)));
        }
      }
      c += weights(i)->numParam;
    }
  }
};
*/
struct IOC_DemoCost {
  arr x0; // Demonstrated joint space
  arr lambda0; // Demonstrated constrains
  arr lambda; // Current lambda estimate
  arr Dwdx; // Matrix mapping x to w
  uintA phi_perm;
  uint numParam; // number of x parameter
  uint T;

  bool constrainsActive; // constrained or unconstrained variant
  bool learnTransitionCosts;

  // task vars
  arr J_Jt,PHI,J,JP;
  // constrain vars
  arr Jg,g, JgP,Jg_Jgt,Jg_JgtP;
  arr J_Jgt;
  arr dWdx_dPHI_J_G_Jt_dPHI_dWdx;
  arr Jgt_JgJgtI_Jg;
  arr dPHI_J_Jt_dPHI;
  arr JgJgtI_Jg_J_dPHI;

  IOC_DemoCost(MotionProblem &_MP,arr &_x0,arr &_lambda0, arr &_Dwdx,uintA &_phi_perm,uint _numParam):x0(_x0),lambda0(_lambda0),Dwdx(_Dwdx),phi_perm(_phi_perm),numParam(_numParam) {
    // precompute some terms
    MotionProblemFunction MPF(_MP);
    ConstrainedProblem & v = Convert(MPF);
    v.fc(NoArr,NoArr,g,JgP,x0);
    cout << g << endl;

    for (uint i =0;i<g.d0;i++){
      lambda0(i) = g(i)>=0.;
    }
    cout << lambda0 << endl;

    MT::timerStart();

    PHI = v.y;
    JP = v.Jy;

    T = x0.d0;

    // permute PHI an JP such that they are ordered by cost type first and then by time
    PHI.permuteInv(phi_perm);
    JP.permuteRowsInv(phi_perm);

    arr J_JtP = comp_A_At(JP);
    J = unpack(JP); J.special = arr::noneST;
    dPHI_J_Jt_dPHI = (PHI*~PHI)%unpack(J_JtP);

    if (lambda0.N==0 || max(lambda0) == 0.) {
      /// Unconstrained case
      constrainsActive = false;
      dWdx_dPHI_J_G_Jt_dPHI_dWdx = ~Dwdx*dPHI_J_Jt_dPHI*Dwdx;
      //    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;
    } else {
      /// Constrained case
      constrainsActive = true;
      // reduce Jg to only active part (lambda !=0)
      MT::Array<uint> idx;
      lambda0.findValues(idx,0.);
      lambda0.removeAllValues(0.);
      Jg = unpack(JgP);
      cout << lambda0 << endl;
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
      //      dWdx_dPHI_J_G_Jt_dPHI_dWdx = dPHI_J_Jt_dPHI - ( (PHI*~PHI)%(J*Jgt_JgJgtI_Jg*~J));
      arr tmp = ~J*(diag(PHI)*Dwdx);
      dWdx_dPHI_J_G_Jt_dPHI_dWdx = ~Dwdx*dPHI_J_Jt_dPHI*Dwdx - (~tmp*Jgt_JgJgtI_Jg*tmp);
      JgJgtI_Jg_J_dPHI = inverse_SymPosDef(Jg_Jgt)*Jg*~J*diag(PHI);
      J_Jgt = J*~Jg;
    }

    //    dWdx_dPHI_J_G_Jt_dPHI_dWdx =  ~Dwdx*dWdx_dPHI_J_G_Jt_dPHI_dWdx*Dwdx;
  }

  void compute_weights(arr& w,arr& dwdx, arr& dwdxdx,const arr& x){
    w=ones(T*x0.d1,1)*x(0);

    w.flatten();

    //    cout << w << endl;
    GaussianCosts gc;
    gc.mu = 10.;
    gc.std = x(1);
    gc.w = 1.;
    arr y;
    arr t = linspace(0,T-1,T-1);
    gc.f(t,y);

    y = repmat(y,1,3);
    y.flatten();
    //    cout << "y" << y << endl;
    w.append(y);
    //    cout << "w " << w << endl;

    arr dy,Hy;
    gc.dfdstd(t,dy,Hy);
    dy = repmat(dy,1,3);
    dy.flatten();
    Hy = repmat(Hy,1,3);
    Hy.flatten();
    //    cout << dy << endl;


    //    dwdx = catCol(eye(T*x0.d1),zeros(T*x0.d1,y.d0));
    //    arr dwdx2 =  catCol(zeros(y.d0,T*x0.d1),diag(dy) ) ;
    //    dwdx.append(dwdx2);

    dwdx = catCol(ones(T*x0.d1,1),zeros(T*x0.d1,1));
    arr dwdx2 =  catCol(zeros(y.d0,1),dy ) ;
    dwdx.append(dwdx2);

    dwdxdx = catCol(zeros(T*x0.d1,1),zeros(T*x0.d1,1),zeros(T*x0.d1,1),zeros(T*x0.d1,1));
    arr tmp = catCol(zeros(Hy.d0,1),zeros(Hy.d0,1),zeros(Hy.d0,1),Hy );
    dwdxdx.append(tmp);
    //    cout << "dwdxdx: " << dwdxdx << endl;
    // hardcoded parameter, 1: transition costs, 2: position as gaussian

  }

  double eval(arr& df, arr& Hf,arr& gLambda, arr& JgLambda, const arr& x) {
    // compute w vector
    arr augx = x;
    if (!learnTransitionCosts) {
      augx(0) = 0.;
    }

    arr w,dwdx,dwdxdx;
    compute_weights(w,dwdx,dwdxdx,x);
    arr PHIw = PHI%w;

    arr JP_PHIw = comp_At_x(JP,PHIw);
    arr J_Jt_PHIw = comp_A_x(JP,JP_PHIw);
    arr J_G_Jt_PHIw = J_Jt_PHIw;

    arr JgJgtI_Jg_Jt_PHIw;
    if (constrainsActive) {
      JgJgtI_Jg_Jt_PHIw = lapack_Ainv_b_sym(Jg_JgtP,comp_A_x(JgP,JP_PHIw));
      J_G_Jt_PHIw = J_G_Jt_PHIw - comp_A_x(J_Jgt,JgJgtI_Jg_Jt_PHIw);
      lambda = -JgJgtI_Jg_Jt_PHIw;
    }

    arr f = 4.*(~PHIw)*J_G_Jt_PHIw;
    double y = f(0);
    arr h;
    if (&df) {
      h = 8.*(PHI%J_G_Jt_PHIw);
      df = ~h*dwdx ;
      df.flatten();
    }
    if (&Hf) {
      Hf = 8.*~dwdx*dPHI_J_Jt_dPHI*dwdx;
      arr tmp = ~h*dwdxdx;
      tmp.reshapeAs(Hf);
      Hf += ~tmp;
    }
    if (&gLambda && constrainsActive) {
      gLambda = 2.*JgJgtI_Jg_Jt_PHIw;
    }
    if (&JgLambda && constrainsActive) {
      JgLambda = 2.*JgJgtI_Jg_J_dPHI ;
    }
    return y;
  }

};



struct Scene {
  arr xRef; // reference solution
  arr lambdaRef; // constraint trajectory
  arr paramRef;
  arr xInit; // init solution for optimization
  uint contactTime;
  MotionProblem* MP;
  ors::KinematicWorld* world;
  IOC_DemoCost* cost;// ioc cost function for this demonstrations

  Scene () { }
};


struct IOC:ConstrainedProblem {
  MT::Array<Scene> scenes;
  arr xOpt;
  uint numParam;
  uint numLambda;
  arr Dwdx;
  arr Dpdp;
  uintA phi_perm;
  uint n;
  uint T;
  bool learnTransitionCosts;
  bool normalizeParam;

  virtual uint dim_x() { return numParam;}
  virtual uint dim_g() {
    if (normalizeParam) return numParam+numLambda+1;
    return numParam+numLambda;
  }
  //    virtual uint dim_g() { return numParam+numLambda+1+7;}
  //  virtual uint dim_g() { return numParam+numLambda;}

  IOC(MT::Array<Scene> &_scenes,uint _numParam):scenes(_scenes),numParam(_numParam) {
    n = scenes(0).MP->world.getJointStateDimension();
    T = scenes(0).MP->T;

    learnTransitionCosts = MT::getParameter<bool>("learnTransitionCost");
    normalizeParam = MT::getParameter<bool>("normalizeParam");

    // comput cost counts;
    uintA cost_counts;
    arr counts=zeros(1+scenes(0).MP->taskCosts.N);

    cost_counts.append(n*(T+1)); // transition costs;
    for (uint c=0;c<scenes(0).MP->taskCosts.N;c++) { // task costs;
      cost_counts.append(0.);
      for (uint t=0;t<= T;t++) {
        uint dim = scenes(0).MP->taskCosts(c)->dim_phi(*scenes(0).world,t);
        cost_counts.last() += dim;
      }
    }

    // precompute DWdx
    for (uint t=0;t<= T;t++) {
      // add transition cost elements
      if (learnTransitionCosts){
        Dpdp.append(linspace(counts(0),counts(0)+n-1,n-1));
        Dwdx.append(catCol(ones(n,1),zeros(n,numParam-1)));
        counts(0) += n;
      } else {
        Dwdx.append(catCol(ones(n,1)*0.,zeros(n,numParam-1)));
      }

      // add task cost elements
      for (uint c=0;c<scenes(0).MP->taskCosts.N;c++) {
        if ( scenes(0).MP->taskCosts(c)->prec.N >t && (scenes(0).MP->taskCosts(c)->prec(t) > 0) && scenes(0).MP->taskCosts(c)->active && !scenes(0).MP->taskCosts(c)->map.constraint) {
          uint m;
          m = scenes(0).MP->taskCosts(c)->dim_phi(*scenes(0).world,t);
          double b = sum(cost_counts.subRange(0,c));
          Dpdp.append(b + linspace(counts(c+1),counts(c+1)+m-1,m-1));
          counts(c+1) += m;
          arr tmp = zeros(m,1);
          tmp = catCol(tmp,zeros(m,c));
          tmp = catCol(tmp,ones(m,1));
          tmp = catCol(tmp,zeros(m,numParam-tmp.d1));
          Dwdx.append(tmp);
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
      scenes(i).cost = new IOC_DemoCost(*scenes(i).MP,scenes(i).xRef,scenes(i).lambdaRef,Dwdx,phi_perm,numParam);
      scenes(i).cost->learnTransitionCosts = learnTransitionCosts;
      if ( scenes(i).cost->constrainsActive )
        numLambda += scenes(i).cost->lambda0.N;
    }
  }


  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    xOpt = x;

    double y = 0.;
    if (&df) {
      df.resize(numParam);df.setZero();
    }
    if (&Hf) {
      Hf.resize(numParam,numParam);Hf.setZero();
    }

    if (&g) {
      g.clear();
      if (normalizeParam) g.append((sumOfSqr(x)-1e0)*(sumOfSqr(x)-1e0)); // ||w|| = 1

      g.append(-x); // w > 0
      //      g.append(-x.subRange(0,6)+1e-2); // w_trans > 2
      //      g.append(-x.subRange(7,x.d0-1)+1e-2); // w_trans > 2
    }
    if (&Jg) {
      Jg.clear();
      if (normalizeParam) Jg=4.*~x*(sumOfSqr(x)-1e0);
      Jg.append(-eye(numParam));
      //      Jg.append(catCol(-eye(7),zeros(7,numParam-7)));
      //      Jg.append(catCol(zeros(/*7*/),eye(numParam-7),numParam-7)));
    }

    // iterate over demonstrations
    for (uint i=0;i<scenes.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      y += scenes(i).cost->eval(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, x);

      if (&df) {
        df += dfi;
      }
      if (&Hf) {
        Hf += Hfi;
      }
      if (&g) {
        g.append(gi);
      }
      if (&Jg && Jgi.N>0) {
        Jg.append(Jgi*Dwdx);
      }
    }
    return y;
  }

  void costReport(arr &w) {
    xOpt = w;
    cout << "\n*****************************************************" << endl;
    cout << "******************** COST REPORT ********************" << endl;
    cout << "*****************************************************" << endl;
    cout << "Found parameter: " <<xOpt << endl;
    cout << "Found parameter (scaled): " <<xOpt/sqrt(sumOfSqr(xOpt))*1e1 << endl;
    cout << "Reference parameter (scaled): " << scenes(0).paramRef/sqrt(sumOfSqr(scenes(0).paramRef))*1e1 << endl;
    for (uint i=0;i<scenes.d0;i++) {
      cout << "\nDemonstration: " << i << endl;
      cout << "IOC costs: " << scenes(i).cost->eval(NoArr,NoArr,NoArr,NoArr,xOpt) << endl;
      cout << "Lambda: " << scenes(i).cost->lambda/sqrt(sumOfSqr(scenes(i).cost->lambda)) << endl;
      cout << "Lambda Ground Truth: " << scenes(i).cost->lambda0/sqrt(sumOfSqr(scenes(i).cost->lambda0+1e-12)) << "\n\n"<< endl;
    }
  }
};


#endif // IOC_H
