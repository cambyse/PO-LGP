#ifndef INVERSE_MOTION_H
#define INVERSE_MOTION_H

#include <Optim/optimization.h>
#include "scene.h"

struct InverseMotionProblem:ConstrainedProblem {
  Scenario scenario;

  /// number of joints,timesteps,parameters
  uint nX,nT,nP;

  /// matrices for the linear weight function
  arr dwC,HwC;

  uintA phi_perm;
  uint numLambda;
  bool optNonlinearParam;

  enum PARAM_INIT {ONES,RAND,VEC};

  InverseMotionProblem(Scenario &_scenario);

  /// set the cost parameter of the motion problem
  arr initParam(PARAM_INIT mode, const arr &param = NoArr);
  void setParam(MotionProblem &MP,const arr &param);
  void compWeights(arr &w, arr &dw, arr &Hw, const arr &param);
  void compParamConstraints(arr &g, arr &Jg, const arr &param);
  void costReport(arr param, arr param0);

  virtual void fc(arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x) {
    arr df,Hf,g,Jg,h,Jh;
    double costs = 0.;

    /// compute weight vector
    arr w;
    compWeights(w,(dwC.N>0)?NoArr:dwC,(HwC.N>0)?NoArr:HwC,x);

    if (&J) { df.resize(nP);df.setZero();}
    if (&H) { Hf.resize(nP,nP);Hf.setZero();}

    /// Set global constraints
    compParamConstraints(g,(&J)?Jg:NoArr,x);

    /// iterate over demonstrations
    for (uint i=0;i<scenario.scenes.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      costs += scenario.scenes(i).compCosts( (&J)?dfi:NoArr, (&H)?Hfi:NoArr, gi, (&J)?Jgi:NoArr, w, dwC, HwC);
      if (&J) {
        df += dfi;
      }
      if (&H) {
        Hf += Hfi;
      }
      /// Set constraints for each demonstration
      g.append(gi);
      if (&J && Jgi.N>0) {
        Jg.append(Jgi);
      }
    }

    if(&phi) { phi.clear(); }
    if(&J) { J.clear(); }
    if(&H) { H.clear(); }
    if(&tt) { tt.clear(); }

    phi.append(ARR(costs)); if(&J) J.append(~df); if(&H) H.append(Hf);
    phi.append(g); if(&J) J.append(Jg);
    phi.append(h); if(&J) J.append(Jh);
    if(&tt) {
      tt.append(fTT);
      for (uint i=0;i<g.d0;i++) { tt.append(ineqTT); }
      for (uint i=0;i<h.d0;i++) { tt.append(eqTT); }
    }
  }


};

#endif // INVERSE_MOTION_H
