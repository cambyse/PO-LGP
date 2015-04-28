#ifndef INVERSE_MOTION_H
#define INVERSE_MOTION_H

#include <Optim/optimization.h>
#include "../src/scene.h"

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

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) {
    double costs = 0.;

    /// compute weight vector
    arr w;
    compWeights(w,(dwC.N>0)?NoArr:dwC,(HwC.N>0)?NoArr:HwC,x);

    if (&df) { df.resize(nP);df.setZero();}
    if (&Hf) { Hf.resize(nP,nP);Hf.setZero();}

    /// Set global constraints
    if (&g) {
      g.clear();
      compParamConstraints(g,Jg,x);
    }

    /// iterate over demonstrations
    for (uint i=0;i<scenario.scenes.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      costs += scenario.scenes(i).compCosts(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, w, dwC, HwC);
      if (&df) {
        df += dfi;
      }
      if (&Hf) {
        Hf += Hfi;
      }
      /// Set constraints for each demonstration
      if (&g) {
        g.append(gi);
      }
      if (&Jg && Jgi.N>0) {
        Jg.append(Jgi);
      }
    }
    return costs;
  }

};

#endif // INVERSE_MOTION_H
