#ifndef IKMO_H
#define IKMO_H

#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Optim/optimization.h>
#include <Kin/taskMaps.h>
#include <vector>
#include "gaussian_costs.h"
#include "rbf_costs.h"
#include <Plot/plot.h>


struct CostWeight {
  enum WeightType {Transition=0,Block=1,Constant=2,Gaussian=3,RBF=4};
  WeightType type;
  uint numParam;  // number of learned parameter
  arr fixedParam; // fixed parameter
  arr limits;     // lower and upper limit for parameter (will be included as constraint)
  uint T;         // number of time steps
  uint nY;        // size of the task map \phi

  CostWeight() { };
  CostWeight(WeightType _type, uint _numParam, arr _fixedParam, uint _T, uint _nY, arr _limits=ARR(0,1e5));
  void compWeights(arr &w, arr &dw, arr &Hw, const arr &param, bool optPrec=false);

  /// compute for each parameter an upper and a lower constraint (gL,gU)
  /// also compute the some of square (gSq) for the constraints that prevents w=0
  void compConstraints(arr &gL, arr &gU, arr &gS, arr &JgL, arr &JgU, arr &JgS, const arr &param);
};

struct Scene {
  /// options
  bool optConstraintsParam;
  bool optNonlinearParam;
  bool constraintsActive;
  arr bretlM;
  arr bretlrh;

  /// scene description
  KOMO* MP;
  mlr::KinematicWorld* world;

  /// reference solutions
  arr lambdaRef;
  arr paramRef;

  arr O;

  /// demonstrations
  arr xDem;
  arr lambdaDem;

  arr lambda;
  uintA phi_perm;
  arr xInit; // init solution for optimization
  uint contactTime;
  uint T;

  arr JxP,JgP,JhP; // packed jacobians
  arr Jx,Jg; // unpacked jacobians

  // task vars
  arr J_Jt,PHI;
  // constrain vars
  arr g, Jg_Jgt,Jg_JgtP;
  arr h;
  arr J_Jgt;
  arr dWdx_dPHI_J_G_Jt_dPHI_dWdx;
  arr Jgt_JgJgtI_Jg;
  arr dPHI_J_Jt_dPHI;
  arr JgJgtI_Jg_J_dPHI;

  //  Scene(arr& _xDem, uintA& _phi_perm, uint _numParam);
  void initCosts(uintA &_phi_perm, bool _optConstraintsParam, bool _optNonlinearParam);

  double compCosts(arr& df, arr& Hf,arr& g, arr& Jg, const arr& w, const arr &dw, const arr &Hw);
};

struct IKMO:ConstrainedProblem {
  mlr::Array<Scene> scenes;
  mlr::Array<CostWeight> weights;
  uint nX,nT,nP; // number of joints,timesteps,parameters
  uintA phi_perm;
  uint numLambda;
  double costScale;

  // optimization options
  bool optLearnTransParam;
  bool optNormParam;
  bool optNonlinearParam;
  bool optConstraintsParam;
  bool initBretl;
  /// constant matrices for the linear weight function
  arr dwC,HwC;

  IKMO(mlr::Array<Scene> &_scenes, mlr::Array<CostWeight> &_weights,uint _nP,double _costScale);

  /// set the cost parameter of the motion problem
  void setParam(KOMO &MP,const arr &param);
  void compWeights(arr &w, arr &dw, arr &Hw, const arr &param);
  void compParamConstraints(arr &g, arr &Jg, const arr &param);
  void costReport(arr param, arr param0);

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, arr& h, arr& Jh, const arr& x) {
    double costs = 0.;



    /// compute weight vector
    arr w;
    if (optNonlinearParam) {
      dwC.clear();HwC.clear();
      compWeights(w,dwC,HwC,x);

    } else {
      // compute Hw and dw only once if there are no nonlinear parameter
      compWeights(w,(dwC.N>0)?NoArr:dwC,(HwC.N>0)?NoArr:HwC,x);

    }

    if (!initBretl){
      initBretl=true;
      uint numL = numLambda;// = scenes.last().Jg.d0;
      dwC = catCol(dwC,zeros(dwC.d0,numL));
      dwC.append(catCol(zeros(numL,dwC.d1-numL),eye(numL)));


    }

    w.append(x.subRange(weights.d0,x.d0-1));



    if (&df) { df.resize(nP);df.setZero();}
    if (&Hf) { Hf.resize(nP,nP);Hf.setZero();}

    /// Set global constraints
    if (&g) {
//      g.clear();
//      compParamConstraints(g,Jg,x);
    }

    /// iterate over demonstrations
    for (uint i=0;i<scenes.d0;i++) {
      arr dfi,Hfi,gi,Jgi;
      costs += scenes(i).compCosts(dfi, Hfi, &g?gi:NoArr, &Jg?Jgi:NoArr, w, dwC, HwC);
      if (&df) {
        df += dfi;
      }
      if (&Hf) {
        Hf += Hfi;
      }
      /// Set constraints for each demonstration
      if (&g) {
//        g.append(gi);
      }
      if (&Jg && Jgi.N>0) {
//        Jg.append(Jgi);
      }
    }
    return costs;
  }

};

#endif // IKMO_H
