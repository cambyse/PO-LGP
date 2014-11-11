#ifndef IKMO_H
#define IKMO_H

#include <Ors/ors.h>
#include <Motion/motion.h>
#include <Optim/optimization.h>
#include <Motion/taskMap_default.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_constrained.h>
#include <vector>
#include "gaussian_costs.h"
#include "rbf_costs.h"
#include <Gui/plot.h>


struct CostWeight {
  enum WeightType {Transition=0,Dirac=1,Constant=2,Gaussian=3,RBF=4};
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

  /// scene description
  MotionProblem* MP;
  ors::KinematicWorld* world;

  /// reference solutions
  arr lambdaRef;
  arr paramRef;

  /// demonstrations
  arr xDem;
  arr lambdaDem;

  uintA phi_perm;
  arr xInit; // init solution for optimization
  uint contactTime;
  uint T;

  // task vars
  arr J_Jt,PHI,J,JP;
  // constrain vars
  arr Jg,g, JgP,Jg_Jgt,Jg_JgtP;
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
  MT::Array<Scene> scenes;
  MT::Array<CostWeight> weights;
  uint nX,nT,nP; // number of joints,timesteps,parameters
  uintA phi_perm;
  uint numLambda;

  // optimization options
  bool optLearnTransParam;
  bool optNormParam;
  bool optNonlinearParam;
  bool optConstraintsParam;

  /// constant matrices for the linear weight function
  arr dwC,HwC;

  IKMO(MT::Array<Scene> &_scenes, MT::Array<CostWeight> &_weights,uint _nP);

  /// set the cost parameter of the motion problem
  void setParam(MotionProblem &MP,const arr &param);
  void compWeights(arr &w, arr &dw, arr &Hw, const arr &param);
  void compParamConstraints(arr &g, arr &Jg, const arr &param);
  void costReport(arr param);

  virtual uint dim_x() {return nP;}

  // param limits + norm constraint + num active cons
  virtual uint dim_g() {return 2*nP+1+numLambda;}

  virtual double fc(arr& df, arr& Hf, arr& g, arr& Jg, const arr& x) {
    double costs = 0.;
    /// compute weight vector
    arr w;
    if (optNonlinearParam) {
      dwC.clear();HwC.clear();
      compWeights(w,dwC,HwC,x);
    } else {
      // compute Hw and dw once if there are only nonlinear parameter
      compWeights(w,(dwC.N>0)?NoArr:dwC,(HwC.N>0)?NoArr:HwC,x);
    }


    if (&df) { df.resize(nP);df.setZero();}
    if (&Hf) { Hf.resize(nP,nP);Hf.setZero();}

    /// Set global constraints
    if (&g) {
      g.clear();
      compParamConstraints(g,Jg,x);
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
        g.append(gi);
      }
      if (&Jg && Jgi.N>0) {
        Jg.append(Jgi);
      }
    }
    return costs;
  }

};

#endif // IKMO_H
