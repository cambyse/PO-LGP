#ifndef COSTWEIGHT_H
#define COSTWEIGHT_H

#include <Core/array.h>
#include <Gui/plot.h>

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

struct GaussianCosts {
  double mu;
  double std;
  double w;

  GaussianCosts() {};

  void f(const arr &x, arr &y);
  void dfdmu(const arr &x, arr &g, arr& H=NoArr);
  void dfdstd(const arr &x, arr &g, arr& H=NoArr);
  void dfdw(const arr &x, arr &g, arr& H=NoArr);

  void dfdwdmu(const arr &x,arr& H);
  void dfdwdstd(const arr &x,arr& H);
  void dfdmudstd(const arr &x,arr& H);
};

struct RBFCosts {
  uint nB; // number of basis functions

  arr M;  // mean values
  arr C;  // std values
  arr W;  // weight values of basis functions

  RBFCosts() {};

  void f(const arr &x, arr &y);
  void dfdM(const arr &x, arr &g, arr& H=NoArr);
  void plotBasis(const arr &x);
};

#endif // COSTWEIGHTS_H
