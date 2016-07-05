#pragma once

#include <Core/array.h>
#include "optimization.h"

struct GraphProblem {
  /// We have 'variableDimensions.N' variables, each with a different dimension 'variableDimensions(i)'.
  /// We have 'featureVariables.N' features, each depends on the tuple/clique 'featureVariables(j)' of variables.
  /// That is, 'featureVariables' is a list of tuples/cliques that defines the hyper graph
  virtual void getStructure(uintA& variableDimensions, uintAA& featureVariables, TermTypeA& featureTypes) = 0;

  /// We require 'x.N == \sum_i variableDimensions(i)'; so x defines the value of all variables
  /// This returns the feature values, types and Jacobians at state x
  /// Only for features of type 'fTT' also a Hessian is returned
  /// Jacobians and Hessians are dense! But only w.r.t. the variables the feature depends on!!
  /// (It is the job of the optimizer to translate this to sparse global Jacobians/Hessians)
  virtual void phi(arr& phi, arrA& J, arrA& H, const arr& x) = 0;

//  bool checkStructure(const arr& x);                 ///< check if Jacobians and Hessians have right dimensions (=clique size)
//  bool checkJacobian(const arr& x, double tolerance); ///< finite differences check of the returned Jacobian at x
//  bool checkHessian(const arr& x, double tolerance);  ///< finite differences check of the returned Hessian at x
};

/// Reduction to an unstructured constrained problem
struct Graph_ConstrainedProblem:ConstrainedProblem{
  GraphProblem& G;
  uintA variableDimensions, varDimIntegral;
  uintAA featureVariables;
  TermTypeA featureTypes;
  arrA J_G, H_G;

  Graph_ConstrainedProblem(GraphProblem& _G);
  void f(arr& phi, arr& J, arr& H, TermTypeA& tt, const arr& x);
};

