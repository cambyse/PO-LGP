#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>

template <typename T>
struct DecLagrangianProblem : ScalarFunction {
  T& L;

  //-- parameters of the ADMM
  double mu;         ///< ADMM square penalty
  arr z;             ///< external ADMM reference (global)
  arr lambda;        ///< ADMM lagrange multiplier
  intA var;          ///< indicate where an index of x (local) contributes on z (global)
  intA admmVar;      ///< indices where on x do we have ADMM multipliers
  arr admmMask;      ///< 0 or 1 depending on whether we have an addm mutl at this index

  DecLagrangianProblem(T& L, const arr & z, const intA & _var, const intA & _admmVar, OptOptions opt=NOOPT)
    : L(L)
    , mu(0.0) // first step done with 0 (to avoid fitting to a unset reference)
    , z(z)
    , var(_var)
    , admmVar(_admmVar)
  {
    lambda = zeros(var.d0);
    admmMask = zeros(var.d0);

    for(auto i: admmVar)
    {
      admmMask(i) = 1.0;
    }

    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) const;

  arr deltaZ(const arr& x) const;

  void updateADMM(const arr& x, const arr& z);
};

#include <Optimization/decentralized_lagrangian.tpp>
