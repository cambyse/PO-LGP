#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>

struct DecLagrangianProblem : ScalarFunction {
  LagrangianProblem& L;

  //-- parameters of the ADMM
  double mu;         ///< ADMM square penalty
  arr z;             ///< external ADMM reference (global)
  arr lambda;        ///< ADMM lagrange multiplier
  intA var;          ///< where on x does this subproblem contribute

  DecLagrangianProblem(LagrangianProblem&L, const arr & z, const intA & _var, OptOptions opt=NOOPT)
    : L(L)
    , mu(0.0) // first step done with 0 (to avoid fitting to a unset reference)
    , z(z)
    , var(_var)
  {
    lambda = zeros(var.d0);

    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) const;

  arr deltaZ(const arr& x) const;

  void updateADMM(const arr& x, const arr& z);
};
