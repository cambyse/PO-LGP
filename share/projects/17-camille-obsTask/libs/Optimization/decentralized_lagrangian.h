#pragma once

#include <Optim/newton.h>
#include <Optim/constrained.h>

struct DecLagrangianProblem : ScalarFunction {
  LagrangianProblem& L;

  //-- parameters of the ADMM
  double mu;         ///< ADMM square penalty
  arr lambda;        ///< ADMM lagrange multiplier
  arr z;             ///< external ADMM reference
  const arr& xmask;  ///< where on x does this subproblem contribute

  DecLagrangianProblem(LagrangianProblem&L, const arr & z, const arr & xmask, OptOptions opt=NOOPT)
    : L(L)
    , mu(0.0) // first step done with 0 (to avoid fitting to a unset reference)
    , lambda(zeros(z.d0))
    , z(z)
    , xmask(xmask)
  {
    ScalarFunction::operator=([this](arr& dL, arr& HL, const arr& x) -> double {
      return this->decLagrangian(dL, HL, x);
    });
  }

  double decLagrangian(arr& dL, arr& HL, const arr& x) const;

  void updateADMM(const arr& x, const arr& z);
};
