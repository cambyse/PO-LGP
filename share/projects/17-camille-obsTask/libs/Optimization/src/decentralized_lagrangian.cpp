#include <Optimization/decentralized_lagrangian.h>
#include <Optimization/utils.h>

double DecLagrangianProblem::decLagrangian(arr& dL, arr& HL, const arr& x) const
{
  double l = L(dL, HL, x);

  arr delta = deltaZ(x);

  // value
  l += scalarProduct(lambda, delta) + 0.5 * mu * scalarProduct(delta, delta);

  // jacobian
  dL += lambda + mu * delta;

  // hessian
  add(HL, mu, admmVar, admmMask);

  return l;
}

arr DecLagrangianProblem::deltaZ(const arr& x) const
{
  arr delta = zeros(x.d0);
  for(uint i: admmVar)
  {
    auto I = var(i);
    if(I>0) // I < 0 indicates a no-contribution at this step
      delta(i) = x(i) - z(I);
  }
  return delta;
}

void DecLagrangianProblem::updateADMM(const arr& x, const arr& z)
{
    this->z = z;
    auto delta = deltaZ(x);
    lambda += mu * delta; // Is like doing gradient descent on the dual problem (mu is the step size, and x-z the gradient)

    if(mu==0.0) mu=1.0;
    //else mu *= 2.0; // updating mu in a principeld way (increase, decrease) described in
    //Distributed Optimization and Statistical
    //Learning via the Alternating Direction
    //Method of Multipliers (p.20)
}
