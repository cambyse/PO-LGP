#include <decentralized_lagrangian.h>

double DecLagrangianProblem::decLagrangian(arr& dL, arr& HL, const arr& x) const
{
  double l = L(dL, HL, x);

  arr delta = x - z;
  for(uint i=0;i<delta.N;i++) delta.elem(i) *= xmask.elem(i);

  // value
  l += scalarProduct(lambda, delta) + 0.5 * mu * scalarProduct(delta, delta);

  // jacobian
  dL += lambda + mu * delta;

  // hessian
  if(isSparseMatrix(HL))
  {
    auto Hs = dynamic_cast<rai::SparseMatrix*>(HL.special);
    for(auto i = 0; i < x.d0; ++i)
    {
      if(xmask(i)) Hs->elem(i, i) += mu;
    }
  }
  else
  {
    for(auto i = 0; i < x.d0; ++i)
    {
      if(xmask(i)) HL(i, i) += mu;
    }
  }

  return l;
}

void DecLagrangianProblem::updateADMM(const arr& x, const arr& z)
{
    this->z = z;
    lambda += mu * (x - z); // Is like doing gradient descent on the dual problem (mu is the step size, and x-z the gradient)

    if(mu==0.0) mu=1.0;
    //else mu *= 2.0; // updating mu in a principeld way (increase, decrease) described in
    //Distributed Optimization and Statistical
    //Learning via the Alternating Direction
    //Method of Multipliers (p.20)
}
