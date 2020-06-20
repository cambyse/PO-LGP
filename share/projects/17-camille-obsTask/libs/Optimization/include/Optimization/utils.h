#pragma once

#include <Core/array.h>

/**
 * @brief Add mu to the diagonal elements specified by var and mask
 * @param HL
 * @param mu
 * @param admmVar
 * @param admmMask
 */
inline void add(arr& HL, double mu, const intA& admmVar, const arr& admmMask)
{
  if(isSparseMatrix(HL))
  {
    const auto Hs = dynamic_cast<rai::SparseMatrix*>(HL.special);
    const auto& elems = Hs->elems;
    const auto& Z = Hs->Z;

    intA incremented; incremented.reserve(admmVar.d0);

    for(uint k=0; k < elems.d0; k++)
    {
      const auto i = elems.p[2*k];
      const auto j = elems.p[2*k+1];
      if(i == j && admmMask(i))
      {
        Z.elem(k) += mu;
        incremented.append(i);
      }
    }

    if(incremented.d0 != admmVar.d0)
    {
      // slow, quadratic, should happen rarely!!
      for(auto i: admmVar)
      {
        bool found = false;
        for(auto j: incremented)
        {
          if(i==j) found = true;
        }
        if(!found) Hs->addEntry(i, i) = mu;
      }
    }
  }
  else // not sparse
  {
    for(auto i: admmVar)
    {
      HL(i, i) += mu;
    }
  }
}

inline double sparsity(arr & H)
{
  if(isSparseMatrix(H))
  {
    auto Hs = dynamic_cast<rai::SparseMatrix*>(H.special);

    return double(Hs->elems.d0) / (H.d0 * H.d0);
  }
  else
  {
    return H.sparsity();
  }

  return 0.0;
}
