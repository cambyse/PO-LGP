/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */
#ifndef MLR_MinSumGaussNewton_h
#define MLR_MinSumGaussNewton_h

#include <fstream>
#include <Core/array.h>

struct Fij { arr A, B, C, a, b; double hata; };
struct Mu { arr M, m;       double hatm; };

struct MinSumGaussNewton {
  arr x;
  arr dampingReference;
  double tolerance, maxStep, damping;
  
  uintA Msgs; //edges: nx2 array for n edges
  /* Actually: this is not the edge set but rather the set of message indeces:
     it includes forward and backward tuples (i,j) and (j,i) and also (i,i) to index node potentials
     (message mu_{i\to i})
     
     Both arrays 'fij' and 'mu' below are indexed this way. This is clear for mu.
     For fij: we compute the pair-wise potential fij twice: once for usage
     in the backward message (update equation) and once for usage in the forward update equation.
     Why this redundant computation and storage? Because it simplifies the code.
     */
  mlr::Array<uintA> del; //in-neighbors
  mlr::Array<Fij> fij;
  mlr::Array<Mu>  mu;
  boolA clamped;
  std::ofstream fil;

  void setUndirectedGraph(uint n,const uintA& E);
  
  //indirect GaussNewton type problem interface:
  virtual void Psi(arr& psi, arr& psiI, arr& psiJ, uint i, uint j, const arr& x_i, const arr& x_j){ throw("NIY"); }
  //direct factor type proble interface:
  virtual double f(uint i, uint j, const arr& x_i, const arr& x_j);
  virtual void reapproxPotentials(uint i, const arr& hat_x_i);

  void updateMessage(uint m);
  void updateMessagesToNode(uint i);
  double totalCost(bool verbose=false);
  void init();
  void step(uint steps);
  double updateNode(uint i);
};

#ifdef  MLR_IMPLEMENTATION
#  include "MinSumGaussNewton.cpp"
#endif

#endif
