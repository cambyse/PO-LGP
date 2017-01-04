/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */

#pragma once

#include <Core/array.h>
#include <Core/graph.h>
#include <Optim/optimization.h>

struct Variable;
typedef mlr::Array<Variable*> VariableL;

//===========================================================================

struct Function{
  virtual void fwd(arr& out, const arrA& in) = 0;
  /// for each input x, computes "dF/dx = dF/dout * del_out/del_x" or "Jin(i) = Jout * (del out / del in(i))"
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in) = 0;
};

//===========================================================================

struct Variable{
  Node *n;       ///< node in the graph structure
  Function *f;   ///< function

  uintA dim;     ///< dimensions of this variable

  arr value;     ///< value of his variable
  arrA in;       ///< (references of) input values to this variable (make list??)
  arr del, J;    ///< partial and total derivatives
  arrA Jin;      ///< buffer of push backs of total derivatives

  ObjectiveType ot; ///< declare if this an objective variable (cost, sumOfSqr, eq, ineq)

  Variable() : n(NULL), f(NULL), ot(OT_none) {}

  void write(ostream &os) const;
};

inline bool operator==(const Variable&, const Variable&){ return false; }
stdOutPipe(Variable)

//===========================================================================

struct Net : ConstrainedProblem{
  Graph G;

  //-- constructing the net
  Variable* newConstant(const char* key, const uintA& dim, bool isParameter); //(yes, here dim is required)
  Variable* newConstant(const char* key, const arr& value, bool isParameter);
  Variable* newFunction(const char* key, const VariableL& parents, Function* f, uintA dim, ObjectiveType ot=OT_none); //TODO: do you need to know dim here?

  //-- fwd and backward computation
  void fwdCompute();
  void bwdCompute();

  const arr& getValue(Variable *n); ///< query the value of a variable (e.g., output)
  void zeroAllPartialDerivatives(uint d); ///< set the partial derivative del_L/del_x of the loss w.r.t. a variable
  void setPartialDerivative(Variable *n, const arr& del); ///< set the partial derivative del_L/del_x of the loss w.r.t. a variable
  const arr& getTotalDerivative(Variable *n); ///< return the total derivative dL/dx w.r.t a variable

  void randParameters();
  arr getAllParameters();
  arr getAllParameterJacobians(uint ddim, uint wdim);
  void setAllParameters(const arr& w);
  void reportAllParameters();

  void checkAllDerivatives(Variable* out);

  virtual void phi(arr& phi, arr& J, arr& H, ObjectiveTypeA& ot, const arr& x);


  void write(ostream &os) const;

};
stdOutPipe(Net)

//===========================================================================

