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

struct Variable;
typedef mlr::Array<Variable*> VariableL;

//===========================================================================

struct Function{
  virtual void fwd(arr& out, const arrA& in) = 0;
  virtual void bwd(arrA& Jin, const arr& Jout, const arr& out, const arrA& in) = 0;
};

//===========================================================================

struct Variable{
  Node *n;       ///< node in the graph structure
  Function *f;   ///< function
  Type *type;    ///< true type of function

  uintA dim;     ///< dimensions of this variable

  arr value;     ///< value of his variable
  arrA in;       ///< (references of) input values to this variable (make list??)
  arr del, J;    ///< partial and total derivatives
  arrA Jin;      ///< buffer of push backs of total derivatives

  Variable() : n(NULL), f(NULL), type(NULL) {}

  void write(ostream &os) const;
};

inline bool operator==(const Variable&, const Variable&){ return false; }
stdOutPipe(Variable)

//===========================================================================


struct Net{
  Graph G;

  //-- constructing the net
  Variable* newConstant(const char* key, const uintA& dim);
  Variable* newFunction(const char* key, const VariableL& parents, Function* f, uintA dim);

  //-- fwd and backward computation
  void fwdCompute();
  void bwdCompute();

  const arr& getValue(Variable *n); ///< query the value of a variable (e.g., output)
  void zeroAllPartialDerivatives(uint d); ///< set the partial derivative del_L/del_x of the loss w.r.t. a variable
  void setPartialDerivative(Variable *n, const arr& del); ///< set the partial derivative del_L/del_x of the loss w.r.t. a variable
  const arr& getTotalDerivative(Variable *n); ///< return the total derivative dL/dx w.r.t a variable

  void randConstants();
  arr getAllConstants();
  arr getAllConstantJacobians(uint ddim, uint wdim);
  void setAllConstants(const arr& w);
  void reportAllConstants();

  void write(ostream &os) const;

};
stdOutPipe(Net)

//===========================================================================

