/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de

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

#pragma once

#include "optimization.h"

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert {
  KOrderMarkovFunction *kom;
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void *data;
  ScalarFunction sf;
  VectorFunction vf;
  ConstrainedProblem cp;
  ConstrainedProblemMix cpm;

  Convert(const ScalarFunction&);
  Convert(const VectorFunction&);
  Convert(KOrderMarkovFunction&);
  Convert(double(*fs)(arr*, const arr&, void*),void *data);
  Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data);
  ~Convert();
  operator ScalarFunction();
  operator VectorFunction();
  operator ConstrainedProblem();
  operator ConstrainedProblemMix();
  operator KOrderMarkovFunction&();
};

//-- low level converters
ScalarFunction convert_cstylefs_ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data);
VectorFunction convert_cstylefv_VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data);
ScalarFunction convert_VectorFunction_ScalarFunction(const VectorFunction& f);
VectorFunction convert_KOrderMarkovFunction_VectorFunction(KOrderMarkovFunction& f);
ConstrainedProblem convert_KOrderMarkovFunction_ConstrainedProblem(KOrderMarkovFunction& f);
ConstrainedProblemMix convert_KOrderMarkovFunction_ConstrainedProblemMix(KOrderMarkovFunction& f);
