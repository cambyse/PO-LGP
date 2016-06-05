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
#include "KOMO_Problem.h"

//-- basic converters
ScalarFunction     conv_cstylefs2ScalarFunction(double(*fs)(arr*, const arr&, void*),void *data);
VectorFunction     conv_cstylefv2VectorFunction(void (*fv)(arr&, arr*, const arr&, void*),void *data);
ScalarFunction     conv_VectorFunction2ScalarFunction(const VectorFunction& f);
ScalarFunction     conv_KOrderMarkovFunction2ScalarFunction(KOrderMarkovFunction& f);
VectorFunction     conv_KOrderMarkovFunction2VectorFunction(KOrderMarkovFunction& f);
ConstrainedProblem conv_KOrderMarkovFunction2ConstrainedProblem(KOrderMarkovFunction& f);
ConstrainedProblem conv_KOMO2ConstrainedProblem(struct KOMO_Problem& f);

/// this takes a constrained problem over $x$ and re-represents it over $z$ where $x=Bz$
ConstrainedProblem conv_linearlyReparameterize(const ConstrainedProblem& f, const arr& B);

/// A struct that allows to convert one function type into another, even when given as argument
struct Convert {
  KOrderMarkovFunction *kom;
  double(*cstyle_fs)(arr*, const arr&, void*);
  void (*cstyle_fv)(arr&, arr*, const arr&, void*);
  void *data;
  ScalarFunction sf;
  VectorFunction vf;
  ConstrainedProblem cpm;
  struct KOMO_ConstrainedProblem* komo;

  Convert(const ScalarFunction&);
  Convert(const VectorFunction&);
  Convert(KOrderMarkovFunction&);
  Convert(KOMO_Problem&);
  Convert(double(*fs)(arr*, const arr&, void*),void *data);
  Convert(void (*fv)(arr&, arr*, const arr&, void*),void *data);
  ~Convert();
  operator ScalarFunction();
  operator VectorFunction();
  operator ConstrainedProblem();
  operator KOrderMarkovFunction&();
};

