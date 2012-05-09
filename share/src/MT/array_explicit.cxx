/*  Copyright (C) 2000, 2009  Marc Toussaint (mtoussai@cs.tu-berlin.de)
    see the `util.h' file for a full copyright statement  */

/* Explicit instantiations for array.h */

#define MT_IMPLEMENT_TEMPLATES

#include "array.h"
#include "util.h"

#define T double
#include <MT/array_instantiate.cxx>
#undef T

#define T int
#include <MT/array_instantiate.cxx>
#undef T

#define T uint
#include <MT/array_instantiate.cxx>
#undef T

#define T char
#include <MT/array_instantiate.cxx>
#undef T

#define T unsigned char
#include <MT/array_instantiate.cxx>
#undef T

#define T float
#include <MT/array_instantiate.cxx>
#undef T

template void MT::save<MT::Array<double> >(MT::Array<double> const&, char const*);
template unsigned int MT::getParameter<unsigned int>(char const*, unsigned int const&);
// template unsigned int MT::getParameter<int>(char const*, const int&);
// template unsigned int MT::getParameter<double>(char const*, const double&);
