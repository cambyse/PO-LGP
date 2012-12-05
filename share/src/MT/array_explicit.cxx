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
