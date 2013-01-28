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


#ifndef MT_calibration_h
#define MT_calibration_h

#include "array.h"

void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P, bool verbose);
double projectionError(const arr& P, const arr& x, const arr& X);
void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X);
void stereoTriangulation(arr& X, const arr& xL, const arr& xR, const arr& PL, const arr& PR);


void stereoTriangulation_nonhom(arr& X_3d, const arr& x_4d, const arr&PL, const arr& PR);
void stereoTriangulation_nonhom(arr& X, const arr& x);

#ifdef  MT_IMPLEMENTATION
#  include "calibration.cpp"
#endif

#endif